%S2B_RECIEVER DVB-S2 receiver: acquisition through symbol/phase correction, SNR/RSSI, hand-off.
%
%   Receives front-end-processed samples over TCP from
%   S2a_RFAcquisition.m, which owns acquisition (the USRP when
%   config.useSDR = true, the simulated channel model when false) plus
%   the per-sample front end that used to live here -- DC blocking, RSSI
%   measurement and AGC. Each chunk arrives as one framed message
%   carrying both the samples and their RSSI
%   (Functions/Serialization/dvbs2SerializeAcqChunk.m).
%
%   From there this script runs a chunk-buffered receive chain:
%   raw-sample-domain coarse CFO compensation
%   (Functions/dvbs2RawCFOCompensate.m, before the matched filter, to
%   protect Gardner timing recovery) -> matched filter + Gardner symbol
%   timing sync -> PL frame sync (with the weak-lock retry/widen
%   fallback) -> per-frame coarse CFO refinement on the SOF -> PLHEADER
%   decode -> pilot-aided (or blind) fine CFO/phase correction.
%
%   Deliberately stops BEFORE LDPC/BCH decode
%   (Functions/AEC_dvbs2BitRecover.m) -- that belongs to the
%   bit-recovery/BER-PER measurement stage. Instead, for each corrected
%   PLFRAME this script:
%     1) Estimates SNR (Functions/dvbs2SNREstimate.m, pilot-residual
%        based). RSSI arrives already measured from S2a, which is the
%        only place it CAN be measured -- it has to be taken before the
%        AGC stage that now lives there, since AGC deliberately erases
%        absolute power information.
%     2) Hands the corrected symbols + PLHEADER metadata onward over TCP
%        (Functions/dvbs2SerializePLFrame.m).
%     3) Periodically reports the RAW per-frame SNR samples plus RSSI
%        back for ACM (Functions/dvbs2SerializeFeedback.m), throttled to
%        every config.feedbackEveryKFrames frames but batching every
%        sample so none are hidden. This script forms no opinion about
%        MODCOD at all -- the transmitter owns the SNR statistics
%        (mean, variance, trend) and the whole policy, and smoothing
%        here would destroy the variance it needs.
%
%   In SDR mode, a run of config.calibLockFramesRequired CONSECUTIVE
%   successfully decoded frames is treated as link-establishment proof
%   (S1a_Transmitter.m sends calibration bursts until it hears back)
%   rather than real data: none of them are ever forwarded to S3, and
%   the feedback for the final one is sent immediately instead of
%   waiting for the normal throttled cadence.
%
%   Run this alongside the transmitter and processing-unit scripts (and,
%   when config.useSDR = true, S2a_RFAcquisition.m), each as its own
%   MATLAB instance, IN ANY ORDER (see dvbs2TCPConnectRetry.m).

clear; clc;

% Functions/ is organized into subfolders by purpose (core DSP/PHY
% functions stay directly under Functions/; TCP/, Serialization/, and
% Testbed/ hold this testbed's supporting code) -- genpath adds all of
% them recursively, computed from this script's own location so it
% works regardless of MATLAB's current folder when this is run.
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Functions')));

config = dvbs2TestbedConfig();

% Reset dvbs2MatchedFilterTimingSync's and dvbs2RawCFOCompensate's
% persistent System-object state, so a fresh run doesn't inherit
% timing-loop/CFO-compensator state from a previous run in the same
% MATLAB session.
clear dvbs2MatchedFilterTimingSync;

plHeaderSymbols = 90;
sps = config.dvbs2.SamplesPerSymbol;
Fsym = config.chanBW / (1 + config.dvbs2.RolloffFactor);
Fsamp = Fsym * sps;

sofRef = dvbs2SOFReference();
searchLen = 1000;
weakLockStreak = 0;

% dvbs2MatchedFilterTimingSync's Gardner loop has no stable equilibrium
% when there's no real signal to track -- its timing estimate drifts
% indefinitely rather than settling, and since that state persists
% across calls, the drift accumulates without bound until it eventually
% exceeds comm.SymbolSynchronizer's fixed, non-tunable
% MaxOutputExpansionFactor and starts silently truncating its own
% output. Periodically forcing a fresh start (below) bounds how far it
% can drift before that happens, regardless of chunk size.
% This 50-chunk reset stays as a second line of defence. It now covers a
% DIFFERENT failure from the one above: a timing loop that is locked and
% producing sane output, but where no SOF is ever found (wrong MODCOD,
% carrier gone, threshold set too high). The saturation check catches
% runaway on the very chunk it happens, so this no longer has to.
chunksSinceLock = 0;
resyncResetChunks = 50;

% Consecutive chunks the timing loop has come back unusable. Used only to
% keep the console readable -- one line when a runaway starts and one when
% it clears, rather than one per chunk for the whole episode.
syncLostLockStreak = 0;

% PLHEADERs thrown out by dvbs2FrameAcceptable because S1a could not have
% transmitted them. Worth watching as a rate rather than a total: a few
% per run is the Reed-Muller decoder losing to noise, which is the point;
% a steady stream means the two ends disagree about what is being sent.
rejectCount = 0;

% Frames where the pilot and PLHEADER SNR estimates disagreed by more than
% config.snr.maxDisagreementDB. Each one is a frame whose pilot-based
% number would previously have gone to S1a's ACM policy unchallenged.
snrDisagreeCount = 0;

% Run totals for the final summary. A 2-minute run scrolls hundreds of
% per-frame lines past, so these are what make the profile self-contained:
% which MODCODs were really received, whether the two SNR references agreed,
% which one ended up being reported, and how often the loops had to recover.
plsHistogram   = zeros(1, 128);   % indexed PLS+1, so PLS 0 lands at 1
snrSrcPilots   = 0;
snrSrcHeader   = 0;
snrDiffLog     = [];
snrHdrPhaseLog = [];
residHzLog     = [];
residPhaseLog  = [];
hdrResidLog    = [];
syncRunawayCount = 0;

% Dummy PLFRAMEs seen. These are deliberate filler from S1a, not errors --
% they mean the carrier is up but that frame carried no data. Counted rather
% than warned about, since at the top of the ACM ladder there can be
% hundreds per run.
dummyCount = 0;

%% RX acquisition setup
% Samples arrive from S2a_RFAcquisition.m in BOTH modes -- it owns
% acquisition (the radio, or the simulated channel model) as well as the
% per-sample front end, so this script begins at the raw-sample-domain
% CFO stage. Keeping acquisition in its own process is what lets the
% USRP's host-side buffer be drained promptly, rather than coupling
% radioRx()'s cadence to how long a chunk's DSP takes here.
fprintf('S2b: opening RF acquisition server on port %d, waiting for S2a to connect ...\n', ...
    config.rfAcqPort);
acqServer = dvbs2TCPServerRetry(config.rfAcqHost, config.rfAcqPort, "S2b's RF acquisition server");
while ~acqServer.Connected
    pause(0.1);
end
fprintf('S2b: S2a connected.\n');

%% Feedback client, connecting to the transmitter's ACM feedback server
fprintf('S2b: connecting to S1a''s ACM feedback server ...\n');
feedbackClient = dvbs2TCPConnectRetry(config.feedbackHost, config.feedbackPort, "S1a's feedback server");
fprintf('S2b: connected to S1a.\n');

%% Frame hand-off client, connecting to the processing unit's frame server
% CONNECT DURING INITIALISATION, IN BOTH MODES.
%
% In SDR mode this used to be deferred until the link was confirmed, which
% quietly coupled the entire RETURN path to forward-link acquisition: S3
% blocks waiting for this connection, and only once it arrives does S3
% connect to the retransmit-request server. That connection therefore
% landed tens of seconds into the run, by which time S2a was already inside
% its tight main loop. tcpserver's Connected property only updates when the
% event loop turns, so it never flipped, and S2a's drain -- gated on that
% property -- read nothing. 134 well-formed retransmit requests sat unread
% in the socket buffer for a whole run.
%
% Opening every socket here instead means all of them are established while
% the four processes are still initialising and yielding.
%
% THE SOCKET IS NOT THE LINK. linkEstablished still decides what is SENT:
% it stays false until the calibration run in the main loop below confirms
% the PHY link, and no frame is written to S3 before that. S3 would discard
% anything arriving earlier anyway, so this keeps that traffic off the wire
% rather than relying on the far end to throw it away.
fprintf('S2b: connecting to S3''s frame server ...\n');
frameClient = dvbs2TCPConnectRetry(config.frameHost, config.framePort, "S3's frame server");
fprintf('S2b: connected to S3.\n');
linkEstablished = ~config.useSDR;

%% Buffer initialization
rxBuffer = [];
frameSeqNum = 0;
chunkNum = 0;
snrBatch = [];        % raw per-frame SNR samples awaiting the next feedback report
lastSentMean = NaN;   % statistics of the last report, for the change triggers
lastSentSigma = NaN;
lastReportTic = tic;  % heartbeat clock
calibLockCount = 0;    % consecutive successfully-decoded calibration frames since the last reset (see config.calibLockFramesRequired)
cfoState = [];         % dvbs2CFOTracker state; [] until the first measurement initialises it
% Previous frame's length, used as the CFO tracker's dt. Seeded with a
% nominal MODCOD-1 normal PLFRAME rather than left empty: the tracker is
% called on every successful frame-sync lock, but this is only updated on
% a fully DECODED frame, so a run of locks that fail to decode would
% otherwise leave dt unset. A near-zero fallback dt makes the rate update
% (beta*miss/dt) explode -- observed once as a learned rate of 6.6e7 Hz/s
% from a 2 us fallback, which then poisons every later prediction.
prevFrameLenSym = 33282;

fprintf('\nS2b: starting receive loop ...\n');

% Real-time accounting: airtimeSec is how many seconds of RF this process
% has processed, wall-clock is how long that took. Their ratio is the
% real-time factor -- above 1.0 means this process is the bottleneck and
% is applying backpressure all the way up to the radio.
runTic = tic;
airtimeSec = 0;

% Set when the inner chunk-wait loop below detects S2a's connection has
% actually closed -- lets the top-of-loop guard stop immediately rather
% than waiting out the rest of runDurationSec on a dead link.
forceStop = false;

% Deliberately not gated on config.maxFrames: the transmitter only
% treats that as a NEW-DATA budget and keeps running afterward to
% service retransmit requests (selective-repeat ARQ), so this script
% needs to keep relaying for as long as the transmitter might still be
% sending -- it stops via the existing disconnect detection below (the
% simulated-channel link closing, or, on real SDR hardware with no such
% concept, only via a manual stop) rather than a frame count, since a
% count would conflate original and retransmitted frames and could stop
% relaying before a legitimate resend ever arrives.
while true

    %% Stop after config.runDurationSec so all four scripts end together
    if forceStop || toc(runTic) >= config.runDurationSec
        fprintf('\n=== S2b PROFILE === %.1f s wall | %.1f s airtime | RT factor %.3f\n', ...
            toc(runTic), airtimeSec, toc(runTic)/max(airtimeSec,eps));
        fprintf('  chunks %d | frames decoded %d (%.1f%% of chunks)\n', ...
            chunkNum, frameSeqNum, 100*frameSeqNum/max(chunkNum,1));

        % Everything below exists so this summary alone is diagnostic. The
        % per-frame lines scroll past in a long run, so the counters that
        % matter for the parts under active development are totalled here.

        % WHICH MODCODs WERE ACTUALLY RECEIVED. The PLS code packs as
        % MODCOD*4 + short*2 + pilots, so MODCOD = floor(PLS/4). This is the
        % line that answers "did the ACM ladder actually climb, and how far".
        seenPLS = find(plsHistogram > 0);
        if ~isempty(seenPLS)
            fprintf('  MODCODs received:');
            for p = seenPLS
                fprintf('  %d(%s)x%d', floor((p-1)/4), ...
                    localModName(floor((p-1)/4)), plsHistogram(p));
            end
            fprintf('\n');
        end

        % SNR CROSS-CHECK. Two independent references measured per frame;
        % the higher is reported because both can only ever be biased low.
        % A high disagreement rate means one reference is being corrupted --
        % which one is shown by how often each won.
        if frameSeqNum > 0
            fprintf(['  SNR cross-check: %d disagreements >%.0f dB (%.1f%% of frames)' ...
                ' | reported from pilots %d, header %d | median |diff| %.2f dB\n'], ...
                snrDisagreeCount, config.snr.maxDisagreementDB, ...
                100*snrDisagreeCount/frameSeqNum, snrSrcPilots, snrSrcHeader, ...
                median([snrDiffLog NaN], 'omitnan'));
            fprintf('  header phase offset: median %+.1f deg | residual freq: median %+.1f Hz\n', ...
                median([snrHdrPhaseLog NaN], 'omitnan'), ...
                median([residHzLog NaN], 'omitnan'));

            % THE PHASE-MARGIN LINE. AnchorResidual is what the straight-line
            % model could not explain -- i.e. what the demodulator still
            % sees. Compared here against each constellation's decision
            % boundary, because that is what decides whether the upper ACM
            % rungs are reachable at all:
            %
            %   QPSK  +-45.0 deg    16APSK +-22.5 deg
            %   8PSK  +-22.5 deg    32APSK +-11.25 deg
            %
            % and against the equivalent-SNR ceiling it imposes. A phase
            % error theta acts like added noise of variance 2(1-cos theta),
            % so no matter how clean the link gets, the effective SNR cannot
            % exceed -10*log10(2(1-cos theta)). At 10 deg that ceiling is
            % 14.7 dB, which is below what 32APSK 4/5 needs to run cleanly.
            residRMS = median([residPhaseLog NaN], 'omitnan');
            if ~isnan(residRMS)
                ceilingDB = -10*log10(max(2*(1-cosd(residRMS)), eps));
                fprintf(['  PHASE MARGIN: fit residual median %.1f deg RMS ' ...
                    '(header %.1f deg) -> effective-SNR ceiling %.1f dB\n'], ...
                    residRMS, median([hdrResidLog NaN], 'omitnan'), ceilingDB);
                fprintf('                boundary usage: QPSK %.0f%% | 8PSK/16APSK %.0f%% | 32APSK %.0f%%\n', ...
                    100*residRMS/45, 100*residRMS/22.5, 100*residRMS/11.25);
            end
        end

        fprintf(['  timing-loop runaways %d | frames rejected by PLHEADER checks %d' ...
            ' | dummy PLFRAMEs %d (%.1f%% of all frames)\n\n'], ...
            syncRunawayCount, rejectCount, dummyCount, ...
            100*dummyCount/max(frameSeqNum + dummyCount, 1));
        break;
    end

    %% Acquire one chunk from S2a, without blocking indefinitely
    % Already DC-blocked, power-measured and AGC'd there -- this arrives
    % as one framed message carrying the samples and their RSSI together
    % (see dvbs2SerializeAcqChunk.m for why that link is framed).
    %
    % NON-BLOCKING, WITH ITS OWN DURATION CHECK -- same fix, same reason,
    % as S3's frame-read loop. A plain blocking dvbs2TCPFrameRead here can
    % only be woken by a new chunk arriving or the socket actually
    % throwing ConnectionClosed -- and neither is guaranteed once S2a has
    % itself stopped: none of these scripts calls exit, so a finished
    % script's MATLAB process just idles at the prompt with its sockets
    % still open. That means "the connection closes" never actually
    % happens, and without this fix the runDurationSec guard above is
    % unreachable whenever S2a finishes first -- which, once the launcher
    % starts everything together, is every run.
    while true
        acqBytes = dvbs2TCPFrameTryRead(acqServer);
        if ~isempty(acqBytes)
            break;
        end
        if ~acqServer.Connected
            fprintf('S2b: RF acquisition link closed by S2a; stopping.\n');
            forceStop = true;
            break;
        end
        if toc(runTic) >= config.runDurationSec
            break;   % let the top-of-loop guard above print the profile and stop
        end
        pause(0.005);
    end
    if isempty(acqBytes)
        continue;
    end
    % RSSI needs hardware-specific calibration to convert to dBm on real
    % USRPs; carried here as a relative dB figure. chunkFreqOffsetEst is
    % the block-level raw CFO S2a already REMOVED from these samples --
    % every frame drawn from this chunk shares whatever residual that
    % single feed-forward estimate left behind, so it is logged here to
    % check whether it lines up with any pattern in the per-frame
    % PLHEADER decode results downstream.
    [rssiDB, chunkFreqOffsetEst, newSamples_cfocomp] = dvbs2DeserializeAcqChunk(acqBytes);

    chunkNum = chunkNum + 1;
    airtimeSec = airtimeSec + numel(newSamples_cfocomp)/Fsamp;
    fprintf('S2b: chunk %d | RSSI=%.2f dB | rawCFO(S2a)=%.1f Hz | RT factor %.3f\n', ...
        chunkNum, rssiDB, chunkFreqOffsetEst, toc(runTic)/max(airtimeSec,eps));

    % Match filter & timing sync
    [newSamples_sync, ~, syncLostLock] = dvbs2MatchedFilterTimingSync( ...
        newSamples_cfocomp, sps, config.dvbs2.RolloffFactor, 7);

    % A runaway timing loop has already been reset inside the function and
    % returned nothing. What still has to happen here is discarding
    % rxBuffer: the runaway ramps up over several chunks, so by the time
    % it is detectable the buffer is full of symbols recovered at a
    % drifting, meaningless sample phase. Searching those for a SOF only
    % wastes time and risks a false correlation peak.
    %
    % Deliberately NOT a `continue` -- the rest of the loop body still
    % needs to run so chunksSinceLock keeps counting and, more
    % importantly, so the ACM feedback heartbeat at the bottom keeps
    % reaching S1a while the receiver is re-acquiring.
    if syncLostLock
        if syncLostLockStreak == 0
            syncRunawayCount = syncRunawayCount + 1;
            fprintf(['S2b: timing loop ran away at chunk %d -- resetting Gardner, ' ...
                'discarding %d buffered symbols.\n'], chunkNum, numel(rxBuffer));
        end
        syncLostLockStreak = syncLostLockStreak + 1;
        rxBuffer = [];
    else
        if syncLostLockStreak > 0
            fprintf('S2b: timing sync recovered after %d bad chunk(s).\n', ...
                syncLostLockStreak);
            syncLostLockStreak = 0;
        end
        rxBuffer = [rxBuffer; newSamples_sync]; %#ok<AGROW>
    end

    frameFoundThisChunk = false;

    % Inner loop: locate and process every complete PLFRAME currently
    % buffered before requesting the next chunk.
    while length(rxBuffer) > searchLen + length(sofRef) - 1

        [idx, peak, ~] = dvbs2FrameSync(rxBuffer, sofRef, searchLen);
        if peak < config.frameSyncPeakThreshold
            weakLockStreak = weakLockStreak + 1;
            if weakLockStreak < 3
                % Possibly a minor alignment artifact rather than
                % genuine noise: nudge forward a little and retry
                % before giving up on this whole search window.
                rxBuffer(1:min(100, length(rxBuffer))) = [];
                continue;
            end
            rxBuffer(1:searchLen) = [];
            searchLen = 2500;
            weakLockStreak = 0;
            calibLockCount = 0;   % lost lock -- any in-progress consecutive-frame count no longer stands
            continue;
        else
            searchLen = 100;
            weakLockStreak = 0;
        end

        coarseFreqEstHz = dvbs2CoarseFreqEst(rxBuffer, idx, sofRef, Fsym);
        if (idx + plHeaderSymbols - 1) > length(rxBuffer)
            break;   % wait for the next chunk
        end

        % Closed-loop CFO tracking. The tracker is fed the TOTAL offset --
        % what S2a already removed plus what this frame's SOF still sees --
        % rather than the residual alone, because the total is the
        % physically stable quantity. The split between the two moves
        % whenever S2a's blind estimate changes FFT bin, which would look
        % to the tracker like a real frequency jump; their sum does not.
        %
        % dtSec is the previous frame's airtime, i.e. the spacing between
        % this measurement and the last. Exact while the receiver is
        % locked (idx == 1); slightly short after a resync skipped some
        % symbols, which only matters for the rate term.
        totalMeasHz = chunkFreqOffsetEst + coarseFreqEstHz;
        dtSec = prevFrameLenSym / Fsym;
        [totalTrackedHz, cfoState] = dvbs2CFOTracker(totalMeasHz, dtSec, cfoState, config);
        appliedFreqHz = totalTrackedHz - chunkFreqOffsetEst;

        rawHeader = rxBuffer(idx:idx+plHeaderSymbols-1);
        n_header = (0:plHeaderSymbols-1).';
        header_coarse = rawHeader .* exp(-1j*2*pi*(appliedFreqHz/Fsym)*n_header);

        % Remove the residual CONSTANT phase offset before decoding the
        % PLSC. dvbs2CoarseFreqEst above only corrects the frequency
        % RAMP -- it leaves whatever absolute phase the frame happened
        % to arrive with. The PLSC's pi/2-BPSK symbols are decoded
        % coherently (satcom.internal.dvbs.plHeaderRecover correlates
        % against a candidate codeword table), so an uncorrected phase
        % rotates every PLSC symbol away from its reference and biases
        % that correlation systematically rather than randomly.
        %
        % The 26-symbol SOF is a known, fixed pattern (identical in
        % every PLFRAME regardless of MODCOD), so correlating the
        % received SOF against sofRef measures that offset directly.
        %
        % WHY THIS MATTERS: the PLSC is protected by an RM(32,6) code
        % with minimum distance 16, so a single-bit PLSC error is
        % essentially impossible from noise at a healthy SNR -- yet the
        % FECFRAME normal/short bit was observed flipping repeatedly on
        % a live link (PLS 5 decoding as PLS 7 on a fixed MODCOD-1
        % normal-FECFRAME pilots-on stream, i.e. exactly one bit), which
        % is the signature of a systematic bias, not noise. That
        % misdecode then cascades: wrong FECFrameLength -> wrong
        % dvbs2FrameLength -> wrong dvbs2PilotStructure pilot indices ->
        % meaningless SNR estimate and a failed LDPC/BBHEADER decode
        % downstream in S3. See dvbs2PLHeaderRecover.m's header comment,
        % which describes this same failure.
        phOffset = angle(sum(header_coarse(1:length(sofRef)) .* conj(sofRef)));
        header_coarse = header_coarse .* exp(-1j*phOffset);

        try
            % dvbs2PLHeaderRecover.m skips the ambiguous DVB-S2/DVB-S2X
            % auto-detection MathWorks' equivalent function performs --
            % this testbed never transmits DVB-S2X, so that guess is
            % pure risk with no upside here; see its header comment for
            % the full rationale.
            phyParams = dvbs2PLHeaderRecover(header_coarse);
        catch ME
            % This script runs indefinitely -- an uncaught exception
            % here must not crash the whole receiver over one bad frame.
            warning('S2b:HeaderDecodeFailed', 'PLHEADER decode threw: %s', ME.message);
            rxBuffer(1:idx) = [];
            calibLockCount = 0;   % a bad decode breaks the consecutive-good-frame streak
            continue;
        end

        if phyParams.IsDummyFrame
            % A dummy PLFRAME's PLSC carries no real MODCOD/FECFrame
            % encoding, so FECFrameLength/ModulationOrder aren't
            % meaningful inputs for dvbs2FrameLength.m/dvbs2PilotStructure.m
            % below. This testbed never transmits dummy frames on
            % purpose, so seeing one here means the PLSC decode landed
            % on the wrong nearest-neighbor codeword under noise --
            % skip this lock and keep searching for the next SOF rather
            % than feeding it forward.
            % A DUMMY PLFRAME IS NOT AN ERROR ANY MORE.
            %
            % This branch used to warn and reset calibLockCount, on the
            % assumption that this testbed never transmits dummy frames, so
            % seeing one could only mean the PLSC had misdecoded. S1a now
            % sends them deliberately: when the ACM ladder climbs past the
            % point where real PLFRAMEs can be generated in real time, the
            % shortfall is filled with dummies so the carrier stays
            % continuous (see Functions/dvbs2DummyFiller.m).
            %
            % So the meaning inverts. A dummy says "the transmitter is up,
            % frame sync is working, there is simply no data in this frame"
            % -- which is the OPPOSITE of a bad decode, and must not break
            % the consecutive-good-frame streak that establishes the link.
            %
            % Three consequences, all handled here:
            %   - no warning; hundreds per run would bury everything else,
            %     so they are counted for the closing summary instead
            %   - calibLockCount is left alone rather than zeroed
            %   - frameFoundThisChunk is set, because the carrier IS being
            %     found; without it a stretch of filler would trip the
            %     50-chunk "no frame found" resync and reset a healthy loop
            %
            % A dummy PLFRAME is 90 header + 36 slots x 90 = 3330 symbols,
            % ALWAYS, whatever MODCOD real traffic is using -- so unlike the
            % misdecode paths below, the exact frame length is known and the
            % whole frame can be consumed. Advancing by idx alone would
            % leave the SOF search re-scanning the dummy's own payload.
            dummyCount = dummyCount + 1;
            dummySymbols = 3330;
            rxBuffer(1 : min(idx + dummySymbols - 1, numel(rxBuffer))) = [];
            frameFoundThisChunk = true;
            continue;
        end

        % PLAUSIBILITY REJECTION. Everything above this point only asks
        % whether the PLSC decoded to a STRUCTURALLY valid code. This asks
        % the stronger question: could S1a, as configured, have transmitted
        % it at all? A header describing no pilots, a short FECFRAME, or a
        % MODCOD outside acm.modcodSet is not a marginal frame to decode
        % carefully -- it is proof this decode is wrong, because the
        % transmitter has no way to produce one.
        %
        % Placed BEFORE dvbs2FrameLength so a rejected header never gets a
        % frame length computed from it. That matters: a short-FECFRAME
        % misdecode yields roughly a quarter of the true length, and
        % consuming that many symbols would resume the SOF search in the
        % middle of the payload.
        [frameOK, rejectReason] = dvbs2FrameAcceptable(phyParams, config);
        if ~frameOK
            rejectCount = rejectCount + 1;
            % Log the first few and then every 50th. A steady stream of
            % rejections means something real is wrong (wrong modcodSet,
            % or a transmitter sending what the receiver was told it
            % cannot); a trickle is the Reed-Muller decoder losing to
            % noise now and then, which is exactly what this catches.
            if rejectCount <= 5 || mod(rejectCount, 50) == 0
                fprintf(['S2b: frame rejected (%s) at chunk %d | PLS=%d ' ...
                    'peak=%.4f PLHdrConf=%.3f | %d rejected so far\n'], ...
                    rejectReason, chunkNum, phyParams.PLSDecimalCode, peak, ...
                    phyParams.DecodeMeanDistance - phyParams.DecodeMinDistance, ...
                    rejectCount);
            end
            rxBuffer(1:idx) = [];
            calibLockCount = 0;   % a bad decode breaks the consecutive-good-frame streak
            continue;
        end

        frameLength = dvbs2FrameLength(phyParams);
        if ~isfinite(frameLength) || frameLength <= plHeaderSymbols
            % A misdecoded PLSC can also land on a reserved/undefined
            % code outside the dummy-frame range (e.g. the unused
            % MODCOD values above 28 in plain DVB-S2) that still
            % produces a nonsensical frame length -- same handling as
            % any other invalid header: skip this lock and keep
            % searching rather than feeding it forward.
            warning('S2b:InvalidFrameLength', ...
                'PLSC decoded to an invalid/reserved code (PLS=%d, frameLength=%g); skipping.', ...
                phyParams.PLSDecimalCode, frameLength);
            rxBuffer(1:idx) = [];
            calibLockCount = 0;   % a bad decode breaks the consecutive-good-frame streak
            continue;
        end
        fp = dvbs2PilotStructure(phyParams, frameLength);

        if (idx + frameLength - 1) > length(rxBuffer)
            break;   % wait for the next chunk
        end

        rawFrame = rxBuffer(idx + plHeaderSymbols : idx + frameLength - 1);
        n_frame = (plHeaderSymbols : frameLength - 1).';
        % The SAME phOffset measured from the SOF above is applied here
        % too, so the header and the payload stay on ONE common phase
        % reference. Correcting only the header would leave a phase
        % discontinuity at the 90-symbol boundary, which
        % dvbs2PhaseCompensate.m would then try to fit a single phase
        % trajectory through -- its SOF anchor and its pilot-block
        % anchors would be sitting on different references.
        frame_coarse = rawFrame .* exp(-1j*(2*pi*(appliedFreqHz/Fsym)*n_frame + phOffset));
        plSymbols_coarse = [header_coarse; frame_coarse];

        % Built BEFORE the phase compensation, not after, because the
        % compensator now uses the full 90-symbol header as two fit anchors
        % rather than the SOF alone. No circularity: dvbs2PLHeaderRecover
        % has already run, so the decoded PLS code is available here.
        refHeader = dvbs2PLHeaderReference(phyParams);

        phaseInfo = struct;
        if phyParams.HasPilots
            [normCFO, ~] = dvbs2FineFreqEst(plSymbols_coarse, fp);
            n_fine = (0:length(plSymbols_coarse)-1).';
            plSymbols_fine = plSymbols_coarse .* exp(-1j*2*pi*normCFO*n_fine);
            % phaseInfo was previously discarded. It carries FitSlope --
            % the phase ramp per symbol that the compensator had to remove
            % -- which converts straight to the residual frequency error
            % the fine estimator left behind. That is the single most
            % useful number for diagnosing a pilot/header disagreement, and
            % it costs nothing because it is already computed.
            [plSymbols_corrected, phaseInfo] = dvbs2PhaseCompensate(plSymbols_fine, fp, refHeader);
        else
            plSymbols_corrected = dvbs2NonPilotFineFreqPhase(plSymbols_coarse, sofRef, phyParams);
        end

        %% SNR / noise-variance estimate for this frame
        % Two independent references: the pilots (precise, 792 symbols,
        % but spread across the whole frame and derived from the PLSC) and
        % the PLHEADER (reliable, 90 symbols, 270 us, needs no pilot
        % layout). See dvbs2SNREstimate.m for why both are worth having.
        [snrDB, noiseVarEstimate, snrInfo] = dvbs2SNREstimate( ...
            plSymbols_corrected, fp, refHeader, config);

        % Run totals for the closing summary.
        plsHistogram(phyParams.PLSDecimalCode + 1) = ...
            plsHistogram(phyParams.PLSDecimalCode + 1) + 1;
        if snrInfo.Source == "pilots"
            snrSrcPilots = snrSrcPilots + 1;
        else
            snrSrcHeader = snrSrcHeader + 1;
        end
        if ~isnan(snrInfo.DisagreementDB)
            snrDiffLog(end+1) = snrInfo.DisagreementDB; %#ok<SAGROW>
        end
        if ~isnan(snrInfo.HeaderPhaseDeg)
            snrHdrPhaseLog(end+1) = snrInfo.HeaderPhaseDeg; %#ok<SAGROW>
        end
        if isfield(phaseInfo, 'FitSlope')
            residHzLog(end+1) = phaseInfo.FitSlope * Fsym / (2*pi); %#ok<SAGROW>
        end
        % Residual phase the line fit could NOT explain, at the anchors. The
        % quantity that decides whether a dense constellation survives: a
        % 32APSK decision boundary is +-11.25 deg, so an RMS residual of a
        % few degrees is the whole margin.
        if isfield(phaseInfo, 'AnchorResidualRMSdeg')
            residPhaseLog(end+1) = phaseInfo.AnchorResidualRMSdeg; %#ok<SAGROW>
            hdrResidLog(end+1) = phaseInfo.HeaderResidualDeg;      %#ok<SAGROW>
        end

        if ~snrInfo.Trusted
            snrDisagreeCount = snrDisagreeCount + 1;
            % Residual frequency error implied by the phase trajectory the
            % compensator removed: FitSlope is rad/symbol, so
            % Hz = slope * Fsym / 2*pi.
            residHz = NaN;
            if isfield(phaseInfo, 'FitSlope')
                residHz = phaseInfo.FitSlope * Fsym / (2*pi);
            end
            fprintf(['S2b: SNR sources disagree by %.1f dB at chunk %d ' ...
                '(pilots %.2f, header %.2f) -- using header | ' ...
                'block trend %+.3f dB/blk | residual %+.1f Hz | %d so far\n'], ...
                snrInfo.DisagreementDB, chunkNum, snrInfo.PilotSNRdB, ...
                snrInfo.HeaderSNRdB, snrInfo.BlockTrendDB, residHz, ...
                snrDisagreeCount);
        end

        % Collect the RAW per-frame SNR for the next feedback message.
        % Deliberately NO smoothing and NO MODCOD lookup here: the
        % transmitter is the sole ACM authority and needs the sample
        % DISTRIBUTION, not just its centre -- it sizes its safety margin
        % from the measured standard deviation and anticipates fades from
        % the trend (dvbs2ACMPolicy.m). Averaging here would collapse
        % each batch to one number and destroy exactly that information
        % before it crossed the wire. noiseVarEstimate is separate and
        % stays raw: it is this specific frame's LDPC decode input.
        snrBatch(end+1) = snrDB; %#ok<SAGROW>

        if ~linkEstablished
            % Require several CONSECUTIVE successfully-decoded
            % calibration frames (config.calibLockFramesRequired), not
            % just the first one, before reporting link establishment --
            % a run of samples gives S1a a mean AND a spread to pick its
            % first real MODCOD from, rather than bootstrapping the whole
            % link off a single lucky/unlucky sample. Any decode failure
            % in between resets calibLockCount to 0 (see the reset sites
            % above), so this only fires after an unbroken run.
            calibLockCount = calibLockCount + 1;
            if calibLockCount < config.calibLockFramesRequired
                fprintf('S2b: calibration frame locked (%d/%d consecutive), SNR=%.2f dB -- waiting for a stable run before reporting link established\n', ...
                    calibLockCount, config.calibLockFramesRequired, snrDB);
            else
                % This can only be a real, successfully decoded run of
                % calibration frames from S1a (see S1a_Transmitter.m), so
                % it proves the PHY link works. Its content is never
                % forwarded to S3. Report feedback immediately rather
                % than waiting for the normal throttled cadence, then
                % connect to S3 for the real traffic that follows.
                %
                % Only the CONSECUTIVE run is reported: snrBatch grows by
                % exactly one entry per successful decode and a failed
                % decode never reaches this point, so its last
                % calibLockCount entries are precisely that unbroken run.
                calibSamples = snrBatch(end - calibLockCount + 1 : end);

                [cm, cs] = localBatchStats(calibSamples, config);
                feedback.Count = numel(calibSamples);
                feedback.MeanSNRdB = cm;
                feedback.SigmaSNRdB = cs;
                feedback.RSSIdB = rssiDB;
                dvbs2TCPFrameWrite(feedbackClient, dvbs2SerializeFeedback(feedback));
                snrBatch = [];
                lastSentMean = cm; lastSentSigma = cs; lastReportTic = tic;

                % The socket to S3 is already open (see initialisation
                % above); this only opens the VALVE. From here the else
                % branch below starts writing frames to it.
                linkEstablished = true;
                fprintf(['S2b: link established after %d consecutive calibration frames ' ...
                    '(mean SNR=%.2f dB, sigma=%.2f dB) -- forwarding frames to S3 from now on.\n'], ...
                    config.calibLockFramesRequired, mean(calibSamples), std(calibSamples));
            end
        else
            %% Hand off the corrected frame for bit recovery
            frameSeqNum = frameSeqNum + 1;
            plBytes = dvbs2SerializePLFrame( ...
                frameSeqNum, plSymbols_corrected, phyParams, frameLength, noiseVarEstimate);
            dvbs2TCPFrameWrite(frameClient, plBytes);

            % PLHeaderConfidence: gap between the best-matching PLSC
            % codeword's distance and the mean across all candidates
            % (see dvbs2PLHeaderRecover.m) -- a small value flags a
            % marginal header decode worth cross-checking against PLS if
            % anything still looks wrong downstream.
            plHeaderConfidence = phyParams.DecodeMeanDistance - phyParams.DecodeMinDistance;
            % CFO diagnostics: meas is this frame's raw SOF measurement of
            % the total offset, trk the tracker's filtered estimate, and
            % miss their difference. A run of same-signed misses means the
            % offset is genuinely sliding and the tracker is lagging it;
            % random signs are just measurement noise. '*' flags a
            % measurement the outlier gate rejected.
            if cfoState.LastRejected
                cfoFlag = '*';
            else
                cfoFlag = ' ';
            end
            fprintf(['S2b: frame %d | peak=%.4f | CFO meas=%+.1f trk=%+.1f miss=%+.1f%s Hz ' ...
                '(rate %+.1f Hz/s) | SNR=%.2f dB | RSSI=%.2f dB | PLS=%d | PLHdrConf=%.3f\n'], ...
                frameSeqNum, peak, totalMeasHz, totalTrackedHz, cfoState.LastMiss, cfoFlag, ...
                cfoState.Fdot, snrDB, rssiDB, phyParams.PLSDecimalCode, plHeaderConfidence);
        end

        frameFoundThisChunk = true;
        prevFrameLenSym = frameLength;   % sets the tracker's dt for the next frame
        rxBuffer(1 : idx + frameLength - 1) = [];
    end

    %% ACM feedback -- EVENT DRIVEN, not periodic.
    % Samples are collected every frame but reported only when the statistics
    % have actually moved, or when the heartbeat is due. The channel needs
    % 7-13 s to move by one MODCOD rung (see dvbs2TestbedConfig.m), so
    % reporting on every fifth frame was roughly fifty times more often than
    % anything could change.
    %
    % THIS RUNS PER CHUNK, NOT PER FRAME, AND THAT IS THE WHOLE POINT. It used
    % to live inside the frame-decoded branch above, which meant a receiver
    % that decoded NOTHING said nothing at all -- including no heartbeat.
    % Measured on hardware: 36.84 s between reports against a configured 2.0,
    % while S1a's linkLossSec is 5.0. So S1a declared the return link dead
    % precisely when the forward link was failing and it most needed to know.
    % Worse, the one signal designed to carry exactly that condition --
    % Count = 0, "alive but decoded nothing" -- was unreachable, because
    % reaching it required decoding a frame.
    %
    % Only after link establishment. Before that the calibration branch above
    % owns reporting, and a Count = 0 report arriving first would have S1a
    % establish the link on a mean SNR of zero.
    if linkEstablished
        [batchMean, batchSigma] = localBatchStats(snrBatch, config);
        sinceLast = toc(lastReportTic);

        % A CHANGE TRIGGER NEEDS MORE THAN ONE FRAME BEHIND IT.
        % dvbs2SNREstimate returns a wildly wrong value on roughly 4% of
        % otherwise perfect frames -- measured: peak 0.988, PLHdrConf 2.5,
        % correct PLS, SNR = -3.14 dB against a true 16.5 dB. A single such
        % frame satisfies "the mean moved", reports itself immediately, and
        % reaches the policy as a genuine sample. It shows up in S1a's log as
        % "mean 6.50 sigma 9.75" where the real spread is 0.3-0.5 dB -- and
        % since the MODCOD bound is mu - k*sigma, one bad estimate suppresses
        % the whole ladder. config.acm.outlierMADs is meant to catch this, but
        % a batch of ONE has no median absolute deviation to screen against.
        %
        % THE HEARTBEAT IS DELIBERATELY EXEMPT from that minimum. It proves
        % the receiver is alive and must fire on schedule whatever the batch
        % contains -- including nothing at all.
        enoughForChange = numel(snrBatch) >= config.acm.minFramesForChangeReport;

        changedMean  = enoughForChange && ~isnan(lastSentMean) && ...
            abs(batchMean - lastSentMean) > config.acm.reportDeltaMeanDB;
        changedSigma = enoughForChange && ~isnan(lastSentSigma) && ...
            abs(batchSigma - lastSentSigma) > config.acm.reportDeltaSigmaDB;
        heartbeatDue = sinceLast >= config.acm.heartbeatSec;
        firstReport  = isnan(lastSentMean);

        if firstReport || changedMean || changedSigma || heartbeatDue
            feedback.Count = numel(snrBatch);
            feedback.MeanSNRdB = batchMean;
            feedback.SigmaSNRdB = batchSigma;
            feedback.RSSIdB = rssiDB;
            dvbs2TCPFrameWrite(feedbackClient, dvbs2SerializeFeedback(feedback));

            if changedMean,  why = 'mean moved';
            elseif changedSigma, why = 'sigma moved';
            elseif firstReport,  why = 'first report';
            elseif feedback.Count == 0, why = 'heartbeat, DECODED NOTHING';
            else,                why = 'heartbeat';
            end
            fprintf('S2b: feedback sent (%s) | %d frames | mean %.2f sigma %.2f dB | %.2f s since last\n', ...
                why, feedback.Count, batchMean, batchSigma, sinceLast);

            lastSentMean = batchMean;
            lastSentSigma = batchSigma;
            lastReportTic = tic;
            snrBatch = [];
        end
    end

    if frameFoundThisChunk
        chunksSinceLock = 0;
    else
        chunksSinceLock = chunksSinceLock + 1;
        if chunksSinceLock >= resyncResetChunks
            fprintf('S2b: no frame found in %d chunks -- resetting matched filter/timing sync.\n', ...
                chunksSinceLock);
            clear dvbs2MatchedFilterTimingSync;
            chunksSinceLock = 0;
        end
    end
end

fprintf('S2b: stopping.\n');

function [m, s] = localBatchStats(samples, config)
%LOCALBATCHSTATS Mean and standard deviation of one batch, outliers removed.
%
%   Outliers are rejected HERE rather than shipped, because this is the only
%   place that still has the raw samples. Once a batch is summarised into a
%   mean and a sigma, one wild estimate has already contaminated both and the
%   transmitter cannot undo it. The logs show these do occur -- single frames
%   reading -3 dB in the middle of a run at 18 dB.
%
%   Uses the median absolute deviation rather than the standard deviation as
%   the yardstick: sigma is itself inflated by the very outlier being looked
%   for, whereas the MAD is not.
    if isempty(samples)
        m = 0; s = 0; return;
    end
    x = samples(:);
    if numel(x) >= 4
        med = median(x);
        mad = median(abs(x - med));
        if mad > 0
            keep = abs(x - med) <= config.acm.outlierMADs * 1.4826 * mad;
            if any(keep), x = x(keep); end
        end
    end
    m = mean(x);
    % POPULATION standard deviation (normalise by N, not N-1). The
    % transmitter recombines batches with
    %   sigma^2 = sum(ni*(sigmai^2 + mui^2))/n - mu^2
    % which is exact only for population variance; reporting the sample
    % version instead leaves a systematic sqrt(N/(N-1)) inflation -- about
    % 1.7% at 30 frames per batch. Small, but there is no reason to carry a
    % known bias when consistency costs one character.
    if numel(x) >= 2, s = std(x, 1); else, s = 0; end
end

function name = localModName(modcod)
%LOCALMODNAME Modulation for a legacy DVB-S2 MODCOD index, for the summary.
%   Ranges per ETSI EN 302 307-1 table 12. Used only to make the closing
%   "MODCODs received" line readable at a glance -- 13(8PSK) says more than
%   a bare 13 about whether the ACM ladder actually left QPSK.
    if     modcod >= 1  && modcod <= 11, name = 'QPSK';
    elseif modcod >= 12 && modcod <= 17, name = '8PSK';
    elseif modcod >= 18 && modcod <= 23, name = '16APSK';
    elseif modcod >= 24 && modcod <= 28, name = '32APSK';
    else,                                name = '?';
    end
end
