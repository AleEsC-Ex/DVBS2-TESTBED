% S2B_RECIEVER DVB-S2 receiver: acquisition through symbol/phase correction, SNR/RSSI, hand-off.

clear; clc;

% Functions/ is organized into subfolders by purpose (core DSP/PHY.
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Functions')));

config = dvbs2TestbedConfig();

% Reset dvbs2MatchedFilterTimingSync's and dvbs2RawCFOCompensate's.
clear dvbs2MatchedFilterTimingSync;

plHeaderSymbols = 90;
sps = config.dvbs2.SamplesPerSymbol;
Fsym = config.chanBW / (1 + config.dvbs2.RolloffFactor);
Fsamp = Fsym * sps;

sofRef = dvbs2SOFReference();
searchLen = 1000;
weakLockStreak = 0;

% dvbs2MatchedFilterTimingSync's Gardner loop has no stable equilibrium.
chunksSinceLock = 0;
resyncResetChunks = 50;

% Consecutive chunks the timing loop has come back unusable. Used only to.
syncLostLockStreak = 0;

% PLHEADERs thrown out by dvbs2FrameAcceptable because S1a could not have.
rejectCount = 0;

% Frames where the pilot and PLHEADER SNR estimates disagreed by more than.
snrDisagreeCount = 0;

% Run totals for the final summary. A 2-minute run scrolls hundreds of.
plsHistogram   = zeros(1, 128);   % indexed PLS+1, so PLS 0 lands at 1
snrSrcPilots   = 0;
snrSrcHeader   = 0;
snrDiffLog     = [];
snrHdrPhaseLog = [];
residHzLog     = [];
residPhaseLog  = [];
hdrResidLog    = [];
syncRunawayCount = 0;

% Dummy PLFRAMEs seen. These are deliberate filler from S1a, not errors --.
dummyCount = 0;

%% RX acquisition setup
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
% Previous frame's length, used as the CFO tracker's dt. Seeded with a.
prevFrameLenSym = 33282;

fprintf('\nS2b: starting receive loop ...\n');

% Real-time accounting: airtimeSec is how many seconds of RF this process.
runTic = tic;
airtimeSec = 0;

% Set when the inner chunk-wait loop below detects S2a's connection has.
forceStop = false;

% Deliberately not gated on config.maxFrames: the transmitter only.
while true

    %% Stop after config.runDurationSec so all four scripts end together
    if forceStop || toc(runTic) >= config.runDurationSec
        fprintf('\n=== S2b PROFILE === %.1f s wall | %.1f s airtime | RT factor %.3f\n', ...
            toc(runTic), airtimeSec, toc(runTic)/max(airtimeSec,eps));
        fprintf('  chunks %d | frames decoded %d (%.1f%% of chunks)\n', ...
            chunkNum, frameSeqNum, 100*frameSeqNum/max(chunkNum,1));

% Everything below exists so this summary alone is diagnostic. The.

% WHICH MODCODs WERE ACTUALLY RECEIVED. The PLS code packs as.
        seenPLS = find(plsHistogram > 0);
        if ~isempty(seenPLS)
            fprintf('  MODCODs received:');
            for p = seenPLS
                fprintf('  %d(%s)x%d', floor((p-1)/4), ...
                    localModName(floor((p-1)/4)), plsHistogram(p));
            end
            fprintf('\n');
        end

% SNR CROSS-CHECK. Two independent references measured per frame;.
        if frameSeqNum > 0
            fprintf(['  SNR cross-check: %d disagreements >%.0f dB (%.1f%% of frames)' ...
                ' | reported from pilots %d, header %d | median |diff| %.2f dB\n'], ...
                snrDisagreeCount, config.snr.maxDisagreementDB, ...
                100*snrDisagreeCount/frameSeqNum, snrSrcPilots, snrSrcHeader, ...
                median([snrDiffLog NaN], 'omitnan'));
            fprintf('  header phase offset: median %+.1f deg | residual freq: median %+.1f Hz\n', ...
                median([snrHdrPhaseLog NaN], 'omitnan'), ...
                median([residHzLog NaN], 'omitnan'));

% THE PHASE-MARGIN LINE. AnchorResidual is what the straight-line.
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
% RSSI needs hardware-specific calibration to convert to dBm on real.
    [rssiDB, chunkFreqOffsetEst, newSamples_cfocomp] = dvbs2DeserializeAcqChunk(acqBytes);

    chunkNum = chunkNum + 1;
    airtimeSec = airtimeSec + numel(newSamples_cfocomp)/Fsamp;
    fprintf('S2b: chunk %d | RSSI=%.2f dB | rawCFO(S2a)=%.1f Hz | RT factor %.3f\n', ...
        chunkNum, rssiDB, chunkFreqOffsetEst, toc(runTic)/max(airtimeSec,eps));

    % Match filter & timing sync
    [newSamples_sync, ~, syncLostLock] = dvbs2MatchedFilterTimingSync( ...
        newSamples_cfocomp, sps, config.dvbs2.RolloffFactor, 7);

% A runaway timing loop has already been reset inside the function and.
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

% Inner loop: locate and process every complete PLFRAME currently.
    while length(rxBuffer) > searchLen + length(sofRef) - 1

        [idx, peak, ~] = dvbs2FrameSync(rxBuffer, sofRef, searchLen);
        if peak < config.frameSyncPeakThreshold
            weakLockStreak = weakLockStreak + 1;
            if weakLockStreak < 3
% Possibly a minor alignment artifact rather than.
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

% Closed-loop CFO tracking. The tracker is fed the TOTAL offset --.
        totalMeasHz = chunkFreqOffsetEst + coarseFreqEstHz;
        dtSec = prevFrameLenSym / Fsym;
        [totalTrackedHz, cfoState] = dvbs2CFOTracker(totalMeasHz, dtSec, cfoState, config);
        appliedFreqHz = totalTrackedHz - chunkFreqOffsetEst;

        rawHeader = rxBuffer(idx:idx+plHeaderSymbols-1);
        n_header = (0:plHeaderSymbols-1).';
        header_coarse = rawHeader .* exp(-1j*2*pi*(appliedFreqHz/Fsym)*n_header);

% Remove the residual CONSTANT phase offset before decoding the.
        phOffset = angle(sum(header_coarse(1:length(sofRef)) .* conj(sofRef)));
        header_coarse = header_coarse .* exp(-1j*phOffset);

        try
% dvbs2PLHeaderRecover.m skips the ambiguous DVB-S2/DVB-S2X.
            phyParams = dvbs2PLHeaderRecover(header_coarse);
        catch ME
% This script runs indefinitely -- an uncaught exception.
            warning('S2b:HeaderDecodeFailed', 'PLHEADER decode threw: %s', ME.message);
            rxBuffer(1:idx) = [];
            calibLockCount = 0;   % a bad decode breaks the consecutive-good-frame streak
            continue;
        end

        if phyParams.IsDummyFrame
% A dummy PLFRAME's PLSC carries no real MODCOD/FECFrame.
            dummyCount = dummyCount + 1;
            dummySymbols = 3330;
            rxBuffer(1 : min(idx + dummySymbols - 1, numel(rxBuffer))) = [];
            frameFoundThisChunk = true;
            continue;
        end

% PLAUSIBILITY REJECTION. Everything above this point only asks.
        [frameOK, rejectReason] = dvbs2FrameAcceptable(phyParams, config);
        if ~frameOK
            rejectCount = rejectCount + 1;
% Log the first few and then every 50th. A steady stream of.
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
% A misdecoded PLSC can also land on a reserved/undefined.
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
% The SAME phOffset measured from the SOF above is applied here.
        frame_coarse = rawFrame .* exp(-1j*(2*pi*(appliedFreqHz/Fsym)*n_frame + phOffset));
        plSymbols_coarse = [header_coarse; frame_coarse];

% Built BEFORE the phase compensation, not after, because the.
        refHeader = dvbs2PLHeaderReference(phyParams);

        phaseInfo = struct;
        if phyParams.HasPilots
            [normCFO, ~] = dvbs2FineFreqEst(plSymbols_coarse, fp);
            n_fine = (0:length(plSymbols_coarse)-1).';
            plSymbols_fine = plSymbols_coarse .* exp(-1j*2*pi*normCFO*n_fine);
% phaseInfo was previously discarded. It carries FitSlope --.
            [plSymbols_corrected, phaseInfo] = dvbs2PhaseCompensate(plSymbols_fine, fp, refHeader);
        else
            plSymbols_corrected = dvbs2NonPilotFineFreqPhase(plSymbols_coarse, sofRef, phyParams);
        end

        %% SNR / noise-variance estimate for this frame
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
% Residual phase the line fit could NOT explain, at the anchors. The.
        if isfield(phaseInfo, 'AnchorResidualRMSdeg')
            residPhaseLog(end+1) = phaseInfo.AnchorResidualRMSdeg; %#ok<SAGROW>
            hdrResidLog(end+1) = phaseInfo.HeaderResidualDeg;      %#ok<SAGROW>
        end

        if ~snrInfo.Trusted
            snrDisagreeCount = snrDisagreeCount + 1;
% Residual frequency error implied by the phase trajectory the.
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
        snrBatch(end+1) = snrDB; %#ok<SAGROW>

        if ~linkEstablished
% Require several CONSECUTIVE successfully-decoded.
            calibLockCount = calibLockCount + 1;
            if calibLockCount < config.calibLockFramesRequired
                fprintf('S2b: calibration frame locked (%d/%d consecutive), SNR=%.2f dB -- waiting for a stable run before reporting link established\n', ...
                    calibLockCount, config.calibLockFramesRequired, snrDB);
            else
% This can only be a real, successfully decoded run of.
                calibSamples = snrBatch(end - calibLockCount + 1 : end);

                [cm, cs] = localBatchStats(calibSamples, config);
                feedback.Count = numel(calibSamples);
                feedback.MeanSNRdB = cm;
                feedback.SigmaSNRdB = cs;
                feedback.RSSIdB = rssiDB;
                dvbs2TCPFrameWrite(feedbackClient, dvbs2SerializeFeedback(feedback));
                snrBatch = [];
                lastSentMean = cm; lastSentSigma = cs; lastReportTic = tic;

% The socket to S3 is already open (see initialisation.
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

% PLHeaderConfidence: gap between the best-matching PLSC.
            plHeaderConfidence = phyParams.DecodeMeanDistance - phyParams.DecodeMinDistance;
% CFO diagnostics: meas is this frame's raw SOF measurement of.
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
    if linkEstablished
        [batchMean, batchSigma] = localBatchStats(snrBatch, config);
        sinceLast = toc(lastReportTic);

% A CHANGE TRIGGER NEEDS MORE THAN ONE FRAME BEHIND IT.
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
% LOCALBATCHSTATS Mean and standard deviation of one batch, outliers removed.
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
% POPULATION standard deviation (normalise by N, not N-1). The.
    if numel(x) >= 2, s = std(x, 1); else, s = 0; end
end

function name = localModName(modcod)
% LOCALMODNAME Modulation for a legacy DVB-S2 MODCOD index, for the summary.
    if     modcod >= 1  && modcod <= 11, name = 'QPSK';
    elseif modcod >= 12 && modcod <= 17, name = '8PSK';
    elseif modcod >= 18 && modcod <= 23, name = '16APSK';
    elseif modcod >= 24 && modcod <= 28, name = '32APSK';
    else,                                name = '?';
    end
end
