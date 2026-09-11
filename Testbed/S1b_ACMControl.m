%S1B_ACMCONTROL Waveform generation, ACM control, and uplink command decode.
%
%   Owns everything S1a_Transmitter.m does not: the DVB-S2 waveform
%   generator, the transmit FIFO, the selective-repeat ARQ retransmit
%   queue, and ACM policy (Functions/dvbs2ACMPolicy.m). S1a is left with
%   only what genuinely cannot move: the two radios themselves, which
%   share one IP and so must stay in one process.
%
%   UPLINK: ONLY STAGES 6-8 LIVE HERE NOW. S1a runs CCSDS uplink
%   acquisition (Functions/Uplink/ccsdsUplinkAcquire.m, stages 1-5 -- DC
%   suppress through Costas tracking) on the continuous raw uplink stream,
%   close to the radio, and forwards only the CLTUs that actually clear
%   the ASM correlation. This process receives those (already
%   Costas-tracked, already timing/phase-aligned) codeword symbols over
%   config.uplinkAcqPort and runs just derandomize + LDPC decode
%   (Functions/Uplink/ccsdsUplinkDecodeCodeblock.m) on each -- a discrete,
%   occasional decode, not a continuous-stream DSP problem, which is
%   exactly why it was worth splitting off from acquisition in the first
%   place. See S1a_Transmitter.m's own docstring for the full reasoning.
%
%   WHY GENERATION MOVED HERE TOO, WHEN THE FIRST VERSION OF THIS SPLIT
%   DELIBERATELY LEFT IT IN S1a. That first version only moved uplink
%   recovery and ACM control, on the reasoning that generation carried a
%   real pacing risk (see below) not worth taking on for a modest CPU
%   saving. Measured on hardware, that version made things WORSE, not
%   better: MODCOD oscillated (19->22->19, 24->21->19->13 in a handful of
%   reports) and frame loss rose to 13.2%, because splitting the process
%   ALSO split off the accidental debounce that S1's own ~400ms
%   generate-then-transmit cadence used to provide -- ACM feedback that
%   used to get naturally batched down to "one decision per iteration"
%   was suddenly reaching dvbs2ACMPolicy.m individually, and a single bad
%   SNR reading (dvbs2SNREstimate.m misfires on ~4% of frames, a known,
%   already-documented failure mode) was enough to trigger an
%   unprotected multi-rung drop (agreeCountDown=1, minDwellSecDown=0, by
%   design, so a genuinely collapsing link can react in one decision).
%   Moving generation here and giving it its own explicit wall-clock
%   pacer (below) restores that same ~400ms cadence deliberately instead
%   of accidentally, which fixes both problems with one mechanism: ACM
%   evaluation is debounced again, AND the CPU-sharing benefit that was
%   the actual point of this split is now real instead of marginal.
%
%   THE PACING RISK, AND HOW IT'S HANDLED. radioTx() used to be a
%   BLOCKING call, and its blocking was what paced the whole loop to real
%   time -- generation never needed its own clock because it was
%   downstream of something that already had one. Move generation to a
%   process that never touches the radio, and that tick disappears.
%   TCP's own backpressure is not a substitute: a txBlockSamples block
%   (a few MB) is close to or larger than a typical OS socket buffer, so
%   relying on it produces a bursty rhythm -- dump a block, stall on
%   buffer space, dump again -- not the smooth cadence
%   config.tx.genBudgetFraction and the dummy-filler ratio were tuned
%   against. So this loop paces itself explicitly: after sending each
%   block, it waits out whatever is left of
%   config.tx.blockSamples/config.usrp.sampleRate before starting the
%   next one. That is the same tic/toc pacing pattern sim mode always
%   used against its own unthrottled TCP link -- applied here
%   unconditionally, in both modes, because in neither mode does this
%   process touch a radio directly any more.

clear; clc;

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Functions')));

config = dvbs2TestbedConfig();

%% DVB-S2 waveform generator configuration
cfgDVBS2 = dvbs2WaveformGenerator();
cfgDVBS2.StreamFormat = config.dvbs2.StreamFormat;
cfgDVBS2.FECFrame = config.dvbs2.FECFrame;
cfgDVBS2.MODCOD = config.dvbs2.MODCOD;
cfgDVBS2.DFL = getDFL(cfgDVBS2.MODCOD, cfgDVBS2.FECFrame);
cfgDVBS2.HasPilots = config.dvbs2.HasPilots;
cfgDVBS2.SamplesPerSymbol = config.dvbs2.SamplesPerSymbol;
cfgDVBS2.RolloffFactor = config.dvbs2.RolloffFactor;
cfgDVBS2.UPL = config.dvbs2.UPL;

% Sample rate this waveform is produced at -- also the pacing loop's own
% clock (see the docstring above). Consistent with how config.chanBW is
% derived: Fsym = chanBW/(1+RolloffFactor) = usrp.sampleRate/SamplesPerSymbol,
% so Fsym*SamplesPerSymbol is just usrp.sampleRate.
Fsamp = config.usrp.sampleRate;

currentMODCOD = cfgDVBS2.MODCOD;

% Until the first real feedback report proves the link works, transmit
% calibration bursts at the most robust MODCOD instead of real data --
% S1a has nothing of its own to decide this with any more, so this flag
% lives here now and governs what gets GENERATED, not what gets sent.
linkEstablished = ~config.useSDR;
calibMODCOD = 1;
if config.useSDR
    cfgDVBS2.MODCOD = calibMODCOD;
    cfgDVBS2.DFL = getDFL(calibMODCOD, cfgDVBS2.FECFrame);
    currentMODCOD = calibMODCOD;
end

%% Return-link input: raw uplink samples from S1a (RF mode), or the
% feedback/retransmit servers hosted directly (TCP/sim mode).
%
% OPENED BEFORE THE OUTBOUND CONNECT BELOW, DELIBERATELY -- same reason
% as S1a's identical reordering. This process's own server(s) must go up
% before it ever blocks trying to connect to S1a's, or the two processes
% can deadlock: each waiting on a server the other hasn't reached yet.
% Measured on hardware with the old ordering (connect first, serve
% second, symmetric in both files): neither side ever came up.
uplinkRF = config.useSDR && config.uplink.useRF;
if uplinkRF
    fprintf('S1b: opening uplink acquisition server on port %d, waiting for S1a to connect ...\n', ...
        config.uplinkAcqPort);
    uplinkAcqServer = dvbs2TCPServerRetry(config.uplinkAcqHost, config.uplinkAcqPort, "S1b's uplink acquisition server");
    while ~uplinkAcqServer.Connected
        pause(0.1);
    end
    fprintf('S1b: S1a connected.\n');
else
    fprintf('S1b: opening ACM feedback listener on port %d ...\n', config.feedbackPort);
    feedbackServer = dvbs2TCPServerRetry(config.feedbackHost, config.feedbackPort, "S1b's feedback server");
    fprintf('S1b: opening retransmit-request listener on port %d ...\n', config.retransmitPort);
    retransmitServer = dvbs2TCPServerRetry(config.retransmitHost, config.retransmitPort, "S1b's retransmit-request server");
end

%% TX block output TO S1a
% S1a hosts (see its own comment for why); this process is the client,
% retrying until S1a is listening -- and by now S1a's server is either
% already up or will be shortly, since S1a opens it before it ever tries
% to connect to the server above, for exactly the same reason.
fprintf('S1b: connecting to S1a''s TX block server at %s:%d ...\n', ...
    config.txBlockHost, config.txBlockPort);
txBlockClient = dvbs2TCPConnectRetry( ...
    config.txBlockHost, config.txBlockPort, "S1a's TX block server");
fprintf('S1b: connected to S1a.\n');

%% ACM state
acmState = [];
lastReturnTic = tic;
returnLinkLost = false;
uplinkFbQueue = {};
uplinkRtQueue = {};
% Note: RX overruns and acquisition-quality figures (carrier level, ASM
% metric) are S1a-side metrics now, tracked in its own profile -- this
% process never touches the radio and never runs stages 1-5 any more, only
% the decode (stages 6-7) of whatever CLTUs S1a already found.
uplinkDecodes = 0;
uplinkParityFails = 0;
uplinkEsNodB = NaN;
% How many CLTU messages were actually drained and handed to
% ccsdsUplinkDecodeCodeblock -- compare against S1a's "CLTUs found" count
% in its own profile. The two should match exactly; a gap means this loop
% is falling behind S1a's forwarding rate.
cltusReceived = 0;

%% Retransmit-request queue -- validated and applied HERE now, because
% globalPktIdx (below) lives here too: generation and its bookkeeping
% were never going to be split across the process boundary, only
% radio I/O was.
retransmitQueue = struct('StartIdx', {}, 'EndIdx', {});

%% Data source setup
syncByte = de2bi(hex2dec('47'), 8, 'left-msb')';   % MPEG-TS sync byte (0x47), prepended to every packet

% globalPktIdx is a running count of every packet ever transmitted this
% run, independent of burst/MODCOD boundaries. Each packet's payload
% embeds its own globalPktIdx (Functions/dvbs2ReferencePacketPayload.m)
% so the receiving side can independently regenerate the matching
% reference for bit-for-bit BER comparison from the index alone --
% deliberately NOT relying on replaying a single long RNG draw sequence
% in lockstep across processes, which would break under frame loss,
% reordering, or MinNumPackets changing with MODCOD between bursts.
% TRANSMIT FIFO -- how this script sends a variable-length waveform
% through S1a's fixed-length radio block. Unchanged reasoning from
% before the split: comm.SDRuTransmitter (now in S1a) locks its input
% size on the first call, so every block sent over txBlockPort has to be
% exactly config.tx.blockSamples regardless of which MODCOD produced the
% frames inside it. See localFramesPerBurst/localPLFrameSamples below
% for the arithmetic that keeps this bounded below one frame.
txBlockSamples = config.tx.blockSamples;

% Measured cost of generating one PLFRAME, tracked as a running average and
% used to decide how many real frames fit in this block's generation budget.
genBudgetSec = (txBlockSamples / Fsamp) * config.tx.genBudgetFraction;

% GENERATION STATE, BUNDLED. Everything localGenerateBlock (bottom of this
% file) reads and updates as it builds one block, packed into one struct
% so it can be threaded through that function as a single in/out argument
% instead of a dozen separate ones. globalPktIdx is a running count of
% every packet ever transmitted this run, independent of burst/MODCOD
% boundaries -- each packet's payload embeds its own globalPktIdx
% (Functions/dvbs2ReferencePacketPayload.m) so the receiving side can
% independently regenerate the matching reference for bit-for-bit BER
% comparison from the index alone, deliberately NOT relying on replaying a
% single long RNG draw sequence in lockstep across processes, which would
% break under frame loss, reordering, or MinNumPackets changing with
% MODCOD between bursts.
genState = struct( ...
    'txFifo', complex(zeros(0,1)), ...
    'globalPktIdx', 0, ...
    'frameSeqNum', 0, ...
    'burstNum', 0, ...
    'calibBurstNum', 0, ...
    'retransmitQueue', retransmitQueue, ...
    'modcodHistogram', zeros(1, 28), ...
    'maxFifoCarry', 0, ...
    'dummySamplesSent', 0, ...
    'genSecPerFrame', 0.025);
clear retransmitQueue   % lives in genState.retransmitQueue from here on

% PIPELINE STATE -- one block generated ahead of what's currently being
% sent, plus how many times a stale one had to be thrown away. See the
% main loop's own comment, at the point these are used, for the full
% reasoning.
pendingBlock = [];
pendingMeta = [];
blocksDropped = 0;
genDeadlineMisses = 0;   % ticks where generation+send took longer than one block's own airtime

% Pre-warm the dummy-filler cache -- see dvbs2DummyFiller.m's own comment
% for why this has to happen before the first block that needs it, not on it.
dvbs2DummyFiller(txBlockSamples, cfgDVBS2.SamplesPerSymbol, cfgDVBS2.RolloffFactor, 10);

newDataBudgetLogged = false;

runTic = tic;
airtimeSec = 0;
prof = struct('uplinkDSP', 0, 'waveformGen', 0, 'blockSend', 0, 'pacingWait', 0, 'iters', 0);

fprintf('\nS1b: starting generation + ACM control loop (initial MODCOD %d, %d-sample blocks) ...\n', ...
    currentMODCOD, txBlockSamples);

while true

    %% Stop after config.runDurationSec and report where the time went
    if toc(runTic) >= config.runDurationSec
        wall = toc(runTic);
        fprintf('\n=== S1b PROFILE === %.1f s wall | %.1f s airtime | RT factor %.3f | %d iterations\n', ...
            wall, airtimeSec, wall/max(airtimeSec,eps), prof.iters);
        accounted = prof.uplinkDSP + prof.waveformGen + prof.blockSend + prof.pacingWait;
        fprintf('  uplink decode        %7.2f s  %5.1f%%   %d CLTUs received, %d decodes, %d parity fails, Es/No %.1f dB\n', ...
            prof.uplinkDSP, 100*prof.uplinkDSP/wall, cltusReceived, uplinkDecodes, uplinkParityFails, uplinkEsNodB);
        fprintf('  waveform generation  %7.2f s  %5.1f%%\n', ...
            prof.waveformGen, 100*prof.waveformGen/wall);
        fprintf('  block send to S1a    %7.2f s  %5.1f%%\n', ...
            prof.blockSend, 100*prof.blockSend/wall);
        fprintf('  pacing wait          %7.2f s  %5.1f%%   (idle time deliberately spent keeping real-time cadence)\n', ...
            prof.pacingWait, 100*prof.pacingWait/wall);
        fprintf('  everything else      %7.2f s  %5.1f%%\n', ...
            wall - accounted, 100*(wall - accounted)/wall);

        seenMC = find(genState.modcodHistogram > 0);
        if ~isempty(seenMC)
            fprintf('  MODCODs generated:');
            for m = seenMC
                fprintf('  %d(%s)x%d', m, localModName(m), genState.modcodHistogram(m));
            end
            fprintf('\n');
        end
        fprintf('  tx FIFO carry-over: max %d samples (< one PLFRAME) | block %d, never resized\n', ...
            genState.maxFifoCarry, txBlockSamples);
        fprintf('  dummy filler: %.1f%% of generated airtime | generation %.1f ms/frame measured\n', ...
            100*genState.dummySamplesSent/max(genState.dummySamplesSent + prof.iters*txBlockSamples, 1), ...
            1e3*genState.genSecPerFrame);
        fprintf('  --\n');
        fprintf('  generation deadline missed %d of %d blocks (ran behind its own pacing tick)\n', ...
            genDeadlineMisses, prof.iters);
        fprintf('  pending blocks dropped as stale (MODCOD/link-state changed before sending) %d\n', ...
            blocksDropped);
        fprintf('  per iteration: %.1f ms wall, %.1f ms of airtime produced\n\n', ...
            1e3*wall/max(prof.iters,1), 1e3*airtimeSec/max(prof.iters,1));
        break;
    end
    prof.iters = prof.iters + 1;

    tickTic = tic;   % this iteration's pacing anchor -- see the docstring

    drawnow limitrate;   % service the tcpserver accept queue -- see S1a's identical comment

    %% Return-link liveness -- reacts locally now (reconfigure cfgDVBS2
    % directly) instead of sending S1a a message, since this process owns
    % the generator.
    if linkEstablished
        sinceReturn = toc(lastReturnTic);
        if sinceReturn > config.acm.linkLossSec && ~returnLinkLost
            returnLinkLost = true;
            fprintf(2, 'S1b: no return message for %.1f s -- return link presumed LOST; dropping to MODCOD %d.\n', ...
                sinceReturn, calibMODCOD);
            if currentMODCOD ~= calibMODCOD
                release(cfgDVBS2);
                cfgDVBS2.MODCOD = calibMODCOD;
                cfgDVBS2.DFL = getDFL(calibMODCOD, cfgDVBS2.FECFrame);
                currentMODCOD = calibMODCOD;
            end
        end
    end

    %% Collect fbBytes/rtBytes, whichever source is in use
    if uplinkRF
        % DRAIN EVERY CLTU CURRENTLY BUFFERED, not just one. S1a forwards
        % one message per DETECTED CLTU now (not one per radio read, the
        % old raw-relay design) -- but a burst of commands can still queue
        % up several in a row, and the same "read only one per tick"
        % mistake that used to lose ~90% of raw chunks would just as
        % easily lose queued CLTUs here. Draining in a tight loop, and
        % pacing only the GENERATION side below, is what keeps reading
        % fast while still debouncing ACM evaluation.
        cltusThisTick = 0;
        while true
            cltuBytes = dvbs2TCPFrameTryRead(uplinkAcqServer);
            if isempty(cltuBytes)
                break;
            end
            cltusThisTick = cltusThisTick + 1;
            cltu = dvbs2DeserializeCLTU(cltuBytes);

            % Stages 6 and 7 only -- derandomize, LDPC decode. Acquisition
            % (stages 1-5) already happened in S1a; see its own docstring
            % and ccsdsUplinkAcquire.m for why the split lands here.
            tDSP = tic;
            [payload, ok, ~] = ccsdsUplinkDecodeCodeblock( ...
                cltu.codeSyms, cltu.amplitude, cltu.noiseVar, config);
            prof.uplinkDSP = prof.uplinkDSP + toc(tDSP);
            if isfinite(cltu.esNodB), uplinkEsNodB = cltu.esNodB; end

            if ~ok
                uplinkParityFails = uplinkParityFails + 1;
                continue;
            end
            uplinkDecodes = uplinkDecodes + 1;

            % One uplink, two message types -- routed on the 3-bit type
            % field that heads every control message.
            switch dvbs2MessageType(payload)
                case 0, uplinkFbQueue{end+1} = payload; %#ok<SAGROW>
                case 1, uplinkRtQueue{end+1} = payload; %#ok<SAGROW>
            end
        end
        cltusReceived = cltusReceived + cltusThisTick;

        % FEEDBACK: NEWEST WINS, older ones are discarded -- a report is a
        % statistical summary of the link right now, so an older one is
        % misleading, not merely redundant. Retransmit requests are the
        % opposite: each names different packets, so all are queued.
        %
        % THIS ALONE IS NOT WHAT DEBOUNCES ACM EVALUATION ANY MORE -- see
        % the docstring's explanation of why that used to be true almost
        % by accident and isn't reliable on its own. The real debounce is
        % this loop's own pacing tick at the bottom: this queue just
        % keeps a backlog from building INSIDE one tick, exactly as it
        % always did.
        if ~isempty(uplinkFbQueue)
            fbBytes = uplinkFbQueue{end};
            uplinkFbQueue = {};
        else
            fbBytes = [];
        end
        if ~isempty(uplinkRtQueue)
            rtBytes = uplinkRtQueue{1};
            uplinkRtQueue(1) = [];
        else
            rtBytes = [];
        end
    else
        if feedbackServer.Connected
            fbBytes = dvbs2TCPFrameTryRead(feedbackServer);
        else
            fbBytes = [];
        end
        if retransmitServer.Connected
            rtBytes = dvbs2TCPFrameTryRead(retransmitServer);
        else
            rtBytes = [];
        end
    end

    %% ACM decision from feedback, if any arrived -- applied directly,
    % no network hop needed any more.
    if ~isempty(fbBytes)
        feedback = dvbs2DeserializeFeedback(fbBytes);
        lastReturnTic = tic;
        if returnLinkLost
            returnLinkLost = false;
            fprintf('S1b: return link RECOVERED; ACM resuming.\n');
        end

        if ~linkEstablished
            % First feedback ever received -- S2b has proven it's
            % actually receiving. Pick the starting MODCOD from those
            % samples' own mean and spread, via the same ladder the
            % steady-state policy uses, so the two cannot disagree.
            bootMu = feedback.MeanSNRdB;
            bootSigma = feedback.SigmaSNRdB;
            bootMODCOD = dvbs2SelectMODCOD(bootMu, bootSigma, 0, NaN, config);

            linkEstablished = true;
            release(cfgDVBS2);
            cfgDVBS2.MODCOD = bootMODCOD;
            cfgDVBS2.DFL = getDFL(bootMODCOD, cfgDVBS2.FECFrame);
            currentMODCOD = bootMODCOD;
            fprintf('S1b: link established (S2b SNR mean=%.2f dB sigma=%.2f dB over %d frames) -- starting real traffic at MODCOD %d\n', ...
                bootMu, bootSigma, feedback.Count, currentMODCOD);
        else
            [recommendedMODCOD, acmState] = dvbs2ACMPolicy( ...
                feedback, currentMODCOD, acmState, config);
            if feedback.Count == 0
                fprintf(2, 'S1b: receiver reports ZERO frames decoded -- forward link is not being received.\n');
            end
            fprintf(['S1b: report %d frames (mean %.2f sigma %.2f) | window mu=%.2f sigma=%.2f ' ...
                'trend=%+.2f dB/%gs | RSSI=%.2f -> policy: MODCOD %d\n'], ...
                feedback.Count, feedback.MeanSNRdB, feedback.SigmaSNRdB, ...
                acmState.Mu, acmState.Sigma, acmState.TrendAdjDB, ...
                config.acm.trendHorizonSec, feedback.RSSIdB, recommendedMODCOD);

            if recommendedMODCOD ~= currentMODCOD
                fprintf('S1b: *** switching MODCOD %d -> %d at burst boundary ***\n', ...
                    currentMODCOD, recommendedMODCOD);
                release(cfgDVBS2);
                cfgDVBS2.MODCOD = recommendedMODCOD;
                cfgDVBS2.DFL = getDFL(recommendedMODCOD, cfgDVBS2.FECFrame);
                currentMODCOD = recommendedMODCOD;
            end
        end
    end

    %% Retransmit request -> validate and queue. Same checks as before
    % the split, unchanged -- only WHERE they run moved, following
    % globalPktIdx.
    if ~isempty(rtBytes)
        request = dvbs2DeserializeRetransmitRequest(rtBytes);
        lastReturnTic = tic;
        if returnLinkLost
            returnLinkLost = false;
            fprintf('S1b: return link RECOVERED (via retransmit request).\n');
        end

        requestLen = request.EndIdx - request.StartIdx + 1;
        if request.EndIdx < request.StartIdx
            warning('S1b:MalformedRetransmitRequest', ...
                'Dropping malformed retransmit request [%d, %d] (EndIdx before StartIdx).', ...
                request.StartIdx, request.EndIdx);
        elseif requestLen > config.retransmit.maxRequestRange
            warning('S1b:OversizedRetransmitRequest', ...
                'Dropping retransmit request [%d, %d]: %d packets exceeds config.retransmit.maxRequestRange (%d).', ...
                request.StartIdx, request.EndIdx, requestLen, config.retransmit.maxRequestRange);
        elseif genState.globalPktIdx > 0 && request.StartIdx >= genState.globalPktIdx
            warning('S1b:UnsentRetransmitRequest', ...
                'Dropping retransmit request [%d, %d]: no packet at or beyond index %d has been transmitted yet.', ...
                request.StartIdx, request.EndIdx, genState.globalPktIdx);
        else
            fprintf('S1b: retransmit request received for packets [%d, %d]\n', ...
                request.StartIdx, request.EndIdx);
            genState.retransmitQueue(end+1) = struct('StartIdx', request.StartIdx, 'EndIdx', request.EndIdx);
        end
    end

    %% PIPELINE, STAGE 1: drop the pending block if it's gone stale. It
    % was built for the MODCOD/link-state that was active when it was
    % generated (either the previous iteration's prefetch, or -- rarely --
    % the recovery generation just below, right after a switch). If ACM or
    % the return-link-liveness check above has since moved on, sending it
    % anyway would push one more block at a MODCOD the policy has already
    % abandoned -- exactly what config.acm.agreeCountDown=1/minDwellSecDown=0
    % exists to prevent. Better to throw it away and fall one tick behind
    % than to violate that guarantee.
    if ~isempty(pendingBlock) && ...
            (pendingMeta.MODCOD ~= currentMODCOD || pendingMeta.IsCalibration ~= ~linkEstablished)
        blocksDropped = blocksDropped + 1;
        pendingBlock = [];
        pendingMeta = [];
    end

    %% PIPELINE, STAGE 2: recovery generation. Normally pendingBlock was
    % already filled by last iteration's prefetch (stage 4 below) and this
    % is a no-op; it only actually runs right after startup or right after
    % the drop above just emptied it.
    if isempty(pendingBlock)
        [pendingBlock, pendingMeta, genState, genElapsed] = localGenerateBlock( ...
            linkEstablished, currentMODCOD, calibMODCOD, cfgDVBS2, txBlockSamples, ...
            genBudgetSec, config, syncByte, genState);
        prof.waveformGen = prof.waveformGen + genElapsed;
    end

    %% PIPELINE, STAGE 3: send whatever is ready. localGenerateBlock
    % legitimately returns nothing when linked, the new-data budget is
    % spent, and no retransmit is queued -- that is not an error, it's the
    % case this whole redesign was built for: rather than block here
    % waiting for work that was never coming (the old behaviour), just
    % send nothing this tick. S1a falls back to a local dummy frame on its
    % own when a tick brings it no block (see its own comment) instead of
    % stalling the radio.
    if ~isempty(pendingBlock)
        txWaveform = pendingBlock;
        sentMeta = pendingMeta;
        pendingBlock = [];
        pendingMeta = [];

        tSend = tic;
        dvbs2TCPFrameWrite(txBlockClient, dvbs2ComplexToBytes(txWaveform));
        prof.blockSend = prof.blockSend + toc(tSend);
        airtimeSec = airtimeSec + numel(txWaveform)/Fsamp;

        if sentMeta.IsCalibration
            if mod(genState.calibBurstNum, 50) == 0
                if uplinkRF
                    feedbackStateStr = sprintf('RF uplink Es/No %.1f dB, %d decoded (carrier level: see S1a''s log)', ...
                        uplinkEsNodB, uplinkDecodes);
                elseif feedbackServer.Connected
                    feedbackStateStr = 'feedback link UP';
                else
                    feedbackStateStr = 'feedback link DOWN -- S2b has not connected';
                end
                fprintf('S1b: still calibrating (%d bursts sent; %s) ...\n', genState.calibBurstNum, feedbackStateStr);
            end
        elseif mod(genState.burstNum, 10) == 0
            if uplinkRF
                fprintf(['S1b: burst %d sent (MODCOD %d, %d frames so far) | RT factor %.3f | ' ...
                    'uplink %d decoded, Es/No %.1f dB\n'], ...
                    genState.burstNum, currentMODCOD, genState.frameSeqNum, toc(runTic)/max(airtimeSec,eps), ...
                    uplinkDecodes, uplinkEsNodB);
            else
                fprintf('S1b: burst %d sent (MODCOD %d, %d frames so far) | RT factor %.3f\n', ...
                    genState.burstNum, currentMODCOD, genState.frameSeqNum, toc(runTic)/max(airtimeSec,eps));
            end
        end
    elseif linkEstablished && ~newDataBudgetLogged
        fprintf('S1b: new-data budget (%d frames) reached; staying active to service any retransmit requests ...\n', ...
            config.maxFrames);
        newDataBudgetLogged = true;
    end

    %% PIPELINE, STAGE 4: prefetch the next block now, one tick ahead of
    % when it's needed. This is the actual point of the whole pipeline: it
    % decouples "did generation finish before THIS tick's pacing deadline"
    % from "does S1a get a block on time" by giving generation a full
    % extra tick of slack to absorb, at the cost of at most one stale
    % drop (stage 1) whenever ACM moves in that slack window.
    if isempty(pendingBlock)
        [pendingBlock, pendingMeta, genState, genElapsed] = localGenerateBlock( ...
            linkEstablished, currentMODCOD, calibMODCOD, cfgDVBS2, txBlockSamples, ...
            genBudgetSec, config, syncByte, genState);
        prof.waveformGen = prof.waveformGen + genElapsed;
    end

    %% PACE TO REAL TIME. This tick is what the whole loop now runs on,
    % replacing radioTx()'s old implicit one -- see the docstring. If
    % stages 2-4 together took LONGER than the block's own airtime, that
    % is surfaced separately (genDeadlineMisses) from a downstream TX
    % underrun in S1a's own profile: this counter says THIS process could
    % not keep up; S1a's says the radio actually ran dry. They usually
    % move together but are not the same measurement, and telling them
    % apart is the point of keeping both.
    elapsed = toc(tickTic);
    target = txBlockSamples / Fsamp;
    if elapsed < target
        pause(target - elapsed);
    else
        genDeadlineMisses = genDeadlineMisses + 1;
    end
    prof.pacingWait = prof.pacingWait + max(0, target - elapsed);
end

%% ------------------------------------------------------------------
%% Local functions
%% ------------------------------------------------------------------

function [waveform, meta, st, genElapsed] = localGenerateBlock( ...
    linkEstablished, currentMODCOD, calibMODCOD, cfg, blockSamples, ...
    genBudgetSec, config, syncByte, st)
%LOCALGENERATEBLOCK Build exactly one txBlockSamples-long block, or none.
%
%   Called from both pipeline stages (recovery and prefetch, see the main
%   loop) with identical arguments -- which stage called it doesn't
%   change what it does. st is the genState struct threaded through by
%   value and returned updated (cfg, the waveform generator, is a handle
%   object and mutates in place regardless).
%
%   Returns waveform = [] only on the real/retransmit path, and only when
%   the new-data budget is spent AND no retransmit is queued -- there is
%   genuinely nothing to build. The calibration path never does this: while
%   ~linkEstablished there is always a calibration burst to send.
    if ~linkEstablished
        st.calibBurstNum = st.calibBurstNum + 1;
        pktPayloadLen = cfg.UPL - 8;
        framesPerBurst = localFramesPerBurst(cfg, blockSamples, numel(st.txFifo));
        numPkts = cfg.MinNumPackets * framesPerBurst;
        calibIndices = 0 : numPkts - 1;
        data = dvbs2GeneratePacketBurst(calibIndices, pktPayloadLen, config.dataSeed, syncByte);

        tGen = tic;
        st.txFifo = [st.txFifo; cfg(data)];
        genElapsed = toc(tGen);

        waveform = st.txFifo(1:blockSamples);
        st.txFifo = st.txFifo(blockSamples+1:end);

        meta = struct('MODCOD', calibMODCOD, 'IsCalibration', true, ...
            'IsRetransmit', false, 'FramesPerBurst', framesPerBurst);
        return;
    end

    %% Real/retransmit path. A queued retransmit request has strict
    % priority; otherwise the next normal sequential burst, unless the
    % new-data budget is already spent -- in which case there is nothing
    % to generate this call, and the caller (main loop) sends nothing.
    isRetransmit = ~isempty(st.retransmitQueue);
    if ~isRetransmit && st.frameSeqNum >= config.maxFrames
        waveform = [];
        meta = [];
        genElapsed = 0;
        return;
    end

    st.burstNum = st.burstNum + 1;
    pktPayloadLen = cfg.UPL - 8;
    framesPerBurst = localFramesPerBurst(cfg, blockSamples, numel(st.txFifo));

    % CAP BY WHAT GENERATION CAN AFFORD -- unchanged reasoning from before
    % the split, just now bounded by this process's OWN generation budget
    % rather than S1's combined generation+transmit budget.
    maxRealFrames = max(1, floor(genBudgetSec / max(st.genSecPerFrame, eps)));
    framesPerBurst = min(framesPerBurst, maxRealFrames);

    numPkts = cfg.MinNumPackets * framesPerBurst;

    if isRetransmit
        request = st.retransmitQueue(1);
        st.retransmitQueue(1) = [];
        requestedRange = request.StartIdx : request.EndIdx;

        if numel(requestedRange) > numPkts
            pktIndices = requestedRange(1:numPkts);
            st.retransmitQueue(end+1) = struct('StartIdx', requestedRange(numPkts+1), 'EndIdx', request.EndIdx);
        elseif numel(requestedRange) < numPkts
            padCount = numPkts - numel(requestedRange);
            pktIndices = [requestedRange, requestedRange(end) + (1:padCount)];
        else
            pktIndices = requestedRange;
        end

        fprintf('S1b: sending RETRANSMIT burst for packets [%d..%d] (requested [%d..%d])\n', ...
            pktIndices(1), pktIndices(end), request.StartIdx, request.EndIdx);
    else
        pktIndices = st.globalPktIdx : st.globalPktIdx + numPkts - 1;
        st.globalPktIdx = st.globalPktIdx + numPkts;
    end

    data = dvbs2GeneratePacketBurst(pktIndices, pktPayloadLen, config.dataSeed, syncByte);

    tGen = tic;
    st.txFifo = [st.txFifo; cfg(data)];
    genElapsed = toc(tGen);

    st.genSecPerFrame = 0.8*st.genSecPerFrame + 0.2*(genElapsed / framesPerBurst);

    if numel(st.txFifo) < blockSamples
        shortfall = blockSamples - numel(st.txFifo);
        st.txFifo = [st.txFifo; dvbs2DummyFiller(shortfall, ...
            cfg.SamplesPerSymbol, cfg.RolloffFactor, 10)];
        st.dummySamplesSent = st.dummySamplesSent + shortfall;
    end

    waveform = st.txFifo(1:blockSamples);
    st.txFifo = st.txFifo(blockSamples+1:end);

    st.modcodHistogram(currentMODCOD) = st.modcodHistogram(currentMODCOD) + framesPerBurst;
    st.maxFifoCarry = max(st.maxFifoCarry, numel(st.txFifo));

    if ~isRetransmit
        st.frameSeqNum = st.frameSeqNum + framesPerBurst;
    end

    meta = struct('MODCOD', currentMODCOD, 'IsCalibration', false, ...
        'IsRetransmit', isRetransmit, 'FramesPerBurst', framesPerBurst);
end

function n = localFramesPerBurst(cfg, blockSamples, fifoLen)
%LOCALFRAMESPERBURST How many PLFRAMEs to build to cover one radio block.
%
%   Sized from what is still MISSING (blockSamples - fifoLen) rather than
%   from the block alone, which is what keeps the transmit FIFO bounded.
%   Never returns 0: a burst is always generated. Skipping one would
%   consume packet indices or pop a retransmit request without sending
%   either.
    n = max(1, ceil((blockSamples - fifoLen) / localPLFrameSamples(cfg)));
end

function n = localPLFrameSamples(cfg)
%LOCALPLFRAMESAMPLES Length of one PLFRAME, in samples, at the current MODCOD.
%
%   A FECFRAME is a fixed number of BITS, so its symbol count falls as the
%   modulation gets denser. The modulation is read from the object rather
%   than mapped from the MODCOD index, so this cannot drift out of step
%   with what the generator is actually producing.
    switch string(info(cfg).ModulationScheme)
        case "QPSK",   bitsPerSym = 2;
        case "8PSK",   bitsPerSym = 3;
        case "16APSK", bitsPerSym = 4;
        case "32APSK", bitsPerSym = 5;
        otherwise
            error('S1b:UnknownModulation', ...
                'No PLFRAME size known for modulation "%s".', ...
                info(cfg).ModulationScheme);
    end

    if string(cfg.FECFrame) == "short"
        fecFrameBits = 16200;
    else
        fecFrameBits = 64800;
    end

    numSlots = fecFrameBits / (bitsPerSym * 90);
    numSymbols = 90 + numSlots*90;
    if cfg.HasPilots
        numSymbols = numSymbols + floor((numSlots - 1)/16) * 36;
    end

    n = numSymbols * cfg.SamplesPerSymbol;
end

function name = localModName(modcod)
%LOCALMODNAME Modulation for a legacy DVB-S2 MODCOD index, for the summary.
%   Ranges per ETSI EN 302 307-1 table 12.
    if     modcod >= 1  && modcod <= 11, name = 'QPSK';
    elseif modcod >= 12 && modcod <= 17, name = '8PSK';
    elseif modcod >= 18 && modcod <= 23, name = '16APSK';
    elseif modcod >= 24 && modcod <= 28, name = '32APSK';
    else,                                name = '?';
    end
end
