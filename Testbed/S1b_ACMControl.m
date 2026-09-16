%S1B_ACMCONTROL Waveform generation, ACM control, and uplink command decode.
%
%   Owns the DVB-S2 waveform generator, the transmit FIFO, the
%   selective-repeat ARQ retransmit queue, and the ACM policy
%   (Functions/dvbs2ACMPolicy.m). S1a keeps only what can't move: the two
%   radios, which share one IP and must stay in one process.
%
%   UPLINK: only stages 6-8 (derandomize + LDPC decode,
%   Functions/Uplink/ccsdsUplinkDecodeCodeblock.m) run here. S1a runs
%   acquisition (stages 1-5, Functions/Uplink/ccsdsUplinkAcquire.m) close
%   to the radio and forwards only CLTUs that clear ASM correlation --
%   already Costas-tracked and timing/phase-aligned -- so this process
%   handles a discrete, occasional decode, not continuous-stream DSP.
%
%   PACING. This process never calls radioTx(), so unlike the old
%   single-process design it has no built-in real-time clock, and TCP
%   backpressure isn't a substitute (a txBlockSamples block is close to
%   or larger than a typical socket buffer, giving a bursty rhythm, not a
%   smooth one). So the loop paces itself explicitly: after sending each
%   block, it waits out whatever remains of
%   config.tx.blockSamples/config.usrp.sampleRate before generating the
%   next one.
%
%   WHY GENERATION LIVES HERE, NOT IN S1a. An earlier version left
%   generation in S1a and moved only ACM/uplink recovery here, to avoid
%   the pacing risk above for a modest CPU saving. On hardware this
%   performed worse: MODCOD oscillated and frame loss rose to 13.2%,
%   because splitting the process also removed the incidental debouncing
%   the old ~400 ms generate-then-transmit cadence provided -- individual
%   SNR readings (dvbs2SNREstimate.m misfires on ~4% of frames) started
%   reaching dvbs2ACMPolicy.m unbatched, each able to trigger an
%   unprotected multi-rung drop (agreeCountDown=1, minDwellSecDown=0, by
%   design, so a genuinely collapsing link can still react in one
%   decision). Moving generation here, with the explicit pacer above,
%   restores that debouncing deliberately while keeping the CPU-sharing
%   benefit the split was for.

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

% Sample rate this waveform is generated at; also the pacing loop's own
% clock (see docstring above).
Fsamp = config.usrp.sampleRate;

currentMODCOD = cfgDVBS2.MODCOD;

% Until the first feedback report proves the link works, transmit
% calibration bursts at the most robust MODCOD instead of real data.
% Governs what gets GENERATED, not what gets sent.
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
% Server opened BEFORE the outbound connect below, deliberately: if both
% processes connected first and served second, they can deadlock, each
% waiting on a server the other hasn't opened yet (confirmed on
% hardware). S1a follows the same ordering for the same reason.
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
% S1a hosts this link; this process is the client, retrying until S1a is
% listening (S1a opens its server before connecting to the one above, so
% it's ready by the time this runs).
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
% RX overruns and acquisition-quality metrics (carrier level, ASM) live
% in S1a's own profile now -- this process only decodes (stages 6-7)
% CLTUs S1a already found.
uplinkDecodes = 0;
uplinkParityFails = 0;
uplinkEsNodB = NaN;
% CLTU messages actually drained and decoded; compare against S1a's
% "CLTUs found" count -- a gap means this loop is falling behind S1a.
cltusReceived = 0;

%% Retransmit-request queue -- validated and applied here, since
% globalPktIdx (below) lives here too.
retransmitQueue = struct('StartIdx', {}, 'EndIdx', {});

%% Data source setup
syncByte = de2bi(hex2dec('47'), 8, 'left-msb')';   % MPEG-TS sync byte (0x47), prepended to every packet

% globalPktIdx counts every packet transmitted this run. Each packet
% embeds its own index (Functions/dvbs2ReferencePacketPayload.m) so the
% receiver can regenerate the matching reference for BER comparison from
% the index alone, instead of replaying an RNG sequence in lockstep --
% which would break under frame loss, reordering, or MinNumPackets
% changing with MODCOD.

% TRANSMIT FIFO. comm.SDRuTransmitter (in S1a) locks its input size on
% the first call, so every block sent over txBlockPort must be exactly
% config.tx.blockSamples regardless of which MODCOD produced the frames
% inside it. See localFramesPerBurst/localPLFrameSamples below.
txBlockSamples = config.tx.blockSamples;

% Measured per-frame generation cost, tracked as a running average, used
% to decide how many real frames fit in this block's generation budget.
genBudgetSec = (txBlockSamples / Fsamp) * config.tx.genBudgetFraction;

% Generation state, bundled into one struct so localGenerateBlock (below)
% can take/return it as a single argument instead of a dozen.
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

% PIPELINE STATE: one block generated ahead of what's being sent, plus a
% count of stale ones thrown away. See the main loop for the reasoning.
pendingBlock = [];
pendingMeta = [];
blocksDropped = 0;
genDeadlineMisses = 0;   % ticks where generation+send took longer than one block's own airtime

% Pre-warm the dummy-filler cache before the first block that needs it
% (see dvbs2DummyFiller.m).
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

    tickTic = tic;   % this iteration's pacing anchor

    drawnow limitrate;   % service the tcpserver accept queue (see S1a)

    %% Return-link liveness -- reacts locally (reconfigure cfgDVBS2
    % directly) instead of messaging S1a, since this process owns the
    % generator.
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
        % Drain every CLTU currently buffered, not just one -- S1a
        % forwards one message per detected CLTU, and several can queue
        % up in a burst. Draining in a tight loop (pacing happens only on
        % the generation side below) keeps reads fast without losing
        % queued CLTUs.
        cltusThisTick = 0;
        while true
            cltuBytes = dvbs2TCPFrameTryRead(uplinkAcqServer);
            if isempty(cltuBytes)
                break;
            end
            cltusThisTick = cltusThisTick + 1;
            cltu = dvbs2DeserializeCLTU(cltuBytes);

            % Stages 6-7 only (derandomize, LDPC decode); acquisition
            % (stages 1-5) already ran in S1a.
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

            % One uplink, two message types, routed on the 3-bit type
            % field heading every control message.
            switch dvbs2MessageType(payload)
                case 0, uplinkFbQueue{end+1} = payload; %#ok<SAGROW>
                case 1, uplinkRtQueue{end+1} = payload; %#ok<SAGROW>
            end
        end
        cltusReceived = cltusReceived + cltusThisTick;

        % Feedback: newest wins (a report is a snapshot of the link right
        % now, an older one is misleading). Retransmit requests: all
        % queued, since each names different packets. Note: this queue
        % only prevents a backlog from building WITHIN one tick -- actual
        % ACM debouncing comes from the pacing tick at the bottom of the
        % loop.
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

    %% ACM decision from feedback, if any arrived -- applied directly.
    if ~isempty(fbBytes)
        feedback = dvbs2DeserializeFeedback(fbBytes);
        lastReturnTic = tic;
        if returnLinkLost
            returnLinkLost = false;
            fprintf('S1b: return link RECOVERED; ACM resuming.\n');
        end

        if ~linkEstablished
            % First feedback ever received -- pick the starting MODCOD
            % from its mean/spread via the same ladder the steady-state
            % policy uses, so the two never disagree.
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

    %% Retransmit request -> validate and queue.
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

    %% PIPELINE STAGE 1: drop the pending block if stale. It was built
    % for whichever MODCOD/link-state was active at generation time; if
    % ACM or the liveness check above has since moved on, sending it
    % would push a block at a MODCOD the policy already abandoned --
    % exactly what agreeCountDown=1/minDwellSecDown=0 exists to prevent.
    % Discard and fall one tick behind rather than violate that.
    if ~isempty(pendingBlock) && ...
            (pendingMeta.MODCOD ~= currentMODCOD || pendingMeta.IsCalibration ~= ~linkEstablished)
        blocksDropped = blocksDropped + 1;
        pendingBlock = [];
        pendingMeta = [];
    end

    %% PIPELINE STAGE 2: recovery generation. Normally pendingBlock was
    % already filled by last iteration's prefetch (stage 4) and this is a
    % no-op; it only runs after startup or right after stage 1's drop.
    if isempty(pendingBlock)
        [pendingBlock, pendingMeta, genState, genElapsed] = localGenerateBlock( ...
            linkEstablished, currentMODCOD, calibMODCOD, cfgDVBS2, txBlockSamples, ...
            genBudgetSec, config, syncByte, genState);
        prof.waveformGen = prof.waveformGen + genElapsed;
    end

    %% PIPELINE STAGE 3: send whatever is ready. An empty pendingBlock
    % (linked, new-data budget spent, no retransmit queued) is not an
    % error -- just send nothing this tick. S1a falls back to a local
    % dummy frame on its own end rather than stalling the radio.
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

    %% PIPELINE STAGE 4: prefetch the next block one tick ahead of when
    % it's needed. This is the point of the pipeline: it decouples
    % "generation finished before this tick's deadline" from "S1a gets a
    % block on time", giving generation a full extra tick of slack, at
    % the cost of at most one stale drop (stage 1) if ACM moves during
    % that slack window.
    if isempty(pendingBlock)
        [pendingBlock, pendingMeta, genState, genElapsed] = localGenerateBlock( ...
            linkEstablished, currentMODCOD, calibMODCOD, cfgDVBS2, txBlockSamples, ...
            genBudgetSec, config, syncByte, genState);
        prof.waveformGen = prof.waveformGen + genElapsed;
    end

    %% PACE TO REAL TIME -- what the loop runs on now instead of
    % radioTx()'s old implicit pacing. genDeadlineMisses (stages 2-4
    % together took longer than the block's own airtime) is tracked
    % separately from S1a's own TX-underrun count: one says this process
    % couldn't keep up, the other says the radio actually ran dry -- they
    % usually move together but are not the same measurement.
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
%   Called identically from both pipeline stages (recovery and
%   prefetch). Returns waveform = [] only on the real/retransmit path,
%   when the new-data budget is spent and nothing is queued to retransmit
%   -- the calibration path always has a burst to send.
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

    %% Real/retransmit path: a queued retransmit request has strict
    % priority; otherwise the next sequential burst, unless the new-data
    % budget is already spent.
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

    % Cap by what generation can afford within this process's own budget.
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
%   Sized from what's still missing (blockSamples - fifoLen) rather than
%   the block alone, keeping the transmit FIFO bounded. Never returns 0
%   -- skipping a burst would consume packet indices or pop a retransmit
%   request without sending either.
    n = max(1, ceil((blockSamples - fifoLen) / localPLFrameSamples(cfg)));
end

function n = localPLFrameSamples(cfg)
%LOCALPLFRAMESAMPLES Length of one PLFRAME, in samples, at the current MODCOD.
%
%   A FECFRAME is a fixed number of bits, so symbol count falls as
%   modulation gets denser. Modulation is read from the object itself
%   (not mapped from the MODCOD index) so this can't drift out of sync
%   with what the generator actually produces.
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