%S1a_TRANSMITTER Both radios, plus CCSDS uplink acquisition (stages 1-5).
%
%   Owns both radios (same shared IP, which is why they cannot be split
%   across two processes) and generates nothing of the downlink waveform
%   itself -- every transmitted sample comes from S1b_ACMControl.m over
%   config.txBlockPort. This script makes no MODCOD decisions, runs no ACM
%   policy, and validates no retransmit requests.
%
%   IT DOES, HOWEVER, RUN THE FRONT HALF OF THE UPLINK RECEIVER. Raw
%   samples off the uplink radio go straight into ccsdsUplinkAcquire.m
%   (Functions/Uplink/ccsdsUplinkAcquire.m) -- stages 1-5: DC suppression,
%   coarse carrier acquisition, matched filtering, optional Gardner timing,
%   ASM/CLTU start-sequence detection, and Costas-loop phase tracking. Only
%   CLTUs that actually clear the ASM correlation get forwarded to
%   S1b_ACMControl.m over config.uplinkAcqPort, as their Costas-tracked
%   codeword symbols plus the two scalars (amplitude, noise variance)
%   ccsdsUplinkDecodeCodeblock.m needs to finish the job -- not the
%   continuous, mostly-idle raw sample stream PLOP-2's own continuous
%   carrier produces. S1b is left with stages 6-8 only: derandomize, LDPC
%   decode, command recovery.
%
%   WHY THIS BOUNDARY, AND NOT ANOTHER ONE. Stages 1-5 are what actually
%   pace with the CONTINUOUS uplink carrier -- CCSDS PLOP-2 is deliberately
%   never keyed off, specifically so a receiver's carrier/timing loops never
%   lose lock waiting for a message, which means something has to keep
%   processing that stream end to end regardless of whether it currently
%   carries a real command or idle sequence. Stages 6-8 are the opposite:
%   they only ever run once per DETECTED burst, a discrete, occasional
%   event, with no continuity requirement of their own. Splitting exactly
%   here turns a continuous-DSP problem plus an occasional-decode problem
%   into two processes that each only have to solve one of them.
%
%   Run this alongside the other four scripts, each as its own MATLAB
%   instance, IN ANY ORDER -- every TCP client connection in this testbed
%   (dvbs2TCPConnectRetry.m) retries until its server is up, and every
%   server (tcpserver) starts listening immediately without needing a
%   client yet, so there's no required startup sequence.

clear; clc;

% Functions/ is organized into subfolders by purpose (core DSP/PHY
% functions stay directly under Functions/; TCP/, Serialization/, and
% Testbed/ hold this testbed's supporting code) -- genpath adds all of
% them recursively, computed from this script's own location so it
% works regardless of MATLAB's current folder when this is run.
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Functions')));

config = dvbs2TestbedConfig();

Fsamp = config.usrp.sampleRate;
txBlockSamples = config.tx.blockSamples;

% Pre-warm the dummy-filler cache -- see dvbs2DummyFiller.m's own comment
% for why this has to happen before the first block that needs it, not on
% it. S1a keeps its own warm cache now too: it reaches for one independently
% whenever S1b doesn't have a block ready in time (see the TX-block wait
% loop below), rather than stalling the radio waiting for one that may not
% be coming this tick at all.
dvbs2DummyFiller(txBlockSamples, config.dvbs2.SamplesPerSymbol, config.dvbs2.RolloffFactor, 10);

% If S1b doesn't produce a block within this long, stop waiting and
% synthesize one locally instead. Set comfortably below one full block
% period (txBlockSamples/Fsamp) so there's still time left in this tick to
% transmit it and roughly hold cadence, rather than free-running behind
% S1b. S1b legitimately has nothing to send whenever its own new-data
% budget is exhausted for this tick or ACM just dropped a stale pending
% block (see its own pipeline comments) -- both are expected, not errors.
localDummyWaitThresholdSec = 0.85 * (txBlockSamples / Fsamp);

%% TX dispatch setup
if config.useSDR
    % comm.SDRuTransmitter requires an INTEGER InterpolationFactor -- the
    % USRP's digital upsampling stage only works in discrete integer
    % steps, and chanBW/RolloffFactor/SamplesPerSymbol rarely divide
    % MasterClockRate exactly. Round to the nearest achievable factor
    % and recompute Fsamp to the rate the hardware will actually run
    % at, so it accurately reflects reality rather than the
    % originally-intended, slightly different rate.
    radioTx = comm.SDRuTransmitter( ...
        'Platform', config.usrp.platform, ...
        'IPAddress', config.usrp.txIPAddress, ...
        'MasterClockRate', config.usrp.masterClockRate, ...
        'InterpolationFactor', config.usrp.inter_decimateFactor, ...
        'CenterFrequency', config.usrp.centerFrequency, ...
        'Gain', config.usrp.txGain);
    fprintf('S1a: transmitting via USRP at %s (CenterFrequency=%.3f MHz)\n', ...
        config.usrp.txIPAddress, config.usrp.centerFrequency/1e6);
    cleanupTx = onCleanup(@() release(radioTx));
else
    fprintf('S1a: connecting to S2a''s simulated-channel server at %s:%d ...\n', ...
        config.simChannelHost, config.simChannelPort);
    simChannelClient = dvbs2TCPConnectRetry( ...
        config.simChannelHost, config.simChannelPort, 'S2a''s simulated-channel server');
    fprintf('S1a: connected.\n');
    % tcpclient/tcpserver objects close their connection when their
    % variable is cleared or goes out of scope -- unlike comm System
    % objects (radioTx above), they don't have a release()/delete() call
    % to hook via onCleanup, so no explicit cleanup object is set up
    % here; ending the script (or `clear simChannelClient`) is
    % sufficient.
    %
    % NO PACING PAUSE HERE ANY MORE. Sim mode used to pace itself against
    % real time after every write, because it was also the one generating
    % content and had nothing else bounding its rate. Now S1b paces
    % itself (see S1b_ACMControl.m), and S1a's own wait for the NEXT
    % block below already cannot run ahead of that -- pacing twice would
    % just double the delay for no benefit.
end

%% Uplink radio (RF mode only) -- acquires CLTUs, forwards only what it finds
uplinkRF = config.useSDR && config.uplink.useRF;
uplinkOverruns = 0;
uplinkChunksRead = 0;
% Samples the uplink radio has produced but this process has not collected
% yet, carried between iterations so the drain below never over- or
% under-reads. See the drain in the main loop for why this exists.
uplinkDebtSamples = 0;
% Airtime of the burst transmitted on the PREVIOUS iteration, which is what
% the drain credits itself from. Zero on the first pass -- there is no
% previous burst, so nothing is read and the radio's buffer simply starts
% filling. It self-corrects on the second iteration.
lastBurstSec = 0;

if uplinkRF
    u = config.uplink;
    radioUplinkRx = comm.SDRuReceiver( ...
        'Platform', config.usrp.platform, ...
        'IPAddress', u.rxIPAddress, ...
        'MasterClockRate', config.usrp.masterClockRate, ...
        'DecimationFactor', u.inter_decimateFactor, ...
        'CenterFrequency', u.centerFrequency, ...
        'Gain', u.rxGain, ...
        'OutputDataType', 'double', ...
        'SamplesPerFrame', u.rxFrameLength);
    cleanupUplinkRx = onCleanup(@() release(radioUplinkRx));
    fprintf('S1a: return link over RF -- receiving on %s at %.3f MHz, gain %g dB\n', ...
        u.rxIPAddress, u.centerFrequency/1e6, u.rxGain);

    % ccsdsUplinkAcquire and its DC blocker both hold state across calls.
    clear ccsdsUplinkAcquire ccsdsUplinkDCSuppress;
end

%% TX block input FROM S1b
% S1a hosts this -- it is the side that always needs to be listening,
% regardless of mode or of whether S1b has started generating yet.
%
% OPENED BEFORE THE OUTBOUND CONNECT BELOW, DELIBERATELY. Every other
% pair of scripts in this testbed can start in any order because each
% one's OWN server goes up before it ever tries to connect out to
% anyone else's -- so a circular wait can never form, no matter which
% process happens to start first. Measured on hardware: with this
% server opened AFTER the uplinkAcqClient connect below instead, S1a
% and S1b deadlocked outright -- each blocked connecting to a server
% the other hadn't opened yet, because it hadn't reached this line yet
% either, and neither ever would.
fprintf('S1a: opening TX block listener on port %d ...\n', config.txBlockPort);
txBlockServer = dvbs2TCPServerRetry(config.txBlockHost, config.txBlockPort, "S1a's TX block server");
txBlockClientLogged = false;

if uplinkRF
    fprintf('S1a: connecting to S1b''s uplink acquisition server at %s:%d ...\n', ...
        config.uplinkAcqHost, config.uplinkAcqPort);
    uplinkAcqClient = dvbs2TCPConnectRetry( ...
        config.uplinkAcqHost, config.uplinkAcqPort, "S1b's uplink acquisition server");
    fprintf('S1a: connected to S1b.\n');
end

runTic = tic;
airtimeSec = 0;
prof = struct('uplinkRadio', 0, 'uplinkDSP', 0, 'uplinkForward', 0, 'blockWait', 0, 'radioTxTime', 0, ...
    'reads', 0, 'emptyReads', 0, 'iters', 0, 'underruns', 0, 'blocksReceived', 0, ...
    'localDummyBlocksSent', 0);
% Uplink acquisition diagnostics (stages 1-5, see ccsdsUplinkAcquire.m) --
% aggregated across the whole run, the same figures S1b's profile used to
% report before acquisition moved here.
uplinkWindows = 0;
uplinkDetections = 0;
uplinkAsmLocks = 0;
uplinkDuplicates = 0;
uplinkBestCarrierDB = -Inf;
uplinkBestASMMetric = 0;
uplinkCLTUsForwarded = 0;

fprintf('\nS1a: starting radio I/O loop (%d-sample blocks) ...\n', txBlockSamples);

while true

    %% Stop after config.runDurationSec and report where the time went
    if toc(runTic) >= config.runDurationSec
        wall = toc(runTic);
        fprintf('\n=== S1a PROFILE === %.1f s wall | %.1f s airtime | RT factor %.3f | %d iterations\n', ...
            wall, airtimeSec, wall/max(airtimeSec,eps), prof.iters);
        accounted = prof.uplinkRadio + prof.uplinkDSP + prof.uplinkForward + prof.blockWait + prof.radioTxTime;
        fprintf('  uplink radio reads   %7.2f s  %5.1f%%   %d reads (%d empty), %.1f ms each\n', ...
            prof.uplinkRadio, 100*prof.uplinkRadio/wall, prof.reads, prof.emptyReads, ...
            1e3*prof.uplinkRadio/max(prof.reads,1));
        fprintf('  uplink acquisition   %7.2f s  %5.1f%%   %d windows, %d detections, %d ASM locks, %d duplicates\n', ...
            prof.uplinkDSP, 100*prof.uplinkDSP/wall, uplinkWindows, uplinkDetections, uplinkAsmLocks, uplinkDuplicates);
        fprintf('  uplink forward to S1b %6.2f s  %5.1f%%   %d CLTUs forwarded\n', ...
            prof.uplinkForward, 100*prof.uplinkForward/wall, uplinkCLTUsForwarded);
        fprintf('  waiting for TX block %7.2f s  %5.1f%%   %d blocks received\n', ...
            prof.blockWait, 100*prof.blockWait/wall, prof.blocksReceived);
        fprintf('  radioTx              %7.2f s  %5.1f%%\n', ...
            prof.radioTxTime, 100*prof.radioTxTime/wall);
        fprintf('  everything else      %7.2f s  %5.1f%%\n', ...
            wall - accounted, 100*(wall - accounted)/wall);
        fprintf('  --\n');
        fprintf('  TX underruns %d of %d blocks | uplink RX overruns %d | local dummy fallback %d blocks (%.1f%%)\n', ...
            prof.underruns, prof.blocksReceived, uplinkOverruns, ...
            prof.localDummyBlocksSent, 100*prof.localDummyBlocksSent/max(prof.blocksReceived,1));
        fprintf('  uplink acquisition quality: best carrier %.1f dB, best ASM metric %.3f\n', ...
            uplinkBestCarrierDB, uplinkBestASMMetric);
        fprintf('  per iteration: %.1f ms wall, %.1f ms of airtime produced\n\n', ...
            1e3*wall/max(prof.iters,1), 1e3*airtimeSec/max(prof.iters,1));
        break;
    end
    prof.iters = prof.iters + 1;

    % Yield to MATLAB's event queue before polling the servers below.
    % tcpserver accepts its client ASYNCHRONOUSLY, and that accept is only
    % processed when the event queue is serviced. In SDR mode this loop
    % otherwise never yields -- radioTx() blocks inside a MEX call and
    % fprintf does not service the queue -- so a pending accept is never
    % processed and txBlockServer.Connected stays false indefinitely, even
    % though the operating system has long since completed the TCP
    % handshake. 'limitrate' caps this at roughly 20 Hz, far more than
    % enough to notice a one-time connection without adding measurable
    % cost to the transmit path.
    drawnow limitrate;

    %% Drain the uplink radio, acquire CLTUs, forward only what's found
    % Unchanged pacing logic from before the split -- only what happens to
    % each read changed (acquire, not just relay). See the long comment
    % this used to carry (still in S1b_ACMControl.m's docstring) for why
    % the credit is taken from lastBurstSec (downlink airtime just
    % produced) rather than from wall-clock time: it is what keeps two
    % radios sharing one loop from starving each other.
    if uplinkRF
        uplinkDebtSamples = min( ...
            uplinkDebtSamples + lastBurstSec * config.uplink.sampleRate, ...
            config.uplink.maxDrainReads * config.uplink.rxFrameLength);
        nUplinkReads = floor(uplinkDebtSamples / config.uplink.rxFrameLength);
        uplinkDebtSamples = uplinkDebtSamples - nUplinkReads * config.uplink.rxFrameLength;

        for k = 1:nUplinkReads
            tRead = tic;
            [uRx, uLen, uOvr] = radioUplinkRx();
            prof.uplinkRadio = prof.uplinkRadio + toc(tRead);
            prof.reads = prof.reads + 1;
            if uOvr
                uplinkOverruns = uplinkOverruns + 1;
            end
            if uLen == 0
                % An empty return means the object does NOT block waiting for
                % a frame it hasn't got. Counting these is what tells us
                % which behaviour we are dealing with.
                prof.emptyReads = prof.emptyReads + 1;
                break;
            end
            uplinkChunksRead = uplinkChunksRead + 1;

            % Stages 1-5: DC suppress, coarse CFO, matched filter, optional
            % Gardner, ASM detect, Costas -- see ccsdsUplinkAcquire.m. Runs
            % on every read, continuously, because PLOP-2's carrier never
            % keys off; only what comes OUT of this (a detected CLTU, most
            % reads produce none at all) is worth sending anywhere.
            tDSP = tic;
            [cltus, aInfo] = ccsdsUplinkAcquire(uRx(1:uLen), config);
            prof.uplinkDSP = prof.uplinkDSP + toc(tDSP);
            uplinkWindows = uplinkWindows + aInfo.windows;
            uplinkDetections = uplinkDetections + aInfo.detections;
            uplinkAsmLocks = uplinkAsmLocks + aInfo.asmLocks;
            uplinkDuplicates = uplinkDuplicates + aInfo.duplicates;
            uplinkBestCarrierDB = max(uplinkBestCarrierDB, aInfo.bestCarrierDB);
            uplinkBestASMMetric = max(uplinkBestASMMetric, aInfo.bestASMMetric);

            tFwd = tic;
            for c = 1:numel(cltus)
                dvbs2TCPFrameWrite(uplinkAcqClient, dvbs2SerializeCLTU(cltus{c}));
                uplinkCLTUsForwarded = uplinkCLTUsForwarded + 1;
            end
            prof.uplinkForward = prof.uplinkForward + toc(tFwd);
        end
    end

    %% Wait for the next TX block from S1b -- non-blocking, with its own
    % duration check, same pattern as every other cross-process wait in
    % this testbed (see S2b_Reciever.m's chunk read for why a plain
    % blocking read here would make config.runDurationSec unreliable).
    if ~txBlockClientLogged && txBlockServer.Connected
        fprintf('S1a: S1b has connected to the TX block server.\n');
        txBlockClientLogged = true;
    end
    tWait = tic;
    txWaveform = complex([]);
    usedLocalDummy = false;
    while true
        blockBytes = dvbs2TCPFrameTryRead(txBlockServer);
        if ~isempty(blockBytes)
            txWaveform = dvbs2BytesToComplex(blockBytes);
            break;
        end
        if toc(runTic) >= config.runDurationSec
            break;   % let the top-of-loop guard print the profile and stop
        end
        if toc(tWait) >= localDummyWaitThresholdSec
            % S1b hasn't produced a block in time -- expected whenever its
            % own new-data budget is spent for this tick or it just
            % dropped a stale pending block (see its pipeline comments).
            % The radio must not go silent either way, so synthesize a
            % dummy PLFRAME locally rather than keep waiting for a block
            % that may not be coming this tick at all.
            txWaveform = dvbs2DummyFiller(txBlockSamples, ...
                config.dvbs2.SamplesPerSymbol, config.dvbs2.RolloffFactor, 10);
            txWaveform = txWaveform(1:txBlockSamples);
            usedLocalDummy = true;
            break;
        end
        pause(0.001);
    end
    prof.blockWait = prof.blockWait + toc(tWait);
    if isempty(txWaveform)
        continue;
    end
    prof.blocksReceived = prof.blocksReceived + 1;
    if usedLocalDummy
        prof.localDummyBlocksSent = prof.localDummyBlocksSent + 1;
    end

    %% Transmit dispatch
    if config.useSDR
        tTx = tic;
        underrun = radioTx(txWaveform);
        prof.radioTxTime = prof.radioTxTime + toc(tTx);
        if underrun
            % Counted, not warned. Printing one warning per block put
            % hundreds of console writes into the measurement, which is
            % slow enough to be its own confound -- and the count is what
            % matters.
            prof.underruns = prof.underruns + 1;
            if prof.underruns == 1
                fprintf(2, 'S1a: TX underruns have started (block %d); counting silently from here.\n', ...
                    prof.blocksReceived);
            end
        end
    else
        % Sim mode: stream the CLEAN samples to S2a, which owns the
        % simulated channel model -- mirrors SDR mode above, where this
        % script hands its waveform straight to the radio and everything
        % downstream of the antenna belongs to the receiving side. Sent
        % as a continuous byte stream with no message framing, since it
        % stands in for a continuous RF/sample stream rather than
        % discrete messages.
        write(simChannelClient, dvbs2ComplexToBytes(txWaveform), 'uint8');
    end

    lastBurstSec = numel(txWaveform)/Fsamp;
    airtimeSec = airtimeSec + lastBurstSec;
    if mod(prof.blocksReceived, 10) == 0
        if uplinkRF
            fprintf(['S1a: block %d transmitted | RT factor %.3f | ' ...
                'uplink %d reads, %d CLTUs found, %d RX overruns\n'], ...
                prof.blocksReceived, toc(runTic)/max(airtimeSec,eps), ...
                uplinkChunksRead, uplinkCLTUsForwarded, uplinkOverruns);
        else
            fprintf('S1a: block %d transmitted | RT factor %.3f\n', ...
                prof.blocksReceived, toc(runTic)/max(airtimeSec,eps));
        end
    end
end
