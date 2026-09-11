%S2A_RFACQUISITION Acquisition + receive front end, feeding S2b_Reciever.m's DSP process.
%
%   This process owns everything between "where samples come from" and
%   "symbol-domain DSP", for BOTH testbed modes:
%
%     config.useSDR = true   samples come from the USRP via radioRx().
%     config.useSDR = false  samples arrive from S1a_Transmitter.m as a
%                            clean transmitted waveform, and THIS process
%                            applies the simulated channel impairment
%                            model (Functions/configureDVBS2Channel.m).
%
%   Putting the channel model here rather than in S1a mirrors the real
%   hardware split: in SDR mode S1a hands its waveform to the radio and
%   everything downstream of that -- propagation, front-end effects --
%   belongs to the receiving side. Sim mode now has the same shape, so
%   the two modes differ only in this script's acquisition step and not
%   in the pipeline's structure.
%
%   After acquisition it runs the per-sample front-end stages that used
%   to live in S2b_Reciever.m:
%     1) DC blocking (SDR only -- LO leakage has no simulated analog)
%     2) RSSI, measured BEFORE AGC (AGC deliberately erases absolute
%        power information, so it can only be measured here)
%     3) AGC
%     4) Raw-sample-domain coarse CFO compensation
%        (Functions/dvbs2RawCFOCompensate.m) -- a feed-forward BLOCK
%        estimate with no state carried between calls, applied to the
%        assembled chunk immediately before it is sent, which is exactly
%        where S2b used to apply it. It has to stay ahead of the matched
%        filter (see that function's header for why), and the matched
%        filter is the first thing S2b does, so this is still the last
%        possible point for it.
%   and forwards the result to S2b_Reciever.m as one framed message per
%   block (Functions/Serialization/dvbs2SerializeAcqChunk.m), carrying
%   the samples together with the RSSI and CFO estimate measured for them.
%
%   WHY THE SPLIT IS HERE: these stages are all streaming and
%   chunk-boundary-safe (each carries its own state across calls), so
%   they can run in this process without changing their results, while
%   everything from the matched filter onward needs the cross-chunk
%   symbol buffer that S2b_Reciever.m owns. Moving them off S2b also halves
%   nothing on its own -- the point is that this process was measured at
%   ~1.5% of real time, i.e. almost entirely idle, while S2b carries the
%   bulk of the pipeline's cost.
%
%   Blocks are accumulated to config.rfAcqReadChunkLength before being
%   sent, so S2b's DSP chunk size stays exactly what it was regardless of
%   how many samples each individual acquisition call returns -- the
%   radio is still drained as fast as radioRx() will allow.
%
%   Run alongside S1a_Transmitter.m/S2b_Reciever.m/S3_ProcessingUnit.m,
%   in any order.

clear; clc;

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Functions')));

config = dvbs2TestbedConfig();

% Reset the persistent state of the stateful front-end stages, so a fresh
% run doesn't inherit a DC-blocker or channel-model CFO phase from a
% previous run in the same MATLAB session.
clear dvbs2DCBlock;
clear configureDVBS2Channel;
clear dvbs2RawCFOCompensate;

% Sample rate seen at the receive front end, needed by the raw CFO stage.
Fsym = config.chanBW / (1 + config.dvbs2.RolloffFactor);
Fsamp = Fsym * config.dvbs2.SamplesPerSymbol;

%% Acquisition source setup
if config.useSDR
    radioRx = comm.SDRuReceiver( ...
        'Platform', config.usrp.platform, ...
        'IPAddress', config.usrp.rxIPAddress, ...
        'MasterClockRate', config.usrp.masterClockRate, ...
        'DecimationFactor', config.usrp.inter_decimateFactor, ...
        'CenterFrequency', config.usrp.centerFrequency, ...
        'Gain', config.usrp.rxGain, ...
        'OutputDataType', 'double', ...
        'SamplesPerFrame', config.chunkLength);
    cleanupRx = onCleanup(@() release(radioRx));
    fprintf('S2a: receiving via USRP at %s (CenterFrequency=%.3f MHz)\n', ...
        config.usrp.rxIPAddress, config.usrp.centerFrequency/1e6);
else
    fprintf('S2a: opening simulated-channel server on port %d, waiting for S1a to connect ...\n', ...
        config.simChannelPort);
    simChannelServer = dvbs2TCPServerRetry(config.simChannelHost, config.simChannelPort, ...
        "S2a's simulated-channel server");
    while ~simChannelServer.Connected
        pause(0.1);
    end
    fprintf('S2a: S1a connected.\n');

    % configureDVBS2Channel only reads SamplesPerSymbol and RolloffFactor
    % off its cfgDVBS2 argument, so a plain struct stands in for the full
    % waveform-generator System object (which belongs to S1a and has no
    % reason to exist in this process).
    cfgForChannel.SamplesPerSymbol = config.dvbs2.SamplesPerSymbol;
    cfgForChannel.RolloffFactor = config.dvbs2.RolloffFactor;
    simParams = config.simChannel;
    simParams.chanBW = config.chanBW;
end

%% Downstream link to S2b's DSP process
fprintf('S2a: opening RF acquisition stream on port %d, connecting to S2b ...\n', config.rfAcqPort);
acqClient = dvbs2TCPConnectRetry(config.rfAcqHost, config.rfAcqPort, "S2b's RF acquisition server");
fprintf('S2a: connected to S2b.\n');

%% RF uplink transmitter (config.uplink.useRF only)
% S2a becomes the gateway for everything travelling back to S1a. It hosts
% the two servers S1a used to host, and relays whatever arrives on them over
% the 500 MHz uplink.
%
% S2B AND S3 ARE UNCHANGED BY THIS. They still connect to
% config.feedbackHost:feedbackPort and config.retransmitHost:retransmitPort
% exactly as before -- only which process BINDS those ports moves. Neither
% has any idea whether its bytes go over loopback or over the air, which is
% the whole reason the ports were specified this way.
%
% NO RE-SERIALISATION HAPPENS HERE. S2b sends dvbs2SerializeFeedback bytes
% and S3 sends dvbs2SerializeRetransmitRequest bytes; both are 5 bytes and
% both fit one LDPC(128,64) codeword's 64 information bits. They are handed
% to the uplink verbatim and arrive at S1a as the same 5 bytes, so S1a can
% deserialize them with the functions it already uses. The uplink is a
% transparent pipe, not a protocol layer.
uplinkRF = config.useSDR && config.uplink.useRF;
uplinkTxCount = 0;
uplinkUnderruns = 0;
% Split of what the uplink actually carried. Channel reports are periodic
% ACM feedback from S2b; retransmit requests are ARQ from S3 and are the
% symptom of the forward link losing frames.
uplinkFeedbackCount = 0;
uplinkRetransmitCount = 0;
% Diagnostics for config.uplink.txBudgetFraction (see ccsdsUplinkTxStream.m):
% how deep the real-CLTU queue is right now, the measured cost of encoding
% one, and how many calls have had to hold queued CLTUs back this run
% because the budget was already spent. A growing queue depth alongside a
% climbing capped count means the budget is too tight for the current
% retransmit/feedback traffic -- worth widening; RX overruns climbing
% instead means it is still too loose.
uplinkQueueDepth = 0;
uplinkSecPerCltu = 0;
uplinkBudgetCappedCount = 0;
if uplinkRF
    u = config.uplink;

    % Both hold state across calls -- the sample buffer and the shaping
    % filter's memory.
    clear ccsdsUplinkTxStream ccsdsUplinkPulseShape;

    fprintf('S2a: opening ACM feedback listener on port %d (relayed over RF uplink) ...\n', ...
        config.feedbackPort);
    feedbackServer = dvbs2TCPServerRetry(config.feedbackHost, config.feedbackPort, ...
        "S2a's feedback server");
    fprintf('S2a: opening retransmit-request listener on port %d (relayed over RF uplink) ...\n', ...
        config.retransmitPort);
    retransmitServer = dvbs2TCPServerRetry(config.retransmitHost, config.retransmitPort, ...
        "S2a's retransmit-request server");

    % BLOCK HERE UNTIL S3 CONNECTS, the same way S1a/S1b already block on
    % their own servers before doing any real work. Without this, S2a's
    % main loop (below) starts relaying real downlink acquisition chunks
    % to S2b the moment S2b's server answers -- regardless of whether S3
    % (last in the launch order, so typically the slowest to be ready) has
    % connected yet at all. S2b's own frame-hand-off connect to S3 already
    % blocks S2b's *script* from reaching its main loop until S3 answers,
    % but that did nothing to stop S2a from racing ahead and building up a
    % backlog in the meantime -- this closes that gap from S2a's side too,
    % so the receive chain (S2a/S2b/S3) only starts actually working once
    % every part of it, including S3, is genuinely ready.
    fprintf('S2a: waiting for S3 to connect to the retransmit-request listener ...\n');
    while ~retransmitServer.Connected
        pause(0.1);
    end
    fprintf('S2a: S3 connected.\n');

    radioUplinkTx = comm.SDRuTransmitter( ...
        'Platform', config.usrp.platform, ...
        'IPAddress', u.txIPAddress, ...
        'MasterClockRate', config.usrp.masterClockRate, ...
        'InterpolationFactor', u.inter_decimateFactor, ...
        'CenterFrequency', u.centerFrequency, ...
        'Gain', u.txGain);
    cleanupUplinkTx = onCleanup(@() release(radioUplinkTx));
    fprintf('S2a: uplink transmitting on %s at %.3f MHz, gain %g dB, %d samples/block (%.1f ms)\n', ...
        u.txIPAddress, u.centerFrequency/1e6, u.txGain, u.txBlockSamples, ...
        1e3*u.txBlockSamples/u.sampleRate);
end

%% Front-end state
agc = comm.AGC;
% comm.AGC locks its expected input length on first call and errors on a
% later call with a different length unless released first -- track it,
% since an acquisition block's length can vary (comm.SDRuReceiver's valid
% sample count, or the simulated channel's SCO resampling stage).
agcInputLen = [];

% Samples awaiting emission, accumulated until a full DSP chunk is ready.
outBuf = [];
% Raw (pre-AGC) power accumulated over the acquisition blocks feeding the
% chunk currently being assembled. Summing power here rather than keeping
% a second copy of the pre-AGC samples keeps this O(1) in memory; the
% only cost is that a chunk's RSSI covers exactly the acquisition blocks
% that contributed to it, whose boundaries need not align perfectly with
% the emitted chunk's. RSSI is a slow-moving relative dB figure, so that
% boundary difference is immaterial.
powerSum = 0;
powerCount = 0;
% Last RSSI actually measured, reused by a chunk that the accumulator had
% already been emptied for -- see the emit loop below.
lastRSSIdB = NaN;

fprintf('\nS2a: starting acquisition loop ...\n');

chunkNum = 0;
blockNum = 0;
overrunCount = 0;

% Real-time accounting: airtimeSec is how many seconds of RF this process
% has handled, wall-clock is how long it took. Their ratio is the
% real-time factor -- below 1.0 means this process keeps up with the
% radio, above 1.0 means it cannot and samples are being lost upstream
% (a USRP overrun in SDR mode, TCP backpressure in sim mode).
runTic = tic;
airtimeSec = 0;

% Same split as S1a's: seconds spent inside each specific call, so a blocking
% radio can be told apart from expensive processing.
prof = struct('radioRx', 0, 'frontEnd', 0, 'tcpOut', 0, 'uplinkTx', 0, 'iters', 0);

while true

    %% Stop after config.runDurationSec and report where the time went
    if toc(runTic) >= config.runDurationSec
        wall = toc(runTic);
        accounted = prof.radioRx + prof.frontEnd + prof.tcpOut + prof.uplinkTx;
        fprintf('\n=== S2a PROFILE === %.1f s wall | %.1f s airtime | RT factor %.3f | %d iterations\n', ...
            wall, airtimeSec, wall/max(airtimeSec,eps), prof.iters);
        fprintf('  radioRx (2 GHz)      %7.2f s  %5.1f%%   %.1f ms per call\n', ...
            prof.radioRx, 100*prof.radioRx/wall, 1e3*prof.radioRx/max(prof.iters,1));
        fprintf('  front end (DC/AGC)   %7.2f s  %5.1f%%\n', prof.frontEnd, 100*prof.frontEnd/wall);
        fprintf('  CFO + TCP to S2b     %7.2f s  %5.1f%%\n', prof.tcpOut, 100*prof.tcpOut/wall);
        fprintf('  uplink TX (500 MHz)  %7.2f s  %5.1f%%\n', prof.uplinkTx, 100*prof.uplinkTx/wall);
        fprintf('  everything else      %7.2f s  %5.1f%%\n', ...
            wall - accounted, 100*(wall - accounted)/wall);
        fprintf('  --\n  RX overruns %d | uplink %d CLTUs sent, %d underruns\n', ...
            overrunCount, uplinkTxCount, uplinkUnderruns);
        fprintf('  uplink payloads: %d channel reports + %d retransmit requests = %d\n', ...
            uplinkFeedbackCount, uplinkRetransmitCount, ...
            uplinkFeedbackCount + uplinkRetransmitCount);
        fprintf(['  uplink CLTU budget (config.uplink.txBudgetFraction=%.2f): ' ...
            'capped %d of %d chunks, %.1f ms/CLTU measured, %d still queued at end\n\n'], ...
            config.uplink.txBudgetFraction, uplinkBudgetCappedCount, chunkNum, ...
            1e3*uplinkSecPerCltu, uplinkQueueDepth);
        break;
    end
    prof.iters = prof.iters + 1;

    % Yield to MATLAB's event queue before the blocking radio call below.
    % tcpserver accepts its client ASYNCHRONOUSLY, and that accept is only
    % processed when the event queue is serviced -- see S1a's identical
    % comment for the full mechanism. This loop never yielded on its own
    % (radioRx() blocks inside a MEX call), which is why
    % retransmitServer.Connected stayed false for an ENTIRE run on hardware
    % even as bytes visibly piled up in its buffer (NumBytesAvailable
    % climbing while Connected read 0): S3's connection request, which
    % only arrives well into the run once S3 first detects a gap, was never
    % being accepted. Zero retransmit requests were relayed the whole run
    % as a result. 'limitrate' caps this at roughly 20 Hz, matching S1a/S1b.
    drawnow limitrate;

    %% 1. Acquire one block of raw samples
    if config.useSDR
        tRx = tic;
        [newSamples, validLen, overrun] = radioRx();
        prof.radioRx = prof.radioRx + toc(tRx);
        if overrun
            overrunCount = overrunCount + 1;
            warning('S2a:RXOverrun', 'RX overrun detected -- samples were dropped by the radio/host link.');
        end
        if validLen < length(newSamples)
            newSamples = newSamples(1:validLen);
        end
    else
        bytesPerBlock = config.chunkLength * 16;   % 16 bytes/complex sample, see dvbs2ComplexToBytes.m
        while simChannelServer.Connected && simChannelServer.NumBytesAvailable < bytesPerBlock
            pause(0.001);
        end
        if simChannelServer.NumBytesAvailable < bytesPerBlock
            fprintf('S2a: simulated-channel link closed by S1a; stopping.\n');
            break;
        end
        rawBytes = read(simChannelServer, bytesPerBlock, 'uint8');
        newSamples = dvbs2BytesToComplex(rawBytes);

        % Apply the simulated channel here, where a real receiver would
        % instead be seeing the effects of the actual RF path.
        newSamples = configureDVBS2Channel(newSamples, cfgForChannel, simParams);
    end
    blockNum = blockNum + 1;
    airtimeSec = airtimeSec + numel(newSamples)/Fsamp;

    %% 2. DC block (SDR only)
    % LO leakage in the USRP's direct-conversion front end puts a large
    % spurious spike at exactly 0 Hz that otherwise dominates RSSI and
    % every downstream stage. The simulated channel has no such artifact.
    tFE = tic;
    if config.useSDR
        newSamples = dvbs2DCBlock(newSamples);
    end

    %% 3. RSSI accumulation -- must happen BEFORE the AGC below
    powerSum = powerSum + sum(abs(newSamples).^2);
    powerCount = powerCount + numel(newSamples);

    %% 4. AGC
    if ~isempty(agcInputLen) && agcInputLen ~= length(newSamples)
        release(agc);
    end
    agcInputLen = length(newSamples);
    newSamples = agc(newSamples);
    prof.frontEnd = prof.frontEnd + toc(tFE);

    %% 5. Accumulate and emit whole DSP chunks to S2b
    outBuf = [outBuf; newSamples]; %#ok<AGROW>
    while numel(outBuf) >= config.rfAcqReadChunkLength
        block = outBuf(1:config.rfAcqReadChunkLength);
        outBuf(1:config.rfAcqReadChunkLength) = [];

        % One acquisition block can fill MORE THAN ONE DSP chunk (100000
        % samples covers 1.5 chunks of 66564), and the accumulator is
        % emptied by the first of them -- so the second had powerCount = 0
        % and reported 10*log10(0) = -Inf. That is not a harmless log
        % artifact: it travelled into the ACM report, where the serializer
        % clamped it to -96 dB, so every other feedback message claimed the
        % signal had vanished. Carry the last real measurement instead;
        % RSSI is a slow-moving figure and the samples in question were
        % genuinely covered by it.
        if powerCount > 0
            rssiDB = 10*log10(powerSum / powerCount);
            lastRSSIdB = rssiDB;
            powerSum = 0;
            powerCount = 0;
        else
            rssiDB = lastRSSIdB;
        end

        % Raw-sample-domain coarse CFO compensation. Applied to the whole
        % assembled chunk (not per acquisition block) so the estimate
        % covers exactly the same span it did when this ran in S2b.
        tOut = tic;
        % Blind coarse CFO, applied to the whole assembled chunk (not per
        % acquisition block) so the estimate covers exactly the same span it
        % did when this ran in S2b.
        %
        % OFF BY DEFAULT -- see config.rawCFOEnabled for the full reasoning.
        % In short: the M-th power estimator has to be told the modulation,
        % which S2a cannot know, and it produces wild values near its own
        % +-83 kHz ambiguity edge. One such chunk unlocks S2b's timing loop
        % permanently. Reporting zero is not a fudge: S2b adds this to its own
        % SOF-based estimate and subtracts it back out when applying, so a
        % zero simply hands the whole job to the estimator that can measure
        % it unambiguously.
        if config.rawCFOEnabled
            [block, cfoEstHz] = dvbs2RawCFOCompensate(block, Fsamp, "QPSK", ...
                config.rawCFOResolutionHz);
        else
            cfoEstHz = 0;
        end

        dvbs2TCPFrameWrite(acqClient, dvbs2SerializeAcqChunk(rssiDB, cfoEstHz, block));
        prof.tcpOut = prof.tcpOut + toc(tOut);

        chunkNum = chunkNum + 1;
        if mod(chunkNum, 10) == 0
            if uplinkRF
                % rtSrv/rtBytes are diagnostic -- kept because they are what
                % actually diagnosed the root cause: this loop never yielded
                % to MATLAB's event queue (see the drawnow above), so
                % retransmitServer's async client accept was never
                % processed. conn stayed 0 for a whole run while bytes
                % climbed anyway (OS-level data arriving, MATLAB object
                % never latching Connected), which is exactly what pointed
                % at the missing drawnow rather than a message-format or
                % port-binding problem.
                fprintf(['S2a: chunk %d | RSSI=%.2f dB | rawCFO=%.1f Hz | %d blocks | ' ...
                    'overruns %d | uplink %d sent, %d underruns | ' ...
                    'CLTU queue %d (%.1f ms/CLTU, capped %d) | ' ...
                    'rt srv conn=%d bytes=%d | RT factor %.3f\n'], ...
                    chunkNum, rssiDB, cfoEstHz, blockNum, overrunCount, ...
                    uplinkTxCount, uplinkUnderruns, ...
                    uplinkQueueDepth, 1e3*uplinkSecPerCltu, uplinkBudgetCappedCount, ...
                    retransmitServer.Connected, retransmitServer.NumBytesAvailable, ...
                    toc(runTic)/max(airtimeSec,eps));
            else
                fprintf('S2a: chunk %d | RSSI=%.2f dB | rawCFO=%.1f Hz | %d blocks | overruns %d | RT factor %.3f\n', ...
                    chunkNum, rssiDB, cfoEstHz, blockNum, overrunCount, toc(runTic)/max(airtimeSec,eps));
            end
        end
    end

    %% 6. Keep the RF uplink fed
    % LAST IN THE LOOP, AND EXACTLY ONE BLOCK PER ITERATION. This loop is
    % paced by radioRx(), which returns one chunkLength of downlink airtime
    % per call; txBlockSamples is sized to the same duration, so pushing one
    % block per iteration keeps the uplink running at real time without
    % either radio pulling ahead of the other.
    %
    % It goes last so the downlink drain and the hand-off to S2b -- both on a
    % hard deadline -- happen before this can block on the uplink radio.
    if uplinkRF
        newPayloads = {};

        % Anything queued on either server goes out verbatim. Both message
        % types share one uplink, which is what dvbs2MessageType's 3-bit
        % type field at the head of every message is for.
        % Counted separately as they are queued. The total CLTU count alone
        % cannot distinguish a healthy link -- where almost all uplink
        % traffic is periodic channel reports -- from a struggling one,
        % where retransmit requests dominate because S3 keeps finding gaps.
        % Both share one carrier, so the split is only visible here, at the
        % point the two servers are drained.
        if feedbackServer.Connected
            fbBytes = dvbs2TCPFrameTryRead(feedbackServer);
            if ~isempty(fbBytes)
                newPayloads{end+1} = fbBytes; %#ok<SAGROW>
                uplinkFeedbackCount = uplinkFeedbackCount + 1;
            end
        end
        if retransmitServer.Connected
            rtBytes = dvbs2TCPFrameTryRead(retransmitServer);
            if ~isempty(rtBytes)
                newPayloads{end+1} = rtBytes; %#ok<SAGROW>
                uplinkRetransmitCount = uplinkRetransmitCount + 1;
            end
        end

        tUp = tic;
        [uplinkBlock, uplinkInfo] = ccsdsUplinkTxStream( ...
            config.uplink.txBlockSamples, newPayloads, config);
        uplinkTxCount = uplinkInfo.cltusSent;
        uplinkQueueDepth = uplinkInfo.queued;
        uplinkSecPerCltu = uplinkInfo.secPerCltu;
        if uplinkInfo.cltuBudgetCapped
            uplinkBudgetCappedCount = uplinkBudgetCappedCount + 1;
        end

        % An underrun here is a real hole in a carrier that is supposed to
        % be continuous, so it is counted rather than ignored. A steadily
        % climbing count means txBlockSamples is too small for this loop's
        % actual cadence; RX overruns climbing instead means it is too big.
        if radioUplinkTx(uplinkBlock)
            uplinkUnderruns = uplinkUnderruns + 1;
        end
        prof.uplinkTx = prof.uplinkTx + toc(tUp);
    end
end
