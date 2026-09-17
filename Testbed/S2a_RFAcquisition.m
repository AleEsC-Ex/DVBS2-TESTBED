% S2A_RFACQUISITION Acquisition + receive front end, feeding S2b_Reciever.m's DSP process.

clear; clc;

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Functions')));

config = dvbs2TestbedConfig();

% Reset the persistent state of the stateful front-end stages, so a fresh.
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

% configureDVBS2Channel only reads SamplesPerSymbol and RolloffFactor.
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
uplinkRF = config.useSDR && config.uplink.useRF;
uplinkTxCount = 0;
uplinkUnderruns = 0;
% Split of what the uplink actually carried. Channel reports are periodic.
uplinkFeedbackCount = 0;
uplinkRetransmitCount = 0;
if uplinkRF
    u = config.uplink;

% Both hold state across calls -- the sample buffer and the shaping.
    clear ccsdsUplinkTxStream ccsdsUplinkPulseShape;

    fprintf('S2a: opening ACM feedback listener on port %d (relayed over RF uplink) ...\n', ...
        config.feedbackPort);
    feedbackServer = dvbs2TCPServerRetry(config.feedbackHost, config.feedbackPort, ...
        "S2a's feedback server");
    fprintf('S2a: opening retransmit-request listener on port %d (relayed over RF uplink) ...\n', ...
        config.retransmitPort);
    retransmitServer = dvbs2TCPServerRetry(config.retransmitHost, config.retransmitPort, ...
        "S2a's retransmit-request server");

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
% comm.AGC locks its expected input length on first call and errors on a.
agcInputLen = [];

% Samples awaiting emission, accumulated until a full DSP chunk is ready.
outBuf = [];
% Raw (pre-AGC) power accumulated over the acquisition blocks feeding the.
powerSum = 0;
powerCount = 0;
% Last RSSI actually measured, reused by a chunk that the accumulator had.
lastRSSIdB = NaN;

fprintf('\nS2a: starting acquisition loop ...\n');

chunkNum = 0;
blockNum = 0;
overrunCount = 0;

% Real-time accounting: airtimeSec is how many seconds of RF this process.
runTic = tic;
airtimeSec = 0;

% Same split as S1a's: seconds spent inside each specific call, so a blocking.
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
        fprintf('  uplink payloads: %d channel reports + %d retransmit requests = %d\n\n', ...
            uplinkFeedbackCount, uplinkRetransmitCount, ...
            uplinkFeedbackCount + uplinkRetransmitCount);
        break;
    end
    prof.iters = prof.iters + 1;

% Yield to MATLAB's event queue before the blocking radio call below.
    drawnow limitrate;

    %% 1. Acquire one block of raw samples
    if config.useSDR
        tRx = tic;
        [newSamples, validLen, overrun] = radioRx();
        prof.radioRx = prof.radioRx + toc(tRx);
        if overrun
            overrunCount = overrunCount + 1;
            %warning('S2a:RXOverrun', 'RX overrun detected -- samples were dropped by the radio/host link.');
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

% Apply the simulated channel here, where a real receiver would.
        newSamples = configureDVBS2Channel(newSamples, cfgForChannel, simParams);
    end
    blockNum = blockNum + 1;
    airtimeSec = airtimeSec + numel(newSamples)/Fsamp;

    %% 2. DC block (SDR only)
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

% One acquisition block can fill MORE THAN ONE DSP chunk (100000.
        if powerCount > 0
            rssiDB = 10*log10(powerSum / powerCount);
            lastRSSIdB = rssiDB;
            powerSum = 0;
            powerCount = 0;
        else
            rssiDB = lastRSSIdB;
        end

% Raw-sample-domain coarse CFO compensation. Applied to the whole.
        tOut = tic;
% Blind coarse CFO, applied to the whole assembled chunk (not per.
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
% rtSrv/rtBytes are diagnostic -- kept because they are what.
                fprintf(['S2a: chunk %d | RSSI=%.2f dB | rawCFO=%.1f Hz | %d blocks | ' ...
                    'overruns %d | uplink %d sent, %d underruns | ' ...
                    'rt srv conn=%d bytes=%d | RT factor %.3f\n'], ...
                    chunkNum, rssiDB, cfoEstHz, blockNum, overrunCount, ...
                    uplinkTxCount, uplinkUnderruns, ...
                    retransmitServer.Connected, retransmitServer.NumBytesAvailable, ...
                    toc(runTic)/max(airtimeSec,eps));
            else
                fprintf('S2a: chunk %d | RSSI=%.2f dB | rawCFO=%.1f Hz | %d blocks | overruns %d | RT factor %.3f\n', ...
                    chunkNum, rssiDB, cfoEstHz, blockNum, overrunCount, toc(runTic)/max(airtimeSec,eps));
            end
        end
    end

    %% 6. Keep the RF uplink fed
    if uplinkRF
        newPayloads = {};

% Anything queued on either server goes out verbatim. Both message.
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

% An underrun here is a real hole in a carrier that is supposed to.
        if radioUplinkTx(uplinkBlock)
            uplinkUnderruns = uplinkUnderruns + 1;
        end
        prof.uplinkTx = prof.uplinkTx + toc(tUp);
    end
end
