%UPLINKTX Standalone CCSDS Telecommand uplink transmitter, PLOP-2.
%
%   Transmits CONTINUOUSLY on 500 MHz. The carrier never drops:
%
%     acquisition sequence -> CLTU -> idle -> idle -> CLTU -> idle ...
%
%   A command is queued every config.uplink.beaconPeriodSec, alternating the
%   two the uplink carries:
%
%     report    link quality: frames summarised, mean and sigma SNR, RSSI
%     request   retransmission of a frame or a range of frames
%
%   Both are 5 bytes and both fit one LDPC(128,64) codeword, so they cost
%   identical airtime. The values sent sweep over their ranges so a receiver
%   can check that the numbers survived the link, not merely that something
%   arrived.
%
%   THIS IS A ONE-HOP BRING-UP TEST. Nothing else needs to be running: no S1,
%   no S2, no downlink. That is the point -- in the integrated testbed the
%   uplink only carries traffic once the receiver has locked the 2 GHz
%   downlink, so a broken uplink and a broken downlink produce exactly the
%   same symptom and cannot be told apart. Proving each RF hop on its own
%   first removes that ambiguity.
%
%   Bring-up order:
%     1. this script + UplinkRx.m          proves the 500 MHz path
%     2. S1 + S2a + S2                     proves the 2 GHz path
%     3. all four                          only once both halves are proven
%
%   WATCH THE UNDERRUN COUNT. A burst transmitter could underrun harmlessly
%   between bursts; a continuous one cannot. Any underrun here is a real gap
%   in the carrier, and a gap can drop the receiver's loops -- costing far
%   more than the samples that went missing. If the count climbs, raise
%   config.uplink.txBlockSamples.
%
%   Stop with Ctrl+C.

clear; clc;

addpath(genpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Functions')));

config = dvbs2TestbedConfig();
u = config.uplink;

% Both hold state across calls -- the sample buffer and the shaping filter's
% memory. Cleared so a fresh run cannot inherit a half-finished CLTU or a
% filter tail from a previous one.
clear ccsdsUplinkTxStream ccsdsUplinkPulseShape;

fr = ccsdsUplinkFraming(config);
fprintf('UplinkTx: CCSDS TC, coherent BPSK, %s(%d,%d), PLOP-2 continuous\n', ...
    u.channelCoding, fr.codewordLength, fr.infoLength);
fprintf('UplinkTx: %g sym/s, %g samples/symbol, %.1f ksps, RRC rolloff %.2f\n', ...
    u.symbolRate, u.samplesPerSymbol, u.sampleRate/1e3, u.rolloffFactor);
fprintf('UplinkTx: acquisition %d symbols (%.0f ms), CLTU %d symbols (%.0f ms), min idle %d symbols\n', ...
    u.plop.acquisitionSymbols, 1e3*u.plop.acquisitionSymbols/u.symbolRate, ...
    fr.cltuSymbols, 1e3*fr.cltuSymbols/u.symbolRate, u.plop.minIdleSymbols);
fprintf('UplinkTx: occupied bandwidth about %.1f kHz, block %d samples (%.0f ms)\n', ...
    u.symbolRate*(1+u.rolloffFactor)/1e3, u.txBlockSamples, ...
    1e3*u.txBlockSamples/u.sampleRate);
fprintf('UplinkTx: one command every %.2f s\n\n', u.beaconPeriodSec);

radioTx = comm.SDRuTransmitter( ...
    'Platform', config.usrp.platform, ...
    'IPAddress', u.txIPAddress, ...
    'MasterClockRate', config.usrp.masterClockRate, ...
    'InterpolationFactor', u.inter_decimateFactor, ...
    'CenterFrequency', u.centerFrequency, ...
    'Gain', u.txGain);
cleanupTx = onCleanup(@() release(radioTx));
fprintf('UplinkTx: transmitting on %s at %.3f MHz, gain %g dB\n', ...
    u.txIPAddress, u.centerFrequency/1e6, u.txGain);
fprintf('UplinkTx: START GAIN LOW and raise in 3 dB steps.\n\n');

seq = 0;
underruns = 0;
peakSeen = 0;
runTic = tic;
lastQueue = tic;
lastReport = tic;

while true
    % Queue a command when one is due. Nothing else changes -- the stream
    % keeps flowing either way, which is exactly the difference between
    % PLOP-2 and what this script used to do.
    newPayloads = {};
    label = '';
    if toc(lastQueue) >= u.beaconPeriodSec
        lastQueue = tic;
        if mod(seq, 2) == 0
            count = mod(seq, 200) + 1;
            meanSNR = -20 + mod(seq*0.75, 45);
            sigmaSNR = mod(seq*0.125, 4);
            rssi = -80 + mod(seq*0.5, 70);
            newPayloads = {ccsdsUplinkCommand("report", count, meanSNR, sigmaSNR, rssi)};
            label = sprintf('report  count=%3d mean=%+6.2f sigma=%4.2f rssi=%+6.2f', ...
                count, meanSNR, sigmaSNR, rssi);
        else
            startIdx = mod(seq*37, 100000);
            endIdx = startIdx + mod(seq, 300);
            newPayloads = {ccsdsUplinkCommand("request", startIdx, endIdx)};
            label = sprintf('request [%d..%d]', startIdx, endIdx);
        end
        seq = seq + 1;
    end

    [block, info] = ccsdsUplinkTxStream(u.txBlockSamples, newPayloads, config);
    peakSeen = max(peakSeen, info.peak);

    % An underrun here is NOT harmless. This is a continuous carrier, so a
    % starved radio leaves a hole in it.
    if radioTx(block)
        underruns = underruns + 1;
    end

    if ~isempty(label)
        fprintf('UplinkTx: %4d queued | %6.1f s | %s\n', seq, toc(runTic), label);
    end

    if toc(lastReport) >= 5.0
        lastReport = tic;
        fprintf('UplinkTx: %6.1f s | %d CLTUs sent | queued %d | peak %.2f | underruns %d\n', ...
            toc(runTic), info.cltusSent, info.queued, peakSeen, underruns);
        peakSeen = 0;
    end
end
