%ISOLATIONTEST_FAR Far-end transmitter for the isolation measurement.
%
%   Runs on the .3 radio and does one thing: transmit a steady tone on the
%   return-link frequency so IsolationTest_Near.m can measure how far above
%   the near radio's own leakage it lands.
%
%   Start this FIRST, leave it running, then run IsolationTest_Near.m in a
%   second MATLAB. Stop it with Ctrl+C between the two passes described in
%   that script's header.
%
%   Each radio must be owned by exactly one MATLAB process -- a second
%   process on the same IP constructs its objects happily and then fails at
%   the first transmit with "fifo ctrl timed out looking for acks", which
%   names nothing useful about the cause. This script therefore drives the
%   .3 radio and nothing else.
%
%   RF SAFETY: attenuators must be fitted. txGain starts low; raise it in
%   small steps only if the near radio cannot see the tone at all.

clear; clc;
addpath(genpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Functions')));
config = dvbs2TestbedConfig();

radioIP      = config.usrp.rxIPAddress;              % .3
centerHz     = config.usrp.centerFrequency + 20e6;   % 2020 MHz, the return link
txInterp     = config.usrp.inter_decimateFactor;
txGain       = 10;            % START LOW, raise in 3 dB steps
toneOffsetHz = 100e3;         % must match IsolationTest_Near.m
blockLen     = 20000;

txRate = config.usrp.masterClockRate/txInterp;

fprintf('=== isolation test, far transmitter ===\n');
fprintf('radio %s\n', radioIP);
fprintf('  TX %.3f MHz + %.0f kHz tone, gain %g dB, %.3f Msps\n', ...
    centerHz/1e6, toneOffsetHz/1e3, txGain, txRate/1e6);
fprintf('  Ctrl+C to stop.\n\n');

radioTx = comm.SDRuTransmitter( ...
    'Platform', config.usrp.platform, 'IPAddress', radioIP, ...
    'MasterClockRate', config.usrp.masterClockRate, ...
    'InterpolationFactor', txInterp, 'CenterFrequency', centerHz, 'Gain', txGain);
cleanupTx = onCleanup(@() release(radioTx));

% Phase is carried across blocks so the tone is one unbroken sinusoid rather
% than a series of independently-phased bursts.
sampleIdx = 0;
blockNum  = 0;
underruns = 0;

while true
    n = (sampleIdx : sampleIdx + blockLen - 1).';
    txBlock = 0.6 * exp(1j*2*pi*(toneOffsetHz/txRate)*n);
    sampleIdx = sampleIdx + blockLen;

    if radioTx(txBlock), underruns = underruns + 1; end

    blockNum = blockNum + 1;
    if mod(blockNum, 200) == 0
        fprintf('far TX: %d blocks sent, %d underruns\n', blockNum, underruns);
    end
end
