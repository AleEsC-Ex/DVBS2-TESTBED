%ISOLATIONTEST_NEAR Can the transmitting radio hear the far radio over its own leakage?
%
%   Runs on the .2 radio, which in the planned transceiver transmits the
%   DVB-S2 downlink at 2000 MHz and receives the DBPSK return link at
%   2020 MHz. Timing feasibility is already established (DuplexPollingTest:
%   0.979x real time with ~46 ms spare per burst). What is NOT established
%   is whether a return signal survives 20 MHz away from a transmitter
%   sitting in the same box.
%
%   This measures two levels at the receive frequency:
%
%     idle  -- own transmitter fed zeros. The local oscillator still leaks,
%              so this is the LO-leakage-plus-receiver-noise floor.
%     busy  -- own transmitter fed a full-scale signal. This is the real
%              self-interference floor the return link has to clear.
%
%   RUN IT TWICE:
%     Pass 1, with IsolationTest_Far.m NOT running -> the two floors alone.
%     Pass 2, with IsolationTest_Far.m running     -> the far tone on top.
%
%   Then compare. The number that decides the architecture is how far the
%   far tone sits above the busy floor:
%
%     > 15 dB   comfortable, build the FDD return link as planned
%     5-15 dB   workable but tight -- more attenuation or wider spacing
%     < 5 dB    the far signal is buried in own leakage; FDD will not work
%               and the return link must be time-division instead
%
%   RF SAFETY: attenuators must be fitted. txGain starts low deliberately --
%   raise it in small steps and re-check for the saturation warning below
%   rather than jumping straight to the operating value.

clear; clc;
addpath(genpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Functions')));
config = dvbs2TestbedConfig();

radioIP     = config.usrp.txIPAddress;      % .2
txCenterHz  = config.usrp.centerFrequency;      % 2000 MHz, the downlink
rxCenterHz  = config.usrp.centerFrequency + 20e6;  % 2020 MHz, the return link
txInterp    = config.usrp.inter_decimateFactor;
rxDecim     = 200;
txGain      = 10;          % START LOW. Raise in 3 dB steps if needed.
toneOffsetHz = 100e3;      % must match IsolationTest_Far.m

txBlockLen = 20000;
rxBlockLen = 8192;         % power of two: clean FFT
nBlocks    = 60;
rxRate     = config.usrp.masterClockRate/rxDecim;

fprintf('=== self-interference / isolation, near radio ===\n');
fprintf('radio %s\n', radioIP);
fprintf('  own TX %.3f MHz  gain %g dB\n', txCenterHz/1e6, txGain);
fprintf('  own RX %.3f MHz  gain %g dB  @ %.3f Msps\n', rxCenterHz/1e6, config.usrp.rxGain, rxRate/1e6);
fprintf('  looking for a tone %+.0f kHz from the receive centre\n\n', toneOffsetHz/1e3);

radioTx = comm.SDRuTransmitter( ...
    'Platform', config.usrp.platform, 'IPAddress', radioIP, ...
    'MasterClockRate', config.usrp.masterClockRate, ...
    'InterpolationFactor', txInterp, 'CenterFrequency', txCenterHz, 'Gain', txGain);
cleanupTx = onCleanup(@() release(radioTx));

radioRx = comm.SDRuReceiver( ...
    'Platform', config.usrp.platform, 'IPAddress', radioIP, ...
    'MasterClockRate', config.usrp.masterClockRate, ...
    'DecimationFactor', rxDecim, 'CenterFrequency', rxCenterHz, ...
    'Gain', config.usrp.rxGain, 'OutputDataType', 'double', ...
    'SamplesPerFrame', rxBlockLen);
cleanupRx = onCleanup(@() release(radioRx));

% Full-scale-ish DVB-S2-like load for the "busy" phase: a wideband random
% signal is a fairer stand-in for a real modulated downlink than a tone.
rng(1);
busyBlock = 0.5*(randn(txBlockLen,1) + 1j*randn(txBlockLen,1))/sqrt(2);
busyBlock = busyBlock / max(abs(busyBlock)) * 0.7;
idleBlock = zeros(txBlockLen,1);

freqAxis = (-rxBlockLen/2 : rxBlockLen/2-1).' * (rxRate/rxBlockLen);
[~, toneBin] = min(abs(freqAxis - toneOffsetHz));
guard = 6;                              % bins either side counted as "tone"
toneIdx = max(1,toneBin-guard) : min(rxBlockLen,toneBin+guard);
dcIdx   = rxBlockLen/2+1 + (-12:12);    % exclude DC / LO spur from the floor

results = struct('name',{},'total',{},'tone',{},'floor',{},'snr',{},'peak',{});

for phase = 1:2
    if phase == 1
        name = 'idle (TX zeros)'; blk = idleBlock;
    else
        name = 'busy (TX loaded)'; blk = busyBlock;
    end

    for k = 1:15, radioTx(blk); radioRx(); end          % settle

    tot = zeros(nBlocks,1); tn = zeros(nBlocks,1); fl = zeros(nBlocks,1); pk = 0;
    for k = 1:nBlocks
        radioTx(blk);
        [rx, ~, ~] = radioRx();
        pk = max(pk, max(abs(rx)));
        tot(k) = mean(abs(rx).^2);

        S = abs(fftshift(fft(rx))).^2 / rxBlockLen^2;
        tn(k) = max(S(toneIdx));
        m = S; m(toneIdx) = NaN; m(dcIdx) = NaN;
        fl(k) = median(m, 'omitnan');
    end

    r.name  = name;
    r.total = 10*log10(median(tot));
    r.tone  = 10*log10(median(tn));
    r.floor = 10*log10(median(fl));
    r.snr   = r.tone - r.floor;
    r.peak  = pk;
    results(end+1) = r; %#ok<SAGROW>

    fprintf('%-18s  total %7.2f dB | tone bin %7.2f dB | floor %7.2f dB | tone-over-floor %6.2f dB\n', ...
        name, r.total, r.tone, r.floor, r.snr);
    if pk > 0.9
        warning('IsolationTest:Saturation', ...
            '%s: peak sample magnitude %.2f is near full scale -- reduce rxGain or add attenuation.', name, pk);
    end
end

%% Interpretation
selfInt = results(2).total - results(1).total;
fprintf('\n=== what changed when the transmitter switched on ===\n');
fprintf('  wideband level rose by %.2f dB\n', selfInt);
fprintf('  noise floor rose by    %.2f dB\n', results(2).floor - results(1).floor);
fprintf('  tone-bin level rose by %.2f dB\n', results(2).tone - results(1).tone);

fprintf('\n=== how to read this ===\n');
fprintf('  Run 1 (far radio SILENT): both rows are pure leakage. "tone-over-floor"\n');
fprintf('    should be near 0 dB -- there is no tone to find. If it is not, something\n');
fprintf('    else is transmitting near %.3f MHz.\n', (rxCenterHz+toneOffsetHz)/1e6);
fprintf('  Run 2 (far radio TRANSMITTING): the busy row is the operating condition.\n');
fprintf('    Its tone-over-floor is the margin the DBPSK return link actually gets.\n');
fprintf('      >15 dB  build FDD as planned\n');
fprintf('      5-15 dB workable but tight -- more attenuation or wider TX/RX spacing\n');
fprintf('      <5 dB   buried in own leakage; the return link must be time-division\n');
fprintf('\n  DBPSK needs roughly 10 dB for a 1e-5 bit error rate, and the control\n');
fprintf('  messages are short and idempotent, so a few dB below that still passes\n');
fprintf('  usable feedback through.\n');
