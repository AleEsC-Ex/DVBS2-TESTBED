%DUPLEXPOLLINGTEST How often can the transmitting radio afford to listen?
%
%   DuplexCapabilityTest.m established that one MATLAB process CAN hold both
%   a transmitter and a receiver on one USRP, and that the loop is stable --
%   the "exponentially increasing delay" reported in the literature did not
%   appear. What it left open is the cost: that test received on EVERY pass,
%   which is far harder than the real duty cycle needs.
%
%   This measures the two numbers that actually decide the uplink design:
%
%     1. TX-only baseline  -- what one transmit burst costs with no receive.
%     2. RX polling sweep  -- the same loop receiving every Nth pass, for
%                             N = 1, 3, 10, 30.
%
%   From those, the largest N whose real-time factor stays under 1.0 is the
%   polling interval the return link can use.
%
%   REAL-TIME FACTOR is loop time divided by AIRTIME. Airtime is fixed by
%   physics -- samples / sampleRate -- and is how long the radio needs to
%   actually transmit the block. Above 1.0 means MATLAB cannot prepare
%   samples as fast as the radio consumes them, and the transmit buffer
%   eventually runs dry (an underrun).
%
%   The transmit block here is one real MODCOD-1 normal PLFRAME rather than
%   a round number, so the result maps straight onto what S1a actually sends.
%
%   RF SAFETY: transmitting and receiving on the SAME radio couples the
%   transmitter directly into its own front end. txGain is pinned to 0. Run
%   with antennas DETACHED or attenuators fitted -- this test only cares
%   about timing, not about whether any signal is recoverable.

clear; clc;

addpath(genpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Functions')));
config = dvbs2TestbedConfig();

radioIP    = config.usrp.txIPAddress;
txCenterHz = config.usrp.centerFrequency;
rxCenterHz = config.usrp.centerFrequency + 20e6;
txInterp   = config.usrp.inter_decimateFactor;
rxDecim    = 200;

txBlockLen = 66564;          % one MODCOD-1 normal PLFRAME at 2 sps
rxBlockLen = 5000;
nWarm      = 25;             % discarded: covers UHD initialisation
nMeas      = 120;

txRate     = config.usrp.masterClockRate/txInterp;
txAirtime  = txBlockLen/txRate;

fprintf('=== duplex polling cost ===\n');
fprintf('radio %s   TX %.3f MHz @ %.3f Msps   RX %.3f MHz @ %.3f Msps\n', ...
    radioIP, txCenterHz/1e6, txRate/1e6, rxCenterHz/1e6, ...
    config.usrp.masterClockRate/rxDecim/1e6);
fprintf('TX block %d samples = %.2f ms airtime (one MODCOD-1 PLFRAME)\n', ...
    txBlockLen, txAirtime*1e3);
fprintf('warm-up %d iterations discarded, then %d measured\n\n', nWarm, nMeas);

radioTx = comm.SDRuTransmitter( ...
    'Platform', config.usrp.platform, 'IPAddress', radioIP, ...
    'MasterClockRate', config.usrp.masterClockRate, ...
    'InterpolationFactor', txInterp, 'CenterFrequency', txCenterHz, 'Gain', 0);
cleanupTx = onCleanup(@() release(radioTx));

radioRx = comm.SDRuReceiver( ...
    'Platform', config.usrp.platform, 'IPAddress', radioIP, ...
    'MasterClockRate', config.usrp.masterClockRate, ...
    'DecimationFactor', rxDecim, 'CenterFrequency', rxCenterHz, ...
    'Gain', config.usrp.rxGain, 'OutputDataType', 'double', ...
    'SamplesPerFrame', rxBlockLen);
cleanupRx = onCleanup(@() release(radioRx));

n = (0:txBlockLen-1).';
txBlock = 0.2 * exp(1j*2*pi*0.01*n);

% pollEvery = 0 means never receive (the TX-only baseline)
pollList = [0 1 3 10 30];
res = zeros(numel(pollList), 4);

for pi_ = 1:numel(pollList)
    pollEvery = pollList(pi_);

    for k = 1:nWarm
        radioTx(txBlock);
        if pollEvery > 0 && mod(k, pollEvery) == 0, radioRx(); end
    end

    t = zeros(nMeas,1); under = 0; over = 0; rxCalls = 0; rxTime = [];
    for k = 1:nMeas
        t0 = tic;
        u = radioTx(txBlock);
        if pollEvery > 0 && mod(k, pollEvery) == 0
            % Timed separately. The whole-iteration figure cannot reveal this:
            % radioTx() blocks until the transmit buffer has room, so any work
            % done after it is absorbed by that call blocking for less time on
            % the next pass. The iteration lands at airtime either way, which
            % hides the receive cost rather than measuring it.
            tr = tic;
            [~, ~, o] = radioRx();
            rxTime(end+1) = toc(tr); %#ok<SAGROW>
            over = over + double(o ~= 0);
            rxCalls = rxCalls + 1;
        end
        t(k) = toc(t0);
        under = under + double(u ~= 0);
    end
    if isempty(rxTime), rxMed = 0; else, rxMed = median(rxTime); end

    % Median, not mean: one scheduling hiccup should not define the result.
    med = median(t);
    res(pi_,:) = [med, med/txAirtime, under, over];

    if pollEvery == 0
        lbl = 'TX only';
    else
        lbl = sprintf('RX every %d', pollEvery);
    end
    fprintf('%-14s median %6.2f ms  -> RT %5.3f  | RX call %5.2f ms | underruns %2d  overruns %2d  (%d calls)\n', ...
        lbl, med*1e3, med/txAirtime, rxMed*1e3, under, over, rxCalls);
end

%% Interpretation
txOnly = res(1,1);
fprintf('\n=== what this costs ===\n');
fprintf('  transmit alone            : %.2f ms per burst (RT %.3f)\n', txOnly*1e3, res(1,2));
fprintf(['  NOTE: if every row above lands at the same figure, the loop is being PACED BY THE\n' ...
    '  RADIO rather than by MATLAB -- radioTx() blocks until the transmit buffer has room, so\n' ...
    '  it soaks up whatever else the iteration did. That means the receive call is cheaper\n' ...
    '  than the spare capacity, not that it is free. The per-call column is the real cost.\n']);
fprintf(['  Remember the real transmitter also generates a DVB-S2 waveform each burst\n' ...
    '  (~19.6 ms measured for cfgDVBS2 + flushFilter + packet generation), which this test\n' ...
    '  does not -- subtract that from the spare capacity before judging the margin.\n']);

ok = find(res(:,2) < 1.0);
fprintf('\n=== verdict ===\n');
if isempty(ok)
    fprintf(2,'  Every configuration exceeds real time. The transmit path alone is too slow --\n');
    fprintf(2,'  reduce the sample rate before adding a receive path at all.\n');
else
    best = pollList(ok(end));
    if best == 0
        fprintf(2,'  Only TX-only stays under real time. Any receive on this radio costs more than\n');
        fprintf(2,'  the spare capacity -- the return link needs time-division, not polling.\n');
    else
        fprintf('  Largest affordable polling interval: receive every %d bursts (RT %.3f).\n', ...
            best, res(ok(end),2));
        fprintf('  At %.2f ms per burst that is one check every %.0f ms -- ample for a control\n', ...
            txAirtime*1e3, best*txAirtime*1e3);
        fprintf('  link carrying roughly 7 messages per second.\n');
    end
end
fprintf('\n  Reminder: real-time factor = loop time / airtime. Under 1.0 means MATLAB keeps\n');
fprintf('  ahead of the radio; over 1.0 means the transmit buffer slowly starves.\n');
