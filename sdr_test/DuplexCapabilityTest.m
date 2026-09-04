%DUPLEXCAPABILITYTEST Can one USRP transmit and receive from one MATLAB process?
%
%   This is the decisive architectural test before any uplink work. It
%   answers two questions that between them determine the whole
%   transceiver design:
%
%     TEST A (this script): can a SINGLE MATLAB process hold both a
%       comm.SDRuTransmitter and a comm.SDRuReceiver on the SAME radio
%       IP, and sustain both without the receive buffer running away?
%
%     TEST B (manual, see below): does a SECOND MATLAB process opening
%       that same IP fail with a device-busy error?
%
%   WHY IT MATTERS: published work on this exact hardware (Genesys Lab,
%   CrownCom -- "Implementing a MATLAB-based Self-Configurable Software
%   Defined Radio Transceiver") reports that the SDRu System objects'
%   step methods are single-threaded while the N210 is not, so per call
%   you can put a frame OR get a frame but not both. Running them
%   sequentially in a loop is reported to give an exponentially
%   increasing delay and eventually a USRP buffer overflow. If that
%   happens here, the return link has to be time-division (S1 pauses
%   transmitting to listen in a scheduled window) rather than
%   frequency-division. So this script does not just check that the
%   objects can be CREATED -- it measures whether the loop keeps up.
%
%   The rates below deliberately mirror the intended asymmetry: the .2
%   radio transmits the wideband DVB-S2 downlink and receives only the
%   narrowband control return, so its receiver runs at a much higher
%   decimation than its transmitter's interpolation.
%
%   RF SAFETY: transmitting and receiving on the SAME radio couples the
%   transmitter directly into the receiver's front end -- far more
%   strongly than any over-the-air path. txGain is pinned to the minimum
%   below for that reason. Run this with the antennas DETACHED (or with
%   attenuators fitted) unless you have checked the radio's maximum RF
%   input rating; this test only cares whether the software can sustain
%   both streams, not whether the signal is recoverable.

clear; clc;

addpath(genpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Functions')));
config = dvbs2TestbedConfig();

radioIP = config.usrp.txIPAddress;   % the .2 radio -- downlink TX, return-link RX

txCenterHz = config.usrp.centerFrequency;          % downlink,   2.000 GHz
rxCenterHz = config.usrp.centerFrequency + 20e6;   % return link, 2.020 GHz (FDD offset)

txInterp = config.usrp.inter_decimateFactor;   % 50  -> 2 Msps, matches the downlink
rxDecim  = 200;                                % 200 -> 500 ksps, narrowband return

txBlockLen = 20000;
rxBlockLen = 5000;
nIter = 200;

fprintf('=== USRP duplex capability test ===\n');
fprintf('radio            : %s\n', radioIP);
fprintf('TX  %.3f MHz, interp %d  -> %.3f Msps, %d samples/block\n', ...
    txCenterHz/1e6, txInterp, config.usrp.masterClockRate/txInterp/1e6, txBlockLen);
fprintf('RX  %.3f MHz, decim  %d -> %.3f Msps, %d samples/block\n\n', ...
    rxCenterHz/1e6, rxDecim, config.usrp.masterClockRate/rxDecim/1e6, rxBlockLen);

%% Create both objects on the SAME radio
fprintf('Creating transmitter ...\n');
try
    radioTx = comm.SDRuTransmitter( ...
        'Platform', config.usrp.platform, ...
        'IPAddress', radioIP, ...
        'MasterClockRate', config.usrp.masterClockRate, ...
        'InterpolationFactor', txInterp, ...
        'CenterFrequency', txCenterHz, ...
        'Gain', 0);                       % minimum -- see RF SAFETY above
    cleanupTx = onCleanup(@() release(radioTx));
    fprintf('  transmitter created.\n');
catch ME
    fprintf(2, 'RESULT: transmitter could not be created: %s\n', ME.message);
    return;
end

fprintf('Creating receiver on the SAME IP ...\n');
try
    radioRx = comm.SDRuReceiver( ...
        'Platform', config.usrp.platform, ...
        'IPAddress', radioIP, ...
        'MasterClockRate', config.usrp.masterClockRate, ...
        'DecimationFactor', rxDecim, ...
        'CenterFrequency', rxCenterHz, ...
        'Gain', config.usrp.rxGain, ...
        'OutputDataType', 'double', ...
        'SamplesPerFrame', rxBlockLen);
    cleanupRx = onCleanup(@() release(radioRx));
    fprintf('  receiver created.\n\n');
catch ME
    fprintf(2, ['RESULT: FAILED -- a receiver cannot coexist with a transmitter\n' ...
        '        on the same radio in one process.\n        %s\n'], ME.message);
    fprintf(2, '  => the return link must be TIME-DIVISION, not frequency-division.\n');
    return;
end

%% Sustained alternating TX/RX
% A tone is fine -- content is irrelevant, only whether both streams keep up.
n = (0:txBlockLen-1).';
txBlock = 0.2 * exp(1j*2*pi*0.01*n);

iterSec    = zeros(nIter,1);
underruns  = 0;
overruns   = 0;
validTotal = 0;

fprintf('Running %d alternating TX/RX iterations ...\n', nIter);
for k = 1:nIter
    t0 = tic;
    u = radioTx(txBlock);
    [~, validLen, o] = radioRx();
    iterSec(k) = toc(t0);

    underruns  = underruns + double(u ~= 0);
    overruns   = overruns  + double(o ~= 0);
    validTotal = validTotal + double(validLen);

    if mod(k, 50) == 0
        fprintf('  iter %3d | %.2f ms | underruns %d | overruns %d\n', ...
            k, iterSec(k)*1e3, underruns, overruns);
    end
end

%% Verdict
% Discard a warm-up window before judging anything. Opening the UHD session
% costs roughly half a second spread over the first handful of iterations,
% and averaging that into a 200-iteration run inflates the mean by more than
% 2x -- enough to report a comfortably real-time loop as "MARGINAL".
nWarm   = 20;
steady  = iterSec(nWarm+1:end);
first10 = mean(iterSec(1:10));
last10  = mean(iterSec(end-9:end));
growth  = last10 / max(first10, eps);
txAirtime = txBlockLen / (config.usrp.masterClockRate/txInterp);
rtSteady  = mean(steady)/txAirtime;

fprintf('\n=== results ===\n');
fprintf('  mean incl. warm-up  : %.2f ms   (start-up transient, not representative)\n', mean(iterSec)*1e3);
fprintf('  mean after warm-up  : %.2f ms   <- the number that matters\n', mean(steady)*1e3);
fprintf('  median after warm-up: %.2f ms\n', median(steady)*1e3);
fprintf('  first 10 / last 10  : %.2f ms / %.2f ms  (growth x%.2f)\n', ...
    first10*1e3, last10*1e3, growth);
fprintf('  TX block airtime    : %.2f ms  -> steady real-time factor %.3f\n', ...
    txAirtime*1e3, rtSteady);
fprintf('  underruns / overruns: %d / %d  of %d iterations\n', underruns, overruns, nIter);
fprintf('  mean valid RX samples per call: %.0f of %d\n', validTotal/nIter, rxBlockLen);
fprintf(['\n  Note: individual iterations are bimodal. radioTx() returns at once while the\n' ...
    '  transmit buffer has room and blocks when it is full, so the loop alternates between\n' ...
    '  fast passes and airtime-paced ones. Keeping up is decided by the MEAN, not by any\n' ...
    '  single sample.\n']);

fprintf('\n=== verdict ===\n');
if growth > 1.5
    fprintf(2, '  FAIL: iteration time grew x%.2f across the run -- this is the\n', growth);
    fprintf(2, '        documented runaway. Use a TIME-DIVISION return link.\n');
elseif overruns > nIter/10
    fprintf(2, '  FAIL: %d overruns -- the receive buffer is not being drained.\n', overruns);
    fprintf(2, '        Use a TIME-DIVISION return link.\n');
elseif rtSteady > 1.0
    fprintf(2, '  MARGINAL: stable, but slower than real time (%.3fx after warm-up).\n', rtSteady);
    fprintf(2, '        Either poll the receiver less often (DuplexPollingTest.m) or\n');
    fprintf(2, '        reduce the downlink rate.\n');
else
    fprintf('  PASS: both streams sustained, timing stable, %.3fx real time after warm-up.\n', rtSteady);
    fprintf('        Frequency-division full duplex is viable on this hardware, receiving\n');
    fprintf('        on every pass. DuplexPollingTest.m will quantify the spare margin.\n');
end

fprintf(['\nTEST B (run separately): with this script still running, start a\n' ...
    'second MATLAB and try to create ANY SDRu object on %s. If it fails with\n' ...
    'a device-busy error, each radio must be owned by exactly one process --\n' ...
    'which means that radio''s TX and RX cannot be split across two scripts.\n'], radioIP);
