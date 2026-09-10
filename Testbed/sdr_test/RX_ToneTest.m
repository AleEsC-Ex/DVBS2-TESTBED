%RX_TONETEST Receives via the RX-role USRP and detects the transmitted test tone.
%
%   Run alongside TX_ToneTest.m (as its own MATLAB instance, either
%   script started first) to verify the TX and RX radios can stream at
%   the same time. Each captured block is FFT'd to find its strongest
%   tone, reported as a frequency offset from the center frequency and a
%   power level -- the offset lands near config.toneOffsetHz once the
%   two radios are actually linked over RF, with any residual difference
%   coming from the two radios' independent, unsynchronized local
%   oscillators.

clear; clc;

addpath(fileparts(mfilename('fullpath')));
config = sdrTestConfig();

radioRx = comm.SDRuReceiver( ...
    'Platform', config.rx.platform, ...
    'IPAddress', config.rx.ipAddress, ...
    'MasterClockRate', config.masterClockRate, ...
    'DecimationFactor', config.masterClockRate / config.fs, ...
    'CenterFrequency', config.centerFrequency, ...
    'Gain', config.rxGain, ...
    'OutputDataType', 'double', ...
    'SamplesPerFrame', config.captureLength);
cleanupRx = onCleanup(@() release(radioRx));

fprintf('RX: listening at %.3f MHz via %s, expecting a tone near %.1f kHz offset ...\n', ...
    config.centerFrequency/1e6, config.rx.ipAddress, config.toneOffsetHz/1e3);
fprintf('RX: Ctrl+C to stop.\n');

toleranceHz = 5000;   % allows for LO offset between the two independently-clocked radios
freqAxis = (-config.captureLength/2 : config.captureLength/2 - 1).' * (config.fs / config.captureLength);
captureNum = 0;

while true
    [samples, overrun] = radioRx();
    captureNum = captureNum + 1;
    if overrun
        warning('sdr_test:RXOverrun', 'RX overrun on capture %d.', captureNum);
    end

    % samples are normalized IQ (same [-1, 1]-ish convention as
    % comm.SDRuTransmitter's input): a level approaching full scale
    % means the front end is at or past compression, not cleanly
    % receiving a low-level tone -- stop and reduce txGain/rxGain rather
    % than continuing to increase transmit power past this point.
    peakSampleMag = max(abs(samples));
    if peakSampleMag > 0.9
        warning('sdr_test:PossibleSaturation', ...
            'capture %d: sample magnitude %.2f is near full scale -- reduce txGain/rxGain before raising power further.', ...
            captureNum, peakSampleMag);
    end

    spectrum = fftshift(fft(samples));
    [peakMag, peakIdx] = max(abs(spectrum));
    peakFreqHz = freqAxis(peakIdx);
    peakPowerDB = 20*log10(peakMag / config.captureLength);

    if abs(peakFreqHz - config.toneOffsetHz) < toleranceHz
        statusStr = 'TONE DETECTED';
    else
        statusStr = 'no tone at expected offset';
    end
    fprintf('RX: capture %d | peak at %+.1f kHz, %.1f dB | expected %+.1f kHz | %s\n', ...
        captureNum, peakFreqHz/1e3, peakPowerDB, config.toneOffsetHz/1e3, statusStr);
end
