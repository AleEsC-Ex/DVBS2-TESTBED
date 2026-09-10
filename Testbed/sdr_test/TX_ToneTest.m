%TX_TONETEST Transmits a continuous single-tone test signal via the TX-role USRP.
%
%   Run alongside RX_ToneTest.m (as its own MATLAB instance, either
%   script started first) to verify the TX and RX radios can stream at
%   the same time. This transmits a plain complex sinusoid, offset from
%   the center frequency by config.toneOffsetHz -- no waveform
%   generation or demodulation logic is involved, so a clean detection
%   on the receive side isolates radio connectivity and RF-path issues
%   from anything specific to a real payload waveform.
%
%   CAUTION: sdrTestConfig.m's txGain defaults to its minimum
%   deliberately -- at a few meters over the air in a closed room, path
%   loss alone is not reliable protection for the receive radio's front
%   end (small-room reflections can add coupling on top of free-space
%   loss), and directional patch antennas pointed straight at each other
%   couple noticeably more power than lower-gain omni antennas would at
%   the same distance. Aim the two antennas off-boresight from each
%   other (rather than face-on) for this test, and cross-polarize them
%   (one rotated 90 degrees relative to the other) for further isolation
%   essentially for free. Raise txGain gradually in small steps (2-3 dB),
%   re-running RX_ToneTest.m after each step and watching its
%   saturation/overrun warnings, rather than starting at a high gain.

clear; clc;

addpath(fileparts(mfilename('fullpath')));
config = sdrTestConfig();

radioTx = comm.SDRuTransmitter( ...
    'Platform', config.tx.platform, ...
    'IPAddress', config.tx.ipAddress, ...
    'MasterClockRate', config.masterClockRate, ...
    'InterpolationFactor', config.masterClockRate / config.fs, ...
    'CenterFrequency', config.centerFrequency, ...
    'Gain', config.txGain);
cleanupTx = onCleanup(@() release(radioTx));

fprintf('TX: streaming a %.1f kHz tone at %.3f MHz via %s ...\n', ...
    config.toneOffsetHz/1e3, config.centerFrequency/1e6, config.tx.ipAddress);
fprintf('TX: Ctrl+C to stop.\n');

blockLen = 10000;
sampleIdx = 0;
blockNum = 0;

% sampleIdx keeps the tone's phase continuous from one block to the
% next, so the transmitted signal is one unbroken sinusoid rather than a
% series of independently-phased bursts.
while true
    n = (sampleIdx : sampleIdx + blockLen - 1).';
    txBlock = config.txAmplitude * exp(1j*2*pi*(config.toneOffsetHz/config.fs)*n);
    sampleIdx = sampleIdx + blockLen;

    underrun = radioTx(txBlock);
    blockNum = blockNum + 1;
    if underrun
        warning('sdr_test:TXUnderrun', 'TX underrun on block %d.', blockNum);
    end
    if mod(blockNum, 100) == 0
        fprintf('TX: %d blocks transmitted.\n', blockNum);
    end
end
