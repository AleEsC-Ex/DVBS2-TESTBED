function config = sdrTestConfig()
%SDRTESTCONFIG Shared network/hardware configuration for the sdr_test bring-up scripts.
%
%   config = sdrTestConfig() returns one struct loaded by every script in
%   this folder, so IP addresses, platform strings, and gain settings
%   never drift out of sync between independently-edited copies. Edit
%   this file (not the individual scripts) to reconfigure the test.
%
%   These scripts exist to validate, independently of any waveform
%   generation/demodulation logic, that two separate USRP radios can be
%   reached over the network and can transmit/receive at the same time --
%   a connectivity/RF bring-up check meant to run before pointing either
%   radio at any application waveform.

%% Radio identity
% The USRP-2920 (TX role) is a rebadged Ettus N210, addressed via
% Platform="N200/N210/USRP2" over UHD. VERIFY the USRP-2922 (RX role)
% reports the same Platform string, and that it supports the chosen
% centerFrequency below, before relying on any of this -- its RF
% frontend may not cover the same tunable range as the 2920's.
config.tx.ipAddress = '192.168.10.2';
config.tx.platform = 'N200/N210/USRP2';
config.rx.ipAddress = '192.168.10.3';
config.rx.platform = 'N200/N210/USRP2';   % VERIFY against the 2922's actual reported platform

config.masterClockRate = 100e6;   % Hz -- VERIFY against your specific units

%% RF settings
% For an over-the-air link at a few meters in a closed room, free-space
% path loss alone is NOT much protection (only ~46 dB at 2 m/2.4 GHz),
% and reflections off nearby walls/objects in a small room can add
% constructive coupling on top of that -- so these gains are deliberately
% conservative starting points, not tuned-for-performance values. This
% goes double with a pair of directional patch antennas (typically
% several dBi of gain each) pointed at each other: aim them off-boresight
% from one another for this bring-up test rather than face-on, since a
% simple link-detection test doesn't need peak antenna alignment and the
% off-axis rolloff buys real extra margin for free. Increase txGain
% manually in small steps (2-3 dB) from here, re-running RX_ToneTest.m
% after each step and watching for its saturation/overrun warnings,
% rather than jumping to a high gain outright. VERIFY your radios'
% actual maximum RF input power (no damage) rating from NI's datasheet
% and keep the received level comfortably under it.
config.centerFrequency = 2e9;   % Hz -- SET to your actual test frequency, within both radios' tunable range
config.txGain = 0;                % dB -- minimum; raise gradually, see note above
config.rxGain = 5;               % dB -- moderate starting point; adjust alongside txGain

%% Tone test settings
% masterClockRate/fs is used directly as the SDRu objects'
% interpolation/decimation factor, so it must come out to a whole number
% in whatever integer range your specific radios accept.
config.fs = 1e6;               % Hz, sample rate used for TX/RX streaming in this test
config.toneOffsetHz = 100e3;   % Hz, test tone's offset from centerFrequency
config.txAmplitude = 0.3;      % 0-1, kept well below full scale to avoid clipping/spurs
config.captureLength = 100000; % samples per receive call

end
