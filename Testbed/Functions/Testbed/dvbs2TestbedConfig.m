function config = dvbs2TestbedConfig()
%DVBS2TESTBEDCONFIG Single shared configuration for the transmitter/receiver/processing-unit testbed scripts.
%
%   config = dvbs2TestbedConfig() returns one struct that each of the
%   three testbed scripts loads at startup, so IP addresses, ports, the
%   shared RNG seed and ACM thresholds never drift out of sync between
%   independently-edited copies. Edit this file (not the individual
%   scripts) to reconfigure the testbed.
%
%   config.useSDR selects the whole testbed's mode:
%     false - simulated channel (Functions/configureDVBS2Channel.m)
%             streamed transmitter-to-receiver over a local TCP link, in
%             place of real hardware. Use this to validate the 3-process
%             IPC architecture and the ACM control loop before ever
%             touching a radio.
%     true  - real NI USRP-2920 hardware via comm.SDRuTransmitter /
%             comm.SDRuReceiver.

%% Top-level mode switch
config.useSDR = true;

%% TCP topology
% Each *Host value is used as BOTH the tcpserver's local bind address and the tcpclient's target address.

% Receiver -> transmitter ACM feedback.
% Scripts involved: S1a_Transmitter.m (server if useSDR=false) or S2a_RFAcquisition.m (server if useSDR=true), S2b_Reciever.m (client).
config.feedbackHost = '127.0.0.1';
config.feedbackPort = 30001;

% Receiver -> processing unit corrected-PLFRAME hand-off.
% Scripts involved: Processing unit (server), S2b_Reciever.m (client).
config.frameHost = '127.0.0.1';
config.framePort = 30002;

% Transmitter -> RF acquisition process clean transmitted samples.
% Scripts involved: S2a_RFAcquisition.m (server), S1a_Transmitter.m (client).
% NOTE: If useSDR is true, this TCP port/host will not be activated.
config.simChannelHost = '127.0.0.1';
config.simChannelPort = 30003;

% Processing unit -> transmitter retransmit requests (selective-repeat ARQ).
% Scripts involved: S1a_Transmitter.m or S2a_RFAcquisition.m (server depending on mode), Processing unit (client).
config.retransmitHost = '127.0.0.1';
config.retransmitPort = 30004;

% RF acquisition process -> receiver DSP process front-end samples and RSSI.
% Scripts involved: S2b_Reciever.m (server), S2a_RFAcquisition.m (client).
config.rfAcqHost = '127.0.0.1';
config.rfAcqPort = 30005;

% S1a -> S1b raw uplink samples.
% Scripts involved: S1b_ACMControl.m (server), S1a_Transmitter.m (client).
% NOTE: This TCP port/host will not be activated if config.uplink.useRF is false.
config.uplinkAcqHost = '127.0.0.1';
config.uplinkAcqPort = 30006;

% S1b -> S1a ready-to-transmit waveform blocks.
% Scripts involved: S1a_Transmitter.m (server), S1b_ACMControl.m (client).
config.txBlockHost = '127.0.0.1';
config.txBlockPort = 30007;

%% Shared data seed
% Used by both the transmitter and processing unit for local BER comparison data generation.
config.dataSeed = 12345;

%% USRP-2920 hardware settings (used only when useSDR = true)
% Ensure static IPs match your specific network setup.
config.usrp.txIPAddress = '192.168.10.2';
config.usrp.rxIPAddress = '192.168.10.3';
config.usrp.platform = 'N200/N210/USRP2';
config.usrp.masterClockRate = 100e6;   % Hz
config.usrp.centerFrequency = 2e9;     % Hz (S-band target)
config.usrp.txGain = 20;               % dB 
config.usrp.rxGain = 30;               % dB
config.usrp.inter_decimateFactor = 150;
config.usrp.sampleRate = config.usrp.masterClockRate/config.usrp.inter_decimateFactor;

%% DVB-S2 waveform base configuration
% Initial starting MODCOD and frame structural parameters.
config.dvbs2.StreamFormat = 'TS';
config.dvbs2.FECFrame = 'normal';
config.dvbs2.MODCOD = 1;
config.dvbs2.HasPilots = true;
config.dvbs2.SamplesPerSymbol = 2;
config.dvbs2.RolloffFactor = 0.35;
config.dvbs2.UPL = 1504;
config.chanBW = config.usrp.sampleRate/config.dvbs2.SamplesPerSymbol * (1+config.dvbs2.RolloffFactor);

%% Simulated-channel impairments (used only when useSDR = false)
config.simChannel.hasCFO = true;
config.simChannel.cfo = 50000;
config.simChannel.hasSCO = false;
config.simChannel.sco = 5;
config.simChannel.hasPN = false;
config.simChannel.phNoiseLevel = "Low";
config.simChannel.EsNodB = 30;

% Toggles time-varying satellite-pass profile. If false, static offsets above are applied.
config.simChannel.hasTimeVarying = false;
config.simChannel.passDurationSec = 40;   
config.simChannel.EsNodBPeak = 20;        
config.simChannel.EsNodBEdge = 4;         
config.simChannel.cfoPeakHz = 0;          

%% Burst / chunking parameters
% Samples read from the simulated channel or radio per acquisition chunk.
config.chunkLength = floor(66564/(2*config.dvbs2.SamplesPerSymbol));   
% Samples read by the DSP loop per iteration from the RF acquisition stream.
config.rfAcqReadChunkLength = 4*config.chunkLength;

%% Uplink: CCSDS Telecommand (Functions/Uplink/, sdr_test/UplinkTx+UplinkRx)
% Toggles closing the loop over the air instead of via TCP loopback.
config.uplink.useRF = true;

config.uplink.centerFrequency = 500e6;

% IPs for uplink transmission (2922) and reception (2920).
config.uplink.txIPAddress = config.usrp.rxIPAddress;   
config.uplink.rxIPAddress = config.usrp.txIPAddress;   

% Interpolation/decimation factors for the uplink hardware rate.
config.uplink.inter_decimateFactor = 500;
config.uplink.sampleRate = config.usrp.masterClockRate / config.uplink.inter_decimateFactor;

% Uplink symbol rate mapping.
config.uplink.samplesPerSymbol = 14; % Shall be an integer value.
config.uplink.symbolRate = config.uplink.sampleRate / config.uplink.samplesPerSymbol;

% Root-raised-cosine shaping variables.
config.uplink.rolloffFactor = 0.35;
config.uplink.filterSpanSymbols = 10;

% Channel coding and frame metadata settings.
config.uplink.channelCoding = "LDPC";
config.uplink.ldpcCodewordLength = 128;
config.uplink.hasRandomizer = true;
config.uplink.hasTailSequence = true; 

%% PLOP-2 -- the physical layer operations procedure (CCSDS 231.0-B)
% Carrier tracking lock and idle constraints.
config.uplink.plop.acquisitionSymbols = 1000;
config.uplink.plop.minIdleSymbols = 24;
config.uplink.plop.idleChunkSymbols = 256;
config.uplink.plop.startBit = 0;

% Samples handed to the radio per write, balanced specifically to match the S2a drain loop.
config.uplink.txBlockSamples = (320+config.uplink.plop.minIdleSymbols)*config.uplink.samplesPerSymbol;   
% Target RMS amplitude of the continuous transmitted stream.
config.uplink.txRMS = 0.3;

config.uplink.txGain = 10;   
config.uplink.rxGain = 30;

% Burst packaging limits and fixed buffer padding operations. 
% ccsdsTCIdealReceiver refuses any waveform whose length is not an integer
% multiple of samplesPerSymbol, silently producing a burst that can never be
% decoded rather than an error.

config.uplink.maxPayloadBytes = 8;
config.uplink.burstSamples = ...
    ceil(config.uplink.symbolRate*2/config.uplink.samplesPerSymbol) * config.uplink.samplesPerSymbol;   

%% Uplink receiver (Functions/Uplink/ccsdsUplinkReceive.m)
config.uplink.rxFrameLength = 8192;
config.uplink.maxDrainReads = 16;

% DC Blocker memory length logic setting.
config.uplink.dcCornerHz = 20;

% Phase tracking settings.
config.uplink.costasLoopBandwidthHz = 40;
config.uplink.costasDampingFactor = 1/sqrt(2);

% Optional timing synchronization (Gardner loop).
config.uplink.timingSyncEnabled = false;
config.uplink.gardnerLoopBandwidthHz = 120;
config.uplink.gardnerDampingFactor = 1/sqrt(2);

% Normalized correlation peak threshold for ASM detection.
config.uplink.asmThreshold = 0.50;

% Belief propagation constraints.
config.uplink.ldpcMaxIterations = 50;

% Detection boundary search parameters.
config.uplink.searchWindowSamples = 320*config.uplink.samplesPerSymbol*1.5; 
config.uplink.searchStrideSamples = config.uplink.searchWindowSamples/3; 

% Early-gate detection thresholds for the spectrum analyzer.
config.uplink.detectThresholdDB = 12;
config.uplink.maxCarrierOffsetHz = 25000;

config.uplink.beaconPeriodSec = 1.0;

%% Frame-sync detection gate
% Minimum normalized SOF correlation peak required to accept a lock.
config.frameSyncPeakThreshold = 0.7;

%% Carrier frequency offset: acquisition resolution and closed-loop tracking
% Determines the quantization bins for raw offsets.
config.rawCFOResolutionHz = 100;
% Blind, open-loop CFO estimation (disabled for static offsets).
config.rawCFOEnabled = false;

% Track scaling logic to react effectively to SOF variance.
config.cfoTrack.alpha = 0.1;
config.cfoTrack.beta = config.cfoTrack.alpha^2 / (2 - config.cfoTrack.alpha);
config.cfoTrack.gateFactor = 4;
config.cfoTrack.gateFloorHz = 200;
config.cfoTrack.minUpdatesBeforeGating = 10;
config.cfoTrack.maxConsecRejects = 5;

%% ACM policy parameters (see Functions/dvbs2ACMPolicy.m)
% Permitted MODCOD progression indexes.
config.acm.modcodSet = 1:28;
% Maximum steps allowed in a single shift to prevent link flapping.
config.acm.maxJumpRungs = 3;

% Security flags dictating what PLHEADER profiles S2b accepts.
config.rxReject.requirePilots      = true;
config.rxReject.requireNormalFrame = true;
config.rxReject.restrictToModcodSet = true;

% Acceptable dB variance gap between pilot and PLHEADER SNR estimates.
config.snr.maxDisagreementDB = 3;

% Samples parsed into radio buffer (prevents dynamic length renegotiations).
config.tx.blockSamples = 266256;
% Ceiling for airtime fraction spent generating new frames to prevent buffer stalls.
config.tx.genBudgetFraction = 0.45;

% Base Quasi-error-free (QEF) reference Es/No points per MODCOD.
config.acm.qefEsNodB = containers.Map(num2cell(1:28), { ...
    -2.35, -1.24, -0.30,  1.00,  2.23,  3.10,  4.03,  4.68, ... 
     5.18,  6.20,  6.42, ...                                     
     5.50,  6.62,  7.91,  9.35, 10.69, 10.98, ...                
     8.97, 10.21, 11.03, 11.61, 12.89, 13.13, ...                
    12.73, 13.64, 14.28, 15.69, 16.05 ...                        
    });

% Constraints for evaluating safe SNR thresholds using distributions.
config.acm.sigmaK = 1.0;
config.acm.sigmaKStay = 0.5;
config.acm.marginFloorDB = 0.5;
config.acm.historyLength = 50;

% Hysteresis timers/gates determining upward/downward switch aggressiveness.
config.acm.agreeCountUp = 3;      
config.acm.agreeCountDown = 1;    
config.acm.minDwellSecUp = 5;     
config.acm.minDwellSecDown = 0;   

%% Link establishment (SDR mode only)
% Frame successes required to confidently announce startup SNR limits.
config.calibLockFramesRequired = 5;

%% Feedback: event-driven, not periodic
% Outlier rejection gate prior to statistical batching.
config.acm.outlierMADs = 4;

% Value shifts enforcing an asynchronous report transmission.
config.acm.reportDeltaMeanDB = 1;    
config.acm.reportDeltaSigmaDB = 1.5;
config.acm.heartbeatSec = 3.0;

% Frame threshold bounding statistical batches.
config.acm.minFramesForChangeReport = 5;
% Threshold delay to flag the reverse link connection as broken.
config.acm.linkLossSec = 5.0;

% Feedback interpolation constraints.
config.acm.batchWindow = 10;
config.acm.trendMinBatches = 4;
config.acm.trendHorizonSec = 3.0;

%% Retransmission (selective-repeat ARQ)
% Discards requests indicating impossible logical bounds caused by packet CRC failures.
config.retransmit.maxRequestRange = 5000;
% Frame delay between resending unfulfilled outstanding block requests.
config.retransmit.recheckEveryFrames = 20;

%% Run limits
% Cap on total packets produced (Inf continuously transmits data).
config.maxFrames = Inf;
% Limit placed on execution length in seconds (Inf restricts by standard terminal interrupts).
config.runDurationSec = 420;

end