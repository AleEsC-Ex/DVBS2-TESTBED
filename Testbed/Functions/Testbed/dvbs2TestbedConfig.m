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

%% TCP topology (all loopback for now; change hosts if scripts move to separate machines)
% Each *Host value is used as BOTH the tcpserver's local bind address
% (by whichever script is the server for that link) and the tcpclient's
% target address (by whichever script is the client) -- '127.0.0.1'
% only accepts same-machine connections, which is correct while all
% three scripts run on one PC. Moving a script to another machine means
% changing the relevant server's bind address (e.g. to '0.0.0.0' to
% accept any interface) AND the corresponding client(s)' target address
% to that machine's real IP.
%
% Receiver -> transmitter ACM feedback. The receiver is always the
% tcpclient; WHICH PROCESS HOSTS THE SERVER DEPENDS ON THE MODE:
%
%   useSDR = false  S1a_Transmitter.m hosts it, and the report arrives
%                   straight over loopback.
%   useSDR = true   S2a_RFAcquisition.m hosts it, modulates the report
%                   (Functions/Return/) and transmits it on the 500 MHz
%                   return link; S1a recovers it off the air.
%
% Keeping the port and the client side identical in both modes is what lets
% S2b_Reciever.m and S3_ProcessingUnit.m stay completely unaware of which
% way their messages travel -- they write the same bytes to the same
% address either way.
config.feedbackHost = '127.0.0.1';
config.feedbackPort = 30001;

% Receiver -> processing unit corrected-PLFRAME hand-off: the processing
% unit is the tcpserver, the receiver is the tcpclient.
config.frameHost = '127.0.0.1';
config.framePort = 30002;

% Transmitter -> RF acquisition process CLEAN transmitted samples: only
% used when useSDR = false. S2a_RFAcquisition.m is the tcpserver,
% S1a_Transmitter.m is the tcpclient.
%
% S2a (not S1a, and not S2b) applies the simulated channel impairment
% model to these samples, mirroring the real-hardware split: in SDR mode
% S1a hands its waveform to the radio and everything downstream of the
% antenna belongs to the receiving side. Both modes therefore have the
% same process topology, differing only in S2a's acquisition step.
config.simChannelHost = '127.0.0.1';
config.simChannelPort = 30003;

% Processing unit -> transmitter retransmit requests (selective-repeat
% ARQ): the processing unit is the tcpclient, and as with the feedback
% link above the server is S1a in sim mode and S2a in SDR mode, where the
% request goes out over the 500 MHz return link. Independent of the other
% links -- the receiver is not involved in retransmission at all.
config.retransmitHost = '127.0.0.1';
config.retransmitPort = 30004;

% RF acquisition process (S2a_RFAcquisition.m) -> receiver DSP process
% (S2b_Reciever.m): front-end-processed samples plus their RSSI, used in
% BOTH modes. S2b_Reciever.m is the tcpserver, S2a_RFAcquisition.m is the
% tcpclient.
%
% Decoupling acquisition from the receiver's much heavier per-chunk DSP
% chain is what keeps the USRP's own host-side buffer drained promptly
% regardless of DSP time, rather than coupling radioRx()'s cadence
% directly to it. S2a also carries the per-sample front end (DC block,
% RSSI, AGC) -- all streaming, chunk-boundary-safe stages that produce
% identical results wherever they run, moved off S2b because S2a was
% measured at roughly 1.5% of real time while S2b carries the bulk of the
% pipeline's cost.
%
% Unlike the raw byte streams above, this link is message-framed
% (dvbs2TCPFrameWrite.m + dvbs2SerializeAcqChunk.m): each chunk's RSSI
% has to travel bound to the samples it was measured over.
config.rfAcqHost = '127.0.0.1';
config.rfAcqPort = 30005;

% S1a (owns both radios, generates nothing) -> S1b (owns the waveform
% generator, ACM control, AND uplink recovery): raw uplink samples, RF
% mode only (config.uplink.useRF). S1a does no demodulation of its own --
% it just drains radioUplinkRx() on the same pacing schedule it always
% did, and forwards whatever comes back. S1b is the tcpserver (same
% "processing side hosts" convention as S2b for S2a above), S1a the
% tcpclient.
%
% No dedicated serialize/deserialize pair needed -- unlike
% dvbs2SerializeAcqChunk.m's chunks, there is no RSSI or CFO estimate to
% carry alongside these samples (S1a does none of that processing), so
% this is just dvbs2ComplexToBytes.m/dvbs2BytesToComplex.m directly.
config.uplinkAcqHost = '127.0.0.1';
config.uplinkAcqPort = 30006;

% S1b (generates) -> S1a (transmits): one ready-to-transmit waveform
% block per message, config.tx.blockSamples complex samples, packed with
% dvbs2ComplexToBytes.m -- same "no wrapper needed" reasoning as the link
% above, just heavier (a few MB per message instead of a few bytes).
%
% S1a IS THE TCPSERVER HERE, not S1b -- the reverse of the link above,
% and deliberately so: S1a is the side that always needs to be listening
% regardless of when S1b is ready, since it has nothing else to transmit
% while it waits.
%
% MODCOD is no longer a message that crosses this link at all. Earlier
% revisions of this split kept generation in S1a and sent S1a a 1-byte
% MODCOD decision instead -- that meant every ACM decision had to be
% serialized, sent, and applied on the far end. Now that S1b owns
% cfgDVBS2 directly, a decision is just a local reconfiguration; nothing
% about it needs to leave this process. Likewise retransmit requests are
% consumed HERE now, not forwarded to S1a: validating one needs
% globalPktIdx, which used to live in S1a because that was where packets
% were generated -- now that generation is here too, so is
% globalPktIdx, and the validation that depends on it.
%
% THE PACING QUESTION THIS RAISES, AND HOW IT'S ANSWERED. Generation used
% to be implicitly paced by radioTx() blocking in the same process --
% remove that and there is nothing stopping this process from generating
% (and sending) far faster than S1a can actually transmit. S1b paces
% itself explicitly instead: one block, every
% config.tx.blockSamples/config.usrp.sampleRate seconds of real
% wall-clock time, by design rather than as a side effect this time. See
% the pacing comment in S1b_ACMControl.m's main loop for the mechanism.
config.txBlockHost = '127.0.0.1';
config.txBlockPort = 30007;

%% Shared data seed
% Used by BOTH the transmitter (to generate the transmitted bit pattern)
% and the processing unit (to regenerate the identical reference locally
% for bit-for-bit BER comparison), so no reference data ever needs to
% cross the network.
config.dataSeed = 12345;

%% USRP-2920 hardware settings (used only when useSDR = true)
% USRP-2920 is a rebadged Ettus N210; comm.SDRuTransmitter/Receiver
% address it via Platform="N200/N210/USRP2" over UHD. Each radio needs
% its own static IP on the same subnet as this host's NIC(s) --
% VERIFY these against your actual network setup before first use.
config.usrp.txIPAddress = '192.168.10.2';
config.usrp.rxIPAddress = '192.168.10.3';
config.usrp.platform = 'N200/N210/USRP2';
config.usrp.masterClockRate = 100e6;   % Hz -- VERIFY against your specific 2920 units
config.usrp.centerFrequency = 2e9;   % Hz -- S-band target; SET to your actual test frequency
config.usrp.txGain = 20;               % dB -- VERIFY/tune against your RF chain before transmitting
config.usrp.rxGain = 30;               % dB -- VERIFY/tune against your RF chain
config.usrp.inter_decimateFactor = 150;
config.usrp.sampleRate = config.usrp.masterClockRate/config.usrp.inter_decimateFactor;

%% DVB-S2 waveform base configuration
% Starting MODCOD; the transmitter's ACM loop may change MODCOD/DFL at
% burst boundaries away from this initial value (see dvbs2ACMPolicy.m).
config.dvbs2.StreamFormat = 'TS';
config.dvbs2.FECFrame = 'normal';
config.dvbs2.MODCOD = 1;
config.dvbs2.HasPilots = true;
config.dvbs2.SamplesPerSymbol = 2;
config.dvbs2.RolloffFactor = 0.35;
config.dvbs2.UPL = 1504;

config.chanBW = config.usrp.sampleRate/config.dvbs2.SamplesPerSymbol * (1+config.dvbs2.RolloffFactor);   % Hz, used to derive symbol rate

%% Simulated-channel impairments (used only when useSDR = false)
config.simChannel.hasCFO = true;
config.simChannel.cfo = 50000;
config.simChannel.hasSCO = false;
config.simChannel.sco = 5;
config.simChannel.hasPN = false;
config.simChannel.phNoiseLevel = "Low";
config.simChannel.EsNodB = 30;

% Time-varying satellite-pass profile. When false the two values above are
% held constant (the original behaviour). When true, Es/No and CFO instead
% follow the geometry of an overhead pass -- see configureDVBS2Channel.m:
% Es/No peaks and Doppler crosses zero at closest approach, Es/No is worst
% and Doppler largest (opposite signs) at the horizons. The pass clock
% counts AIRTIME, so a run is reproducible whatever the host's speed, and
% time past the end of one pass wraps into the next.
%
% This is what exercises the ACM policy's variance and trend terms and the
% CFO tracker's rate term -- neither of which has anything to do on a
% static channel.
config.simChannel.hasTimeVarying = false;
config.simChannel.passDurationSec = 40;   % compressed; a real LEO pass is ~10 min
config.simChannel.EsNodBPeak = 20;        % at closest approach
config.simChannel.EsNodBEdge = 4;         % at the horizons
% Peak Doppler in Hz, reached at both horizons with opposite signs. 0
% keeps the static config.simChannel.cfo instead. A real 2 GHz LEO link
% sees about 47 kHz; use a smaller value to isolate the ACM behaviour
% from the CFO tracker's.
config.simChannel.cfoPeakHz = 0;

%% Burst / chunking parameters
% SUPERSEDED by config.tx.blockSamples. S1a no longer transmits a fixed
% number of FRAMES per iteration -- it generates however many are needed to
% cover one fixed-size radio block, which is 4 at QPSK (what this used to
% be) and 6 at 8PSK. See localFramesPerBurst in S1a_Transmitter.m.
%
% Left here because the original reasoning is worth keeping: burstFrames had
% to stay small so a MODCOD switch landed promptly rather than waiting out
% one long waveform. That constraint is gone -- the MODCOD can now change on
% any frame boundary, since the radio never sees the frame length at all.
config.burstFrames = 4;   % retained for reference only; nothing reads it
% comm.SDRuReceiver's SamplesPerFrame, used only by S2a_RFAcquisition.m
% (config.useSDR) or directly by S2b_Reciever.m's simulated-channel read
% size (~useSDR). Its output is always SamplesPerFrame long, zero-padded
% past whatever it actually captures before the host stops draining the
% radio's buffer in time -- requesting far more than that leaves most of
% every chunk as zero-padding rather than real signal. Sized here from
% the actual measured ceiling (~16000 valid samples/chunk at the
% acquisition-only loop's cost, once decoupled from the DSP chain),
% with margin below it so chunks come back fully valid rather than
% truncated.
config.chunkLength = floor(2*66564/config.dvbs2.SamplesPerSymbol);   % samples, not symbols
% Samples S2b_Reciever.m's DSP loop reads per iteration from the RF
% acquisition stream (config.useSDR only) -- deliberately decoupled from
% config.chunkLength above (which sizes the acquisition process's own
% USRP-facing chunk) so it can be sized for DSP efficiency instead:
% larger reads mean fewer times per PLFRAME that dvbs2FrameSync has to
% re-run its correlation search while waiting for more of the frame to
% accumulate, rather than being capped by what the USRP host link can
% reliably keep up with.
%
% Capped well below what comm.SymbolSynchronizer's internal, non-tunable
% ~1.1x-per-call MaxOutputExpansionFactor can safely absorb: it checks
% that ratio against THIS call's own input size, so a larger read gives
% an unlocked loop (timing estimate drifting with no signal to track
% yet) far more room to exceed it within a single call before getting
% checked, silently truncating its own output when it does. Smaller
% reads get checked far more often relative to the same underlying
% drift rate, which is why chunkLength=10000 alone never hit this.
%
% Sized to exactly ONE MODCOD-1 normal PLFRAME (33282 symbols x 2 sps).
% Measured frame loss against chunk length at 18 dB, 30 frames:
%
%   one block  0 boundaries   0.0% loss        -
%   66564     29 boundaries   3.3% loss   1.07 MB per TCP message
%   100000    18 boundaries   3.4% loss   1.60 MB
%   200000     8 boundaries   3.6% loss   3.20 MB
%   400000     3 boundaries   4.0% loss   6.40 MB
%
% The loss rate is essentially flat, so the number of boundaries is NOT
% what drives it -- which frees the choice to be made on transport cost
% instead. A 1.07 MB message moves at roughly 280 MB/s over the loopback
% link where a 3.2 MB one manages only ~125 MB/s, because the larger
% message far exceeds the socket buffer and forces writer and reader to
% ping-pong. Smaller reads are also safer for the expansion-factor issue
% described above, not riskier.
%
% Changing this value costs ONE re-lock at startup, because the persistent
% filter and symbol-synchroniser objects release whenever the input length
% changes. That is why it must stay FIXED at runtime -- varying it would
% re-lock on every change -- but a different constant is free.
config.rfAcqReadChunkLength = 2*66564;

%% Uplink: CCSDS Telecommand (Functions/Uplink/, sdr_test/UplinkTx+UplinkRx)
% Carries ACM feedback and ARQ retransmit requests from the receiving side
% back to the transmitter. CCSDS 231.0-B supplies the LDPC coding, the
% randomizer and the CLTU framing.
%
% WHICH HALF IS THE TOOLBOX'S. The transmitter uses ccsdsTCWaveform for
% encoding, randomizing and framing, and adds its own pulse shaping. The
% RECEIVER is entirely ours -- ccsdsTCIdealReceiver is not in the chain at
% all. It is called "ideal" for a reason: it assumes the waveform is already
% frequency-corrected, phase-corrected and aligned to a whole symbol, which
% is precisely the work a real receiver has to do. It also gives no way to
% scale LLRs from a measured noise variance, and no parity flag to gate on.
% Functions/Uplink/ therefore implements all eight stages, using the toolbox
% only to CHECK itself: UplinkRxTest.m verifies every framing constant and
% the parity-check matrix against waveforms ccsdsTCWaveform generates.
%
% FREQUENCY PLAN. Downlink at 2 GHz, uplink at 500 MHz -- 1.5 GHz apart, so
% a transmitter at 2 GHz is rejected by the uplink receiver's analog front
% end before any digital filtering. That is what makes full duplex work at
% all, and it is how real CubeSats are arranged (UHF up, S-band down). Both
% radios cover 500 MHz: the 2920 spans 50 MHz - 2.2 GHz, the 2922
% 400 MHz - 4.4 GHz.
%
% Doppler scales with carrier frequency, so a LEO pass sweeping +-47 kHz at
% 2 GHz sweeps only +-11.7 kHz here.
%
% MODULATION: coherent BPSK, no subcarrier. The subcarrier in PCM/PSK/PM
% exists to keep a residual carrier for a phase-locked loop and to move the
% data away from DC, both of which matter for a deep-space link with a
% pointed dish and metres of cable. For a LEO link this is overhead: it
% spends about half the transmitted power on an unmodulated tone and
% triples the occupied bandwidth for no coding gain.
%
% TWO CONSEQUENCES, both of which land on the receiver rather than here:
%
%   1. There is no residual carrier any more -- measured, DC energy falls
%      from 45% to 0.12%. Frequency acquisition can no longer be one FFT
%      peak; a suppressed-carrier BPSK signal has to be squared first, which
%      puts a tone at TWICE the offset.
%   2. LO leakage at 0 Hz now lands inside the signal band instead of
%      1.5 subcarrier-widths away from it, so DC offset has to be removed
%      rather than simply ignored.
%
% WITH BPSK THE TOOLBOX RETURNS RAW SYMBOLS at one sample per symbol -- it
% drops SymbolRate, SamplesPerSymbol, ModulationIndex and
% SubcarrierFrequency from the config entirely. Pulse shaping and rate
% conversion are therefore ours, which is why the RRC parameters below
% exist and did not before.
% MASTER SWITCH for closing the loop over the air.
%
%   true   S2a hosts the ACM-feedback and retransmit-request servers, and
%          relays whatever arrives on them over the 500 MHz RF uplink. S1a
%          recovers the messages off the air. S2b and S3 are UNCHANGED --
%          they still connect to config.feedbackHost/.retransmitHost and
%          have no idea which way their bytes travel.
%   false  S1a hosts those servers itself and the messages stay on TCP
%          loopback, exactly as before the uplink existed.
%
% Keep this as the first thing to flip when the integrated system
% misbehaves: it isolates "the RF uplink is broken" from "something else
% is broken", without touching any other setting.
config.uplink.useRF = true;

config.uplink.centerFrequency = 500e6;

% Which radio does which end. Matches the eventual integration: S2a (the
% receiving side, on the 2922) transmits the uplink, S1a (on the 2920)
% receives it. Swap these to test the other direction.
config.uplink.txIPAddress = config.usrp.rxIPAddress;   % 2922 transmits the uplink
config.uplink.rxIPAddress = config.usrp.txIPAddress;   % 2920 receives it

% 100 MHz / 500 = 200 ksps, chosen so symbolRate * samplesPerSymbol lands
% exactly on an achievable radio rate. The N210's decimation tops out at
% 512, so this is near the slowest the hardware will run -- which is what we
% want, since noise power grows directly with bandwidth.
config.uplink.inter_decimateFactor = 500;
config.uplink.sampleRate = config.usrp.masterClockRate / config.uplink.inter_decimateFactor;

% SYMBOL RATE. Free to choose now that no subcarrier constrains it, and it
% is the main sensitivity dial: noise power grows with bandwidth, so halving
% the rate is worth 3 dB. Against that, a longer burst gives Doppler more
% time to rotate the phase within one frame. 8 ksym/s puts an LDPC(128,64)
% CLTU at 40 ms, over which the ~190 Hz/s Doppler rate at 500 MHz changes
% the offset by under 8 Hz -- negligible -- while keeping occupied bandwidth
% to about 11 kHz.
config.uplink.symbolRate = 20e3;
config.uplink.samplesPerSymbol = config.uplink.sampleRate / config.uplink.symbolRate;

% Pulse shaping. Needed now, and NOT needed before: PCM/PSK/PM was band
% limited by its own subcarrier, whereas raw BPSK symbols are rectangular
% and would splatter across the band. Root-raised-cosine, matched at the
% receiver, so the cascade is a full raised cosine with no ISI at the
% symbol instants.
config.uplink.rolloffFactor = 0.35;
config.uplink.filterSpanSymbols = 10;

% "LDPC" or "BCH". LDPC(128,64) is rate 1/2 against the BCH codeblock's
% 56-of-64, so it costs about 2.5x the airtime and buys real coding gain --
% the right trade for a control link where a lost message can cost the
% connection. LDPCCodewordLength must be 128 or 512; 256 is not defined.
config.uplink.channelCoding = "LDPC";
config.uplink.ldpcCodewordLength = 128;
% NOTE: under LDPC the CCSDS randomizer is MANDATORY, and MATLAB applies it
% whether or not this flag is set -- confirmed by generating waveforms both
% ways and comparing bit for bit. The flag is only load-bearing for BCH.
config.uplink.hasRandomizer = true;

% Tail sequence at the end of each CLTU: 128 symbols that are deliberately
% not a valid codeword, so a decoder knows the CLTU has ended.
%
% It is 40% of a 320-symbol CLTU and this receiver does not currently need
% it -- one codeblock per CLTU at a known length means the end is implied.
% It is kept anyway because under PLOP-2 the link transmits CONTINUOUSLY, so
% removing it saves nothing: 128 tail symbols would simply become 128 idle
% symbols. Against that zero cost it buys standards conformance, 128 known
% symbols of independent health check (see tailMetric), and the option of
% multi-codeblock CLTUs later without a format change.
config.uplink.hasTailSequence = true;

%% PLOP-2 -- the physical layer operations procedure (CCSDS 231.0-B)
% The uplink is a CONTINUOUS link, not a burst link. The carrier is never
% switched off during a session:
%
%   carrier on -> acquisition sequence -> CLTU -> idle -> CLTU -> idle ...
%
% WHY IT MATTERS. Silence between transmissions gives a receiver nothing to
% hold its loops on, which forces it to re-acquire carrier and timing from
% scratch on every single burst. Continuous transmission means the loops
% lock ONCE, on the acquisition sequence, and then simply track -- which is
% what carrier and timing loops are actually good at.
%
% Acquisition sequence: alternating ones and zeros, sent once at the start
% of a session so the receiver can pull its loops in before any data
% arrives. The standard leaves the length mission-specific; it has to cover
% several loop time constants at the worst SNR the link must work at. At
% costasLoopBandwidthHz = 40 one time constant is about 200 symbols, so 1000
% symbols (125 ms) gives five. TO BE OPTIMISED once there is a receiver to
% measure lock time against -- this is a starting value, not a result.
config.uplink.plop.acquisitionSymbols = 1000;

% Minimum idle between one CLTU and the next, in symbols. 24 = 3 octets.
% This is a floor, not a target: whenever no command is ready the
% transmitter simply keeps emitting idle indefinitely, which is the whole
% point of PLOP-2.
config.uplink.plop.minIdleSymbols = 24;

% How much idle to generate per call when nothing is queued. Purely an
% efficiency knob -- larger means fewer calls into the waveform generator
% for the same airtime. It does NOT delay a queued command: when one is
% waiting, only the idle still owed by minIdleSymbols is emitted.
config.uplink.plop.idleChunkSymbols = 256;

% First bit of the acquisition and idle patterns. CCSDS allows either; both
% give the same alternating sequence, offset by one symbol.
config.uplink.plop.startBit = 0;

% Samples handed to the radio per write. Continuous transmission means the
% radio must never starve, and comm.SDRuTransmitter locks its input length
% on the first call, so this is fixed for the whole run.
%
% SIZED TO S2a's LOOP, not picked for convenience. Once the uplink is
% integrated, S2a owns a continuous TRANSMIT session and a receive session
% with a hard drain deadline, in one loop. That loop is paced by radioRx(),
% which delivers config.chunkLength = 33282 samples of 2 GHz downlink per
% call -- 49.9 ms of airtime, so roughly 20 iterations per second. Pushing
% 10000 samples of 200 ksps uplink per iteration is 50 ms of uplink airtime
% for 49.9 ms of downlink airtime: the two rates match, so neither radio
% starves and neither backs up.
%
% GET THIS WRONG IN EITHER DIRECTION AND SOMETHING BREAKS. Too large and
% radioTx() blocks waiting for the uplink radio to catch up, stalling the
% downlink drain and causing an RX overrun. Too small and the uplink
% underruns, putting a hole in a carrier that is supposed to be continuous.
% Watch BOTH counters in S2a's log and adjust from what they say.
%
% SIZED TO COVER FOUR CLTU + MINIMUM IDLE, so the radio never starves, this
% allow that the uplink is continuously transmitting a coherent amount of data,
% without starving/collapsing the whole S2a loop. The 320-symbol CLTU is the largest one the uplink can carry.
config.uplink.txBlockSamples = (320+config.uplink.plop.minIdleSymbols)*config.uplink.samplesPerSymbol;   % 320 symbols, 50 sps

% Target RMS amplitude of the continuous transmitted stream.
%
% A CONTINUOUS stream cannot be normalised by its peak the way a burst was
% -- there is no end to take the maximum over -- so the level is set by RMS
% instead. Root-raised-cosine shaping gives roughly 4 dB of peak-to-average
% ratio with occasional excursions beyond, so 0.3 RMS puts typical peaks
% near 0.48 and rare ones near 0.75, safely inside the +-1 the radio takes.
config.uplink.txRMS = 0.3;

config.uplink.txGain = 10;   % START LOW; raise in 3 dB steps
config.uplink.rxGain = 30;

% Largest control payload carried. One LDPC(128,64) codeword carries 64
% information bits, so 8 bytes fits in a single codeword; both commands are
% 5 bytes today, leaving 24 bits of headroom inside the same airtime.
config.uplink.maxPayloadBytes = 8;

% Fixed on-air burst length, zero-padded around the shaped CLTU, because
% comm.SDRuTransmitter locks its input length on the first call.
%
% LDPC(128,64) gives a 320-symbol CLTU (64 start + 128 codeword + 128 tail),
% which is 8000 samples at 25 samples/symbol, plus the RRC's filter tails.
% 16000 covers that with a guard interval, and also covers LDPC(512,256)
% at 576 symbols if that is ever selected.
%
% MUST BE A WHOLE NUMBER OF SYMBOLS, and derived rather than written out:
% ccsdsTCIdealReceiver refuses any waveform whose length is not an integer
% multiple of samplesPerSymbol, silently producing a burst that can never be
% decoded rather than an error.
config.uplink.burstSamples = ...
    ceil(16000/config.uplink.samplesPerSymbol) * config.uplink.samplesPerSymbol;   % 16000

%% Uplink receiver (Functions/Uplink/ccsdsUplinkReceive.m)
% Eight stages, each in its own file:
%   1 DC suppression   2 coarse carrier (squaring)   3 RRC matched filter
%   4 start-sequence correlation   5 Costas loop   6 derandomize
%   7 LDPC decode   8 command recovery
config.uplink.rxFrameLength = 8192;
config.uplink.maxDrainReads = 16;

% STAGE 1. Corner frequency of the DC blocker, in Hz. Two constraints:
% far below the 10.8 kHz occupied bandwidth (20 Hz removes ~0.4% of the
% spectrum, nothing), and with a time constant long compared with anything
% in the waveform it could mistake for DC. 20 Hz at 200 ksps is a
% 1592-sample memory, 64 symbols. The second constraint is the one that
% bites: an earlier version of this testbed used a 100-sample memory
% against a return link whose phase was held for 160 samples, so the filter
% tracked and subtracted the signal and produced 100% frame errors. What
% makes an aggressive setting safe here is the randomizer, which guarantees
% the data has no DC content over the filter's memory.
config.uplink.dcCornerHz = 20;

% STAGE 5. Costas loop noise bandwidth and damping. Bandwidth is the usual
% two-sided trade: wide tracks fast changes and admits more noise into the
% phase estimate. It can be narrow here because the loop is started from
% the phase stage 4 already measured, so it never has to acquire -- it only
% has to hold for the 320 symbols of one burst.
%
% WHAT IT IS INSURANCE AGAINST. Measured: with stage 2 working normally the
% loop changes nothing at all, because the residual it leaves is under a
% hertz. Its value shows up when that is not true -- decode rate against
% residual carrier offset, at 8 dB Es/No:
%
%            residual    Bn = 0    Bn = 40    Bn = 150
%                 0 Hz     100%       100%        100%
%                 5 Hz     100%       100%        100%
%                20 Hz       0%       100%        100%
%                50 Hz       0%         0%        100%
%               100 Hz       0%         0%          0%
%
% So 40 Hz buys a 20x margin over the residual stage 2 actually leaves, at
% no measured cost in sensitivity. RAISE IT toward 150 if hardware shows a
% larger residual than simulation does -- Bn = 150 measured no worse at
% 0 dB Es/No either, so the setting is not delicate.
%
% Setting it to 0 makes the loop a pass-through, which is how the table
% above was measured.
config.uplink.costasLoopBandwidthHz = 40;
config.uplink.costasDampingFactor = 1/sqrt(2);

% STAGE 3b. Optional Gardner timing-recovery loop between the matched
% filter and the start-sequence correlation. OFF by default, because on
% this waveform it competes with something already better: the correlation
% in stage 4 searches every sample offset at 25 samples per symbol, so it
% lands within 1/25 of a symbol in one shot, where a loop needs roughly
% 1/(Bn*T) symbols to pull in out of a burst only 320 symbols long -- and
% it spends the guard interval before the burst running on noise, so it
% arrives pointing nowhere in particular.
%
% MEASURED, 16 commands through the full chain and channel:
%
%                        2 dB Es/No    6 dB Es/No
%     timing sync off        16/16         16/16
%     timing sync on         10/16         15/16
%
% So it is not broken -- it tracks, and it decodes -- it is simply beaten by
% the thing it would replace. Turn it on if the burst gets much longer,
% LDPC(512,256) or several codeblocks per CLTU, since only then does a
% sample-clock offset have time to drag the timing across a burst. Two
% free-running USRPs at 2.5 ppm move by 0.0008 of a symbol over 320
% symbols, which is nothing.
%
% Enabling it changes stage 4's input rate to 2 samples per symbol, which
% ccsdsUplinkReceive.m passes through for it.
config.uplink.timingSyncEnabled = false;
config.uplink.gardnerLoopBandwidthHz = 120;
config.uplink.gardnerDampingFactor = 1/sqrt(2);

% STAGE 4 gate. Normalised start-sequence correlation, 0 to 1. A real burst
% tends to 1/sqrt(1 + N0/Es) -- 0.78 at 2 dB Es/No, 0.71 at 0 dB -- while
% the largest noise peak across a window this long sits near 0.31 to 0.39.
% 0.50 sits clear of both. This is the receiver's PRIMARY detection test;
% detectThresholdDB below is only a cost-saving pre-filter in front of it.
config.uplink.asmThreshold = 0.50;

% STAGE 7. Belief-propagation iterations. A clean codeword converges in 1;
% 50 is the point past which a burst that has not converged is not going to.
config.uplink.ldpcMaxIterations = 50;

% Sliding search window and its step. The window only has to CONTAIN a
% whole CLTU -- stage 4 locates the start sequence within it -- so the
% requirement is just window - stride >= the CLTU's span in samples.
%
% An LDPC(128,64) CLTU is 320 symbols, spanning 319*25 + 1 = 7976 samples.
% 12000 and 4000 satisfy that with 24 samples to spare.
%
% THE WINDOW IS SIZED TIGHT ON PURPOSE, and this is not a cost decision.
% Stage 2's detection statistic goes as A^2/N -- signal samples squared,
% over window length -- because the burst has to compete with every noise
% sample in the window it is squared alongside. The old 32000-sample window
% held one 8500-sample burst and 23500 samples of nothing, and threw away
% 4 dB for it. Shrinking the window costs proportionally more windows to
% search, but each FFT is smaller, so the real cost barely moves.
%
% ccsdsUplinkReceive.m checks this containment at run time rather than
% trusting the arithmetic here, since the CLTU length depends on the coding
% and codeword length and this file does not own those.
config.uplink.searchWindowSamples = 320*config.uplink.samplesPerSymbol*1.5; %24000
config.uplink.searchStrideSamples = config.uplink.searchWindowSamples/3; %8000

% STAGE 2 gate, in dB: the strongest peak of the SQUARED signal's spectrum
% relative to that spectrum's median, inside +-2*maxCarrierOffsetHz.
%
% NOT a power-envelope threshold -- that cannot work here. A CLTU is 8000
% samples carrying 64 information bits, so per-sample SNR sits about 21 dB
% BELOW Eb/No and the burst is well under the noise in the time domain,
% invisible to a power detector. Squaring turns the BPSK into a tone at
% twice the carrier offset, and the FFT concentrates that tone while
% spreading the noise.
%
% ITS JOB IS TO SAVE WORK, NOT TO DECIDE. The actual detection decision
% belongs to asmThreshold above, backed by LDPC parity in stage 7. Set this
% LOOSE: a burst rejected here is gone for good, while a noise window that
% slips through costs a fraction of a millisecond and is then thrown out by
% the start-sequence correlation anyway.
%
% MEASURED: pure noise reads 13.2 dB median, 14.9 dB worst; a real burst
% reads 16 dB at 0 dB Es/No, 20 dB at 2 dB, 26 dB at 6 dB, 40 dB at 16 dB.
% At 12 dB this gate therefore passes essentially everything, which is
% deliberate -- it costs almost nothing to leave open. The whole receiver
% measures 9% of real time with it open against 3% with it closed, because
% stage 2 IS most of the cost and runs either way. Raise it to about 16 dB
% if that 6% is ever needed, at the price of losing bursts below ~1 dB
% Es/No.
config.uplink.detectThresholdDB = 12;
% The carrier search looks only this far either side of 0 Hz. Two
% free-running TCXOs at 2.5 ppm give +-2.5 kHz at 500 MHz and a LEO pass
% adds +-11.7 kHz, so 25 kHz covers both while keeping the search away from
% out-of-band spurs. It is also within the +-Rs/4 the estimator can resolve.
config.uplink.maxCarrierOffsetHz = 25000;

% Beacon cadence for the standalone bring-up scripts. Unconditional, so the
% uplink can be proven with the 2 GHz downlink switched off entirely.
config.uplink.beaconPeriodSec = 1.0;

%% Frame-sync detection gate
% Minimum normalised SOF correlation peak (dvbs2FrameSync.m) before a lock
% is accepted. Load-bearing for sensitivity, not a formality: the chain
% decodes correctly from about 1 dB Es/No, but the peak metric does not
% reach 0.8 until roughly 5 dB, so a high gate discards frames that would
% have decoded perfectly.
%
% Measured frames decoded at MODCOD 1, by gate:
%          2 dB   4 dB   6 dB   12 dB
%   0.80      0      2     11      18
%   0.70      0     10     13      18
%   0.60      7     12     13      18   <- maximum at every Es/No
%   0.50      7     12     13      18   (but 57 false locks at 2 dB)
%
% 0.60 WAS CHOSEN ON THAT TABLE AND IT WAS THE WRONG CALL. The reasoning
% was that the errors are asymmetric -- that a false lock is cheap, costing
% one wasted header decode before the loop skips forward, while a missed
% lock costs a whole frame. On hardware the first half of that is simply
% false, and this is the measured evidence:
%
%                        peak          PLHdrConf      PLS         SNR
%   real frames      0.986 - 0.991   2.20 - 2.64   5 (steady)  16 - 17 dB
%   false locks      0.601 - 0.645   0.041 - 0.076  random     0.0 - 0.4 dB
%
% A FALSE LOCK IS NOT CHEAP, because it does not merely fail -- it produces
% a CFO measurement. That garbage measurement (-67 kHz, -149 kHz, +72 kHz on
% consecutive frames) is fed to dvbs2CFOTracker.m, whose rate term then runs
% away: measured going from +31 Hz/s on the first real frame to +129359 Hz/s
% twenty frames later. Once the tracker is poisoned, REAL frames can no
% longer be corrected either, so the false locks end up costing exactly what
% a missed lock costs, and then some. The observed run decoded 8 frames
% perfectly and then lost 18 in a row.
%
% The simulation table is not wrong, it is incomplete: it counted frames
% decoded but not the damage a false lock does to state that persists after
% it. 0.80 separates the two populations above with a wide margin at both
% ends and is what the hardware supports.
%
% A BETTER STATISTIC IS AVAILABLE and not yet used as a gate: PLHdrConf
% (S2b_Reciever.m) separates real from false by a factor of forty, against
% this metric's factor of 1.6. Worth adopting if 0.80 still proves noisy.
config.frameSyncPeakThreshold = 0.7;

%% Carrier frequency offset: acquisition resolution and closed-loop tracking
% Requested FFT resolution for the blind, open-loop CFO estimate S2a runs
% on each chunk (Functions/dvbs2RawCFOCompensate.m). This sets the
% QUANTISATION of that estimate: at 2 Msps and QPSK, the old value of
% 1000 gave a step of 976.5625 Hz, and a true offset sitting between two
% bins made successive chunks dither between them -- visible in the logs
% as rawCFO alternating between exactly -976.6 and -1953.1 Hz, each flip
% shoving a ~977 Hz step into everything downstream. 100 takes the FFT to
% 8192 points and the step to ~61 Hz, for roughly 1.5x the compute of
% that stage -- affordable, since it now runs in S2a which sits at
% roughly 10-16% of real time.
config.rawCFOResolutionHz = 100;

% DISABLED ON THE BENCH. S2a's blind coarse CFO stage
% (dvbs2RawCFOCompensate.m) is switched off; cfoEstHz is reported as 0 and
% the samples pass through un-rotated. S2b needs no change -- it adds the two
% estimates and subtracts S2a's back out, so a zero simply means the
% SOF-based estimator measures the whole offset and the tracker applies all
% of it.
%
% WHY OFF. The stage uses an M-th power blind estimator, and it has two
% problems that only matter once you leave the bench:
%
%   1. IT NEEDS TO KNOW THE MODULATION. S2a hardcodes "QPSK" because the
%      MODCOD lives in the PLHEADER, which is decoded downstream in S2b --
%      a circular dependency S2a cannot resolve. Raising 8PSK to the 4th
%      power leaves BPSK, not a tone; 16APSK collapses under no power at
%      all. It is correct today only because modcodSet is capped at QPSK.
%   2. IT HAS A CLIFF. For QPSK the unambiguous range is +-Fs/(2*4) =
%      +-83 333 Hz, and a noise peak landing near that edge produces a
%      wild estimate. Measured: chunk 233 read -1546 Hz, chunk 234 read
%      +81787 Hz. S2a then de-rotated that chunk by 81.8 kHz, which
%      unlocked S2b's Gardner loop, and S2b never recovered -- 120 frames
%      decoded perfectly, then nothing for the rest of the run. Every bad
%      value in the logs sits within a few kHz of +-83 333.
%
% WHY IT IS SAFE TO REMOVE HERE. The stage exists because an uncorrected
% CFO biases Gardner timing recovery. The bench offset is ~1400 Hz, which
% is 0.42% of the 333 333 sym/s symbol rate -- 0.75 degrees across Gardner's
% half-symbol span, far below its own self-noise. And dvbs2CoarseFreqEst's
% Luise & Reggiannini stage-1 lag of 3 gives an unambiguous range of
% +-Rs/8 = +-41.7 kHz, so it measures 1400 Hz on its own with 30x margin.
%
% WHEN IT MUST COME BACK. A real 2 GHz LEO pass is +-47 kHz of Doppler --
% 14% of the symbol rate, enough to bias Gardner, and just outside the L&R
% range. The replacement then has to be MODULATION-INDEPENDENT: a band-edge
% FLL or a spectral-centroid estimator, both of which use only the RRC
% pulse shape's symmetry and never look at the constellation.
config.rawCFOEnabled = false;

% Closed-loop CFO tracking in S2b (see Functions/dvbs2CFOTracker.m). The
% per-frame SOF estimate is a single-shot measurement from only 26
% symbols, so it is noisy; the true offset is two crystals drifting and
% is far more stable than the measurements of it. Tracking therefore buys
% precision for free -- the same "estimate the distribution, not the
% sample" idea the ACM policy uses.
%
% alpha: fraction of each measurement error applied to the offset
%   estimate. Near 1 follows measurements closely and is jumpy; small is
%   smooth but slow. 0.1 suits a bench link where the offset is static.
% beta: the same for the RATE of change, from the standard
%   beta = alpha^2/(2-alpha) relation, so alpha is the only real dial.
%   On the bench the rate stays near zero and beta does nothing. It
%   matters under Doppler, where a frequency-only tracker would settle at
%   a constant lag behind a sliding offset. RAISE alpha to ~0.3-0.4 for
%   satellite passes so the tracker can follow the slide.
config.cfoTrack.alpha = 0.1;
config.cfoTrack.beta = config.cfoTrack.alpha^2 / (2 - config.cfoTrack.alpha);
% Outlier gate: reject a measurement whose error exceeds gateFactor times
% the running average error, floored at gateFloorHz so a very quiet link
% cannot shrink the gate to nothing. A bad SOF correlation is far more
% likely than a real frequency jump of that size.
config.cfoTrack.gateFactor = 4;
config.cfoTrack.gateFloorHz = 200;
% Measurements accepted unconditionally while the running error average
% is still settling, so the gate cannot lock out the initial convergence.
config.cfoTrack.minUpdatesBeforeGating = 10;
% Consecutive rejections after which the tracker concludes the offset
% genuinely moved (rather than the measurements being bad) and re-acquires
% on the latest one instead of coasting on a stale estimate.
config.cfoTrack.maxConsecRejects = 5;

%% ACM policy parameters (see Functions/dvbs2ACMPolicy.m)
% The MODCOD ladder the policy is allowed to choose from. The full DVB-S2
% set is 1:28 (QPSK 1/4 through 32APSK 9/10); this is deliberately capped
% at 16 (8PSK 8/9) because the higher rungs have unresolved problems on
% this hardware -- 16APSK and 32APSK need an accurate amplitude reference
% the receiver does not yet establish, and dvbs2NonPilotFineFreqPhase.m
% warns that its M-th power blind CFO estimate is not supported above
% ModulationOrder 8, falling back to phase-only pre-correction. Raise this
% once those are addressed; nothing else needs to change.
% CAPPED AT 11 = QPSK ONLY, AND THIS IS A WORKAROUND, NOT A DESIGN CHOICE.
%
% A DVB-S2 normal FECFRAME is a fixed 64800 BITS whatever the MODCOD, so the
% number of SYMBOLS it needs depends only on bits per symbol:
%
%   QPSK   (1-11)  2 bits/sym   32400 symbols
%   8PSK   (12-17) 3 bits/sym   21600 symbols
%   16APSK (18-23) 4 bits/sym   16200 symbols
%   32APSK (24-28) 5 bits/sym   12960 symbols
%
% Moving WITHIN a modulation order costs nothing -- the waveform length is
% unchanged, so comm.SDRuTransmitter keeps its locked input size and the
% radio session stays open. CROSSING one changes the sample count, which
% forces release() and a full UHD re-negotiation: measured at 6.5 SECONDS of
% dead carrier, after which S1a declares the return link lost, drops to
% MODCOD 1, resizes AGAIN, and the run does not recover. Confirmed by the
% length ratio in the log, 266276/177572 = 1.4996 = exactly 3/2, the
% bits-per-symbol ratio between 8PSK and QPSK.
%
% RESOLVED. S1a no longer hands the radio whole PLFRAMEs. Generated samples
% go into a queue and the radio is always given exactly
% config.tx.blockSamples from the front of it, so the MODCOD can change on
% any frame boundary and comm.SDRuTransmitter never sees the frame length at
% all. Frames land wherever they land inside a block, which S2b does not care
% about -- it searches for the SOF and buffers across chunk boundaries, so
% it never assumed frame alignment.
%
% (The earlier note here proposed padding with dummy PLFRAMEs. That does not
% work on its own: a dummy is 3330 symbols and none of the gaps divide by
% it -- (33282-22194)/3330 = 3.33, (33282-13338)/3330 = 5.99 -- so the
% fixed length has to be enforced on SAMPLES, not frames. Dummy frames are
% still worth adding later as cheap filler if waveform generation ever falls
% behind, which is a risk at 16APSK and above; see below.)
%
% 1:28 = the full legacy DVB-S2 ladder: QPSK (1-11), 8PSK (12-17),
% 16APSK (18-23), 32APSK (24-28). 8PSK 2/3 was confirmed working end to end
% on hardware -- S3 recovered 28 of 28 packets per frame at PLS=53 -- so the
% remaining question is not whether the radio copes but whether S1a can
% GENERATE fast enough.
%
% WATCH THE WAVEFORM-GENERATION SHARE IN S1a'S PROFILE. LDPC encoding costs
% the same per frame at any MODCOD (always 64800 bits), but a denser frame
% occupies LESS airtime, so the duty cycle climbs as the ladder is climbed:
%
%   QPSK    99.8 ms airtime/frame   ~22 ms to generate   22%
%   8PSK    66.6 ms                 ~22 ms               33%
%   16APSK  50.1 ms                 ~22 ms               44%
%   32APSK  40.0 ms                 ~22 ms               55%
%
% S1a measured 18.6% at RT 1.198 on the 8PSK run. If the profile shows that
% share approaching ~50% at the top of the ladder, S1a cannot keep the
% transmit FIFO fed and the fix is cheap dummy PLFRAMEs as filler (no LDPC
% encode needed), not a narrower modcodSet.
config.acm.modcodSet = 1:28;

% Largest move along the ladder a single ACM decision may make, in RUNGS
% (positions in modcodSet, not raw MODCOD numbers).
%
% WHY A CAP AT ALL. Every MODCOD change forces S1a to release and reopen the
% radio, because comm.SDRuTransmitter locks its input length and the
% waveform length changes with the MODCOD -- visible in the log as
% "waveform length 266276 -> 106724 samples; releasing radio to resize".
% That is a real gap in the carrier. A link that jumps 1 -> 28 on its first
% decision therefore breaks itself, the receiver reports the resulting
% garbage, and the policy jumps back down: an oscillation whose driver is
% the switching, not the channel.
%
% UPWARD ONLY. Moving DOWN is never capped, and must not be: every moment
% spent on a MODCOD the channel can no longer support costs real frames,
% which is the same asymmetry agreeCountDown and minDwellSecDown already
% encode. A cap here would make a collapsing link take several decisions to
% reach safety.
%
% 3 rungs against agreeCountUp = 3 and minDwellSecUp = 5 s means climbing
% from 1 to 16 takes about five decisions -- slow enough that each step is
% validated by real feedback before the next is attempted.
config.acm.maxJumpRungs = 3;

% PLHEADER PLAUSIBILITY REJECTION (Functions/dvbs2FrameAcceptable.m).
%
% The PLSC is 64 symbols under a Reed-Muller code, decoded by nearest
% neighbour. Under noise it lands on a valid-but-WRONG codeword often
% enough to matter: the decode reports high confidence and hands the chain
% a MODCOD, frame length and pilot layout for a frame nobody sent. Seen on
% hardware as PLS=47, PLS=71 and a ModulationOrder=16 decode on a
% QPSK-only link. Confidence weighting cannot catch these -- the wrong
% codeword was decoded confidently.
%
% What can catch them is that S2b already knows what S1a is permitted to
% transmit. Each flag below asserts one such agreement.
%
% NONE OF THESE DESCRIBE ANYTHING DVB-S2 FORBIDS. They describe what this
% transmitter currently does, and they are separate flags so each can be
% retired on its own as the testbed grows into more of the standard. The
% condition to meet before flipping each one is documented at the check
% itself in dvbs2FrameAcceptable.m.
%
% Removal notes, shortest path first:
%   restrictToModcodSet - needs no edit at all. It widens automatically
%                         when config.acm.modcodSet widens; it only ever
%                         asserts the two ends agree on the same set.
%   requirePilots       - needs the non-pilot receive paths to be
%                         trustworthy first. dvbs2SNREstimate's fallback
%                         still measures against a hardcoded QPSK
%                         constellation, so it is wrong for 8PSK and above.
%   requireNormalFrame  - needs S1a to actually build short FECFRAMEs, and
%                         the short-frame LDPC tables wired up.
config.rxReject.requirePilots      = true;
config.rxReject.requireNormalFrame = true;
config.rxReject.restrictToModcodSet = true;

% SNR CROSS-CHECK (Functions/dvbs2SNREstimate.m).
%
% Every frame's SNR is measured twice, against two independent known-symbol
% references, and this is how far apart they may be before the frame is
% treated as unmeasurable:
%
%   pilots    792 symbols, +-0.15 dB, spread over the whole 99.8 ms frame,
%             positions derived from the decoded PLSC
%   PLHEADER   90 symbols, +-0.44 dB, confined to 270 us, needs no pilot
%             layout, always pi/2-BPSK whatever the frame's modulation
%
% Normal disagreement between two unbiased estimators of this precision is
% sqrt(0.44^2 + 0.19^2) ~ 0.48 dB. The failures actually observed were
% ~17 dB apart -- six consecutive frames reporting about -1 dB on a 16 dB
% link. 3 dB sits with six times margin on both sides of that gap.
%
% When they agree the pilot value is reported, because it is the more
% precise. When they disagree the header value is reported, because a
% +-0.44 dB estimate beats a -17 dB bias. Either way the frame is still
% sent to S1a as a raw sample -- S2b does not smooth, and this is a
% correction of the reference, not of the distribution.
config.snr.maxDisagreementDB = 3;

% FIXED RADIO BLOCK SIZE (S1a_Transmitter.m's transmit FIFO).
%
% comm.SDRuTransmitter locks its input length on the first call; changing it
% needs release(), which tears down the UHD session for ~6.5 s. A DVB-S2
% FECFRAME is a fixed 64800 BITS, so its SYMBOL count shrinks as the
% modulation gets denser -- QPSK 33282, 8PSK 22194, 16APSK 16686,
% 32APSK 13338 -- and every MODCOD change used to force that release. It is
% why acm.modcodSet was capped at QPSK.
%
% Frame-level padding cannot fix it. A dummy PLFRAME is 3330 symbols and
% none of the gaps divide by it: (33282-22194)/3330 = 3.33,
% (33282-16686)/3330 = 4.98, (33282-13338)/3330 = 5.99. A common multiple of
% the four frame lengths runs to minutes of airtime.
%
% So S1a queues generated samples and hands the radio exactly this many from
% the front, every time. Frames fall wherever they fall inside a block --
% which costs nothing, because S2b hunts for the SOF and buffers across chunk
% boundaries rather than assuming frame alignment.
%
% 266256 = 4 QPSK PLFRAMEs at 2 samples/symbol = 399.4 ms of airtime, which
% is exactly what a burst was before this change. Keeping it identical means
% the uplink drain credit, the pacing and the profiling numbers all stay
% comparable with every run recorded so far.
config.tx.blockSamples = 266256;

% SHARE OF EACH BLOCK'S AIRTIME S1a MAY SPEND GENERATING REAL PLFRAMES.
% Whatever it cannot generate inside this budget is filled with dummy
% PLFRAMEs instead (Functions/dvbs2DummyFiller.m).
%
% Why a budget is needed at all: LDPC encoding costs the same per frame at
% any MODCOD (always 64800 bits), but a denser frame occupies LESS airtime,
% so generation duty climbs as the ACM ladder is climbed. Measured at
% 27.7 ms per frame on this machine:
%
%       QPSK     99.8 ms airtime/frame   28%
%       8PSK     66.6 ms                 42%
%       16APSK   50.1 ms                 55%
%       32APSK   40.0 ms                 69%
%
% and the uplink receiver takes a further ~28% of airtime regardless. At the
% top of the ladder the total exceeds 100%, which is not a scheduling
% problem but a genuine shortage of time -- so the fix has to be doing less
% work, not reordering it. On the first full-ladder run this showed up as
% TX underruns rising from 6/126 bursts to 97/274.
%
% WHERE THIS NUMBER COMES FROM, AND WHY IT MOVED. The figure above (0.35)
% was measured back when S1 was one process: non-generation work was 62.9%
% of wall at RT 1.100 -- because that process also owned radioTx() AND
% radioUplinkRx(), which together dominated its time. Splitting the radios
% into S1a (Functions/Testbed's own architecture notes have the full
% history) left THIS process, S1b_ACMControl.m, competing only with its own
% uplink DSP -- not with any radio call at all. Measured after that split:
% uplink DSP 15.0%, block send to S1a 5.8%, other overhead 14.6%, for
% ~35.4% non-generation work -- against 0.35's old 62.9%. The old value was
% capping generation at roughly the level that used to be NECESSARY
% headroom, which is no longer needed: S1b's own profile showed 42.8% of
% its time spent idle ("pacing wait"), and 16.1% dummy filler that
% headroom could have covered instead.
%
% 0.55 leaves meaningful slack rather than claiming the theoretical
% maximum (~65%) outright -- uplink DSP time is bursty (it spikes on every
% decode, not evenly spread), and pushing the budget to consume ALL
% measured idle time would leave no margin for that burstiness before
% generation itself starts missing its own pacing tick (see
% genDeadlineMisses in S1b_ACMControl.m's profile -- already nonzero even
% at 0.35, so some variability is inherent, not new).
%
% TESTED AT 0.55 ON HARDWARE AND REVERTED -- the idle-time argument above
% was correct as far as it went, but incomplete: it treated the pacing
% tick as a soft, forgiving budget, when the S1b->S1a link is actually
% fully synchronous with no lookahead. S1a has no buffer ahead of
% radioTx() any more (unlike the old single-process txFifo, which could
% carry a small cushion); the moment S1b runs one tick long, S1a is
% simply waiting with nothing to send. Measured going from 0.35 to 0.55:
% dummy filler dropped 16.1% -> 5.4% as intended, but generation deadline
% misses rose 7.2% -> 24.8% of blocks, which propagated straight through
% to S1a (TX underruns 2.5% -> 11.0%, RX overruns 0 -> 12) and out to the
% actual link (frame loss 11.8% -> 25.9%, more than doubled). The idle
% time this budget was spending was not wasted -- it was the margin that
% kept ticks landing close to their deadline.
%
% THE LOOKAHEAD BUFFER NOW EXISTS -- S1b_ACMControl.m generates one block
% ahead (pendingBlock/pendingMeta in its main loop) instead of generating
% and sending in the same tick, so an occasional slow tick is absorbed by
% the queued block instead of stalling S1a's radioTx() directly. A
% pending block is dropped rather than sent if the MODCOD/link-established
% state moves on before it ships (blocksDropped in S1b's profile), and S1a
% falls back to a locally-synthesized dummy PLFRAME (localDummyBlocksSent
% in its own profile) on any tick S1b has nothing ready, rather than
% waiting indefinitely.
%
% RETESTED AT 0.45 ON HARDWARE WITH THE PIPELINE, AND KEPT. Four
% configurations measured back to back on the same hardware, MODCOD
% climbing to 32APSK in every one (so all four are under comparably heavy
% load): 0.35 no pipeline (16.1% filler, 7.2% gen-deadline misses, 2.5% TX
% underruns, 11.8% frame loss); 0.55 no pipeline (5.4% filler, 24.8%
% misses, 11.0% underruns, 25.9% loss -- the collapse this whole redesign
% was for); 0.35 WITH pipeline (20.6% filler, 13.4% misses, 12.3%
% underruns, 11.1% loss); 0.45 WITH pipeline (13.2% filler, 25.8% misses,
% 14.0% underruns, 8.0% loss -- the best frame-loss figure of the four,
% despite the worst internal misses/underruns numbers). The pattern holds
% across all four: once a slow tick has the pending-block buffer to land
% in instead of stalling S1a's radio directly, internal timing pressure
% (misses, underruns, filler) stops translating into frame loss the way it
% used to -- the stale-block drop and S1a's local-dummy fallback (both in
% S1b_ACMControl.m / S1a_Transmitter.m) absorb it instead. 0.45 was not an
% exhaustive search of the space above 0.35; it is one deliberate step
% tested to confirm the pipeline actually buys back the headroom this
% comment used to say only a lookahead buffer could.
config.tx.genBudgetFraction = 0.45;
% PLACEHOLDER quasi-error-free (QEF) Es/No operating points in dB, keyed
% by MODCOD index, Normal FECFRAME (the commonly published ETSI EN 302
% 307-1 Table 13 figures). VERIFY against the actual standard before
% relying on these for real ACM decisions -- these are best-effort
% reference values, not a certified source (same caveat as the
% phase-noise masks flagged in configureDVBS2Channel.m).
config.acm.qefEsNodB = containers.Map(num2cell(1:28), { ...
    -2.35, -1.24, -0.30,  1.00,  2.23,  3.10,  4.03,  4.68, ...  % 1-8:   QPSK  1/4 .. 4/5
     5.18,  6.20,  6.42, ...                                     % 9-11:  QPSK  5/6 .. 9/10
     5.50,  6.62,  7.91,  9.35, 10.69, 10.98, ...                % 12-17: 8PSK  3/5 .. 9/10
     8.97, 10.21, 11.03, 11.61, 12.89, 13.13, ...                % 18-23: 16APSK 2/3 .. 9/10
    12.73, 13.64, 14.28, 15.69, 16.05 ...                        % 24-28: 32APSK 3/4 .. 9/10
    });
% Safety margin, expressed in STANDARD DEVIATIONS of the measured SNR
% rather than as a fixed number of dB (see dvbs2SelectMODCOD.m). A
% MODCOD is only selected when mu - k*sigma still clears its QEF
% threshold, so for a roughly normal SNR distribution k sets the
% fraction of time the link sits above that threshold -- i.e. the target
% outage rate (k=1 ~84%, k=2 ~97.7%, k=3 ~99.87%).
%
% TUNE AGAINST MEASURED PER: sigma here is the spread of the SNR
% ESTIMATES, which includes dvbs2SNREstimate.m's own estimator noise
% (sparse pilot blocks give real frame-to-frame variance even on a
% constant channel) and not only genuine channel variation. Because the
% two cannot be separated, a large k penalises you for a noisy estimator
% as much as for a noisy channel. Start at 1 and raise it only if the
% measured packet error rate says you need to.
config.acm.sigmaK = 1.0;
% Relaxed value applied to the MODCOD ALREADY IN USE, so a rung is not
% abandoned the moment sigma nudges its bound below threshold. This is
% what replaces the old asymmetric fixed margin as the anti-flapping
% hysteresis on the selection itself.
config.acm.sigmaKStay = 0.5;
% Minimum protection in dB regardless of sigma -- guards against a quiet
% or very short observation window reporting an unrealistically small
% spread and producing an over-optimistic bound.
config.acm.marginFloorDB = 0.5;
% Length of the rolling SNR observation window, in frames. Too short and
% sigma is itself noisy; too long and the statistics describe channel
% conditions that have already passed. Match it to the channel's
% coherence time once that is known.
config.acm.historyLength = 50;
% (Trend parameters now live with the feedback settings below, expressed in
% SECONDS and BATCHES rather than raw samples, since the policy works from
% batched reports.)
% Reacting to a MODCOD change is asymmetric by design: moving UP (to a
% less robust, more spectrally efficient MODCOD) only ever costs some
% throughput headroom if delayed, so it stays conservative to avoid
% flapping on noisy feedback. Moving DOWN (to a more robust MODCOD)
% costs real frame loss for every extra moment spent on a MODCOD the
% channel can no longer support, so it reacts close to immediately.
config.acm.agreeCountUp = 3;      % consecutive feedback samples that must agree before moving up
config.acm.agreeCountDown = 1;    % a single feedback sample is enough to move down
config.acm.minDwellSecUp = 5;     % minimum time between switches, moving up
config.acm.minDwellSecDown = 0;   % no minimum dwell time moving down
% NOTE: there is deliberately no SNR smoothing parameter any more. The
% receiver reports RAW per-frame samples and the transmitter owns all
% the statistics (dvbs2ACMPolicy.m) -- smoothing at the receiver would
% have destroyed the variance and trend the policy is built on before
% they ever crossed the wire.

%% Link establishment (SDR mode only)
% Number of CONSECUTIVE successfully-decoded calibration frames S2b must
% see before it reports link establishment (SNR + recommended starting
% MODCOD) back to S1a. Each additional frame further EMA-smooths
% snrSmoothedDB (config.acm.snrSmoothingAlpha above), so requiring
% several consecutive clean decodes instead of just the very first one
% gives S1a a materially more stable starting SNR estimate to pick its
% first real MODCOD from, rather than bootstrapping the whole link off a
% single lucky/unlucky sample. Any decode failure (lost lock, PLHEADER
% exception, dummy frame, invalid frame length) resets the count -- only
% an unbroken run of successes counts.
config.calibLockFramesRequired = 5;

%% Feedback: event-driven, not periodic
% The receiver reports when there is something worth reporting, rather than
% on a fixed frame count. A frame count is doubly wrong here: it is tied to
% nothing physical, and it SPEEDS UP as MODCOD rises (frames get shorter),
% so it reported fastest exactly when the channel was best and least
% interesting -- about every 167 ms at MODCOD 21.
%
% HOW OFTEN THE CHANNEL ACTUALLY MOVES. For a 10-minute LEO pass with 16 dB
% of dynamic range, differentiating the pass geometry gives a peak slope of
% about 0.09 dB/s. MODCOD rungs are 0.6-1.2 dB apart, so the channel needs
% 7-13 SECONDS to move by one step. Reporting six times a second was
% roughly fifty times more often than anything could change.
%
% A report is sent when any of these holds:
%   - the mean SNR has moved more than reportDeltaMeanDB since the last one
%   - the standard deviation has moved more than reportDeltaSigmaDB
%   - heartbeatSec has elapsed with nothing to say
%
% SILENCE IS THEREFORE INFORMATION: if the transmitter hears nothing, the
% receiver's statistics are still within those deltas of the last report.
% That only holds because of the heartbeat -- without it, silence would be
% ambiguous between "nothing changed", "the return link failed" and "the
% receiver died". The heartbeat separates them.
% Outlier rejection before a batch is summarised, in median-absolute-
% deviations. The MAD is used rather than the standard deviation because
% sigma is itself inflated by the outlier being looked for.
config.acm.outlierMADs = 4;

config.acm.reportDeltaMeanDB = 1;    % half a MODCOD rung
config.acm.reportDeltaSigmaDB = 1.5;
config.acm.heartbeatSec = 3.0;

% Minimum frames in a batch before a CHANGE trigger may fire. The heartbeat
% is exempt -- it must prove liveness on schedule regardless.
%
% WHY: dvbs2SNREstimate returns a badly wrong value on roughly 4% of
% otherwise perfect frames. Measured on hardware: peak 0.988, PLHdrConf
% 2.5, correct PLS, and SNR = -3.14 dB where the true value was 16.5 dB.
% A single such frame trips "the mean moved", reports itself instantly, and
% arrives at the policy as a real observation -- "mean 6.50 sigma 9.75"
% against a genuine spread of 0.3-0.5 dB. Because the MODCOD bound is
% mu - k*sigma, one bad estimate holds the entire ladder down.
%
% outlierMADs above is supposed to catch exactly this, but a batch of one
% has no median absolute deviation to measure against. Three frames is the
% smallest batch where a MAD means anything.
config.acm.minFramesForChangeReport = 5;

% The transmitter declares the return link lost after this long with no
% message of any kind (feedback OR an ARQ request -- both prove liveness).
% Set to tolerate one lost heartbeat: 3 s heartbeat, 5 s timeout.
%
% A lost change-triggered report is the one real weakness of event-driven
% reporting -- the transmitter cannot know it missed anything. The heartbeat
% bounds the damage.
config.acm.linkLossSec = 5.0;

% Rolling window of feedback reports the policy combines, and the minimum
% number of them before a trend is trusted. Ten reports at a 3 s heartbeat
% spans about 30 s, comfortably longer than the 7-13 s a MODCOD step takes.
config.acm.batchWindow = 10;
config.acm.trendMinBatches = 4;
% Seconds ahead the fitted slope is extrapolated when it is falling.
config.acm.trendHorizonSec = 3.0;

%% Retransmission (selective-repeat ARQ -- see dvbs2GeneratePacketBurst.m
%% and the retransmit queue / gap-detection logic in the transmitter and
%% processing-unit scripts)
% Sanity cap on a single retransmit request's index range: a packet whose
% own CRC-8 fails could have a corrupted 32-bit index embedded in it,
% which would otherwise produce a wild, implausible gap size. Requests
% larger than this are logged and dropped rather than acted on.
config.retransmit.maxRequestRange = 5000;
% How many PROCESSED frames to wait between re-requesting whatever
% packet indices are still outstanding (a frame-count-paced retry, not a
% wall-clock timer) -- this is what makes retransmission self-healing if
% a retransmit burst itself gets lost, since a single request is
% otherwise one-shot.
config.retransmit.recheckEveryFrames = 20;

%% Run limits
% Finite by default, but this ONLY bounds how much NEW DATA the
% transmitter generates -- once it has sent this many frames' worth of
% bursts, it stops producing new content but keeps the process alive,
% still servicing any selective-repeat ARQ retransmit request that
% arrives afterward. A hard stop there would strand any retransmit
% request sent near the tail of a run, with no transmitter left to
% answer it.
%
% The receiver and processing unit are NOT gated on maxFrames at all --
% a frame count on their side would conflate original and retransmitted
% frames and could stop them before a legitimate resend arrives.
% Instead they simply keep relaying/processing for as long as their
% upstream neighbor is alive, and stop via disconnect detection
% (the simulated-channel link closing in sim mode cascades downstream
% via dvbs2TCPFrameRead.m's ConnectionClosed error) once the transmitter
% is manually stopped. On real SDR hardware there is no "disconnect" to
% detect (a live radio just keeps producing samples), so all three
% scripts run until manually stopped (Ctrl+C) regardless of this
% setting -- which is the expected shape for a continuous real-time
% receiver anyway. Set to Inf to skip the new-data budget entirely.
% Inf while the ACM loop is being brought up on hardware. With a finite
% budget the transmitter stops producing new content partway through a run
% -- the log shows "new-data budget (50 frames) reached" -- after which the
% carrier goes quiet, RSSI drops to the noise floor, and every remaining
% chunk the receiver examines contains nothing but noise. That makes the
% run's second half useless for judging the receiver, and it manufactures
% exactly the conditions that produce false frame locks.
config.maxFrames = Inf;

%% Run duration and profiling
% Wall-clock limit on a run, in seconds, measured from the START OF THE MAIN
% LOOP -- radio and server setup is not counted, so this is comparable
% between runs however long the USRPs take to open. Inf runs until Ctrl+C.
%
% Set to 60 for a PROFILING RUN: all four scripts stop together and S1a prints
% a breakdown of where its wall time actually went. That breakdown is the
% only way to separate three explanations for a TX underrun that look
% identical from the outside:
%
%   a blocking radio READ      -> "uplink radio reads" is large, and the
%                                 per-read average approaches one frame's
%                                 airtime (41 ms at 8192 samples, 200 ksps)
%   expensive uplink DSP       -> "uplink DSP" is large
%   the transmitter being
%   correctly paced by the
%   radio                      -> "radioTx" is large and everything else is
%                                 small, which is HEALTHY, not a fault
%
% Without the split, all three produce the same symptom.
% 120 s rather than 60. Climbing the full 1:28 ladder takes real time --
% agreeCountUp = 3 reports plus minDwellSecUp = 5 s per rung, and
% maxJumpRungs = 3 caps each step -- so a 60 s run cannot reach the top of
% the ladder before it ends. Two minutes gives the ACM loop room to walk up
% through 8PSK and into the APSK rungs and settle there.
%
% NOTE ON READING THE PROFILES: S1a should be started first (it has the most
% to bring up: two UHD sessions before a single sample leaves), so it also
% FINISHES first. Once it stops, S2a's RSSI drops to about -52 dB -- that is
% the USRP's LO leakage with nothing being fed to it, not a fault. Expect a
% tail of "DECODED NOTHING" heartbeats at the end of S2b's log equal to
% however long S1a was started ahead of the receivers.
config.runDurationSec = 420;

end




