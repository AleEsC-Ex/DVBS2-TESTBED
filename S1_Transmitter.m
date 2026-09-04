%S1_TRANSMITTER DVB-S2 transmitter with ACM (adaptive coding & modulation).
%
%   Generates a DVB-S2 waveform and transmits it continuously in small
%   bursts, either over a real NI USRP-2920 (config.useSDR = true) or,
%   for bring-up/validation, over a local TCP link carrying
%   simulated-channel-impaired samples to the receiver in place of real
%   hardware (config.useSDR = false; see Functions/dvbs2TestbedConfig.m).
%
%   Between bursts, this script polls (non-blocking) for control messages
%   from the receiver -- batched SNR statistics and selective-repeat ARQ
%   requests -- runs the former through Functions/dvbs2ACMPolicy.m (which
%   applies hysteresis and a minimum dwell time so MODCOD doesn't flap on
%   noisy/late feedback), and reconfigures cfgDVBS2 for the next burst
%   whenever the policy recommends a switch.
%
%   Run this alongside the receiver and processing-unit scripts, each as
%   its own MATLAB instance, IN ANY ORDER -- every TCP client connection
%   in this testbed (dvbs2TCPConnectRetry.m) retries until its server is
%   up, and every server (tcpserver) starts listening immediately
%   without needing a client yet, so there's no required startup
%   sequence between the three scripts.

clear; clc;

% Functions/ is organized into subfolders by purpose (core DSP/PHY
% functions stay directly under Functions/; TCP/, Serialization/, and
% Testbed/ hold this testbed's supporting code) -- genpath adds all of
% them recursively, computed from this script's own location so it
% works regardless of MATLAB's current folder when this is run.
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Functions')));

config = dvbs2TestbedConfig();

%% DVB-S2 waveform generator configuration
cfgDVBS2 = dvbs2WaveformGenerator();
cfgDVBS2.StreamFormat = config.dvbs2.StreamFormat;
cfgDVBS2.FECFrame = config.dvbs2.FECFrame;
cfgDVBS2.MODCOD = config.dvbs2.MODCOD;
cfgDVBS2.DFL = getDFL(cfgDVBS2.MODCOD, cfgDVBS2.FECFrame);
cfgDVBS2.HasPilots = config.dvbs2.HasPilots;
cfgDVBS2.SamplesPerSymbol = config.dvbs2.SamplesPerSymbol;
cfgDVBS2.RolloffFactor = config.dvbs2.RolloffFactor;
cfgDVBS2.UPL = config.dvbs2.UPL;

% Sample rate this waveform is produced at, used only by sim mode's
% real-time pacing at the bottom of the transmit loop. Consistent with
% how config.chanBW is derived: Fsym = chanBW/(1+RolloffFactor) =
% usrp.sampleRate/SamplesPerSymbol, so Fsym*SamplesPerSymbol is just
% usrp.sampleRate.
Fsamp = config.usrp.sampleRate;

currentMODCOD = cfgDVBS2.MODCOD;

% In SDR mode, real hardware has no synchronization between S1 and S2 --
% unlike sim mode's TCP link (natural connect-ordering/backpressure),
% S2 isn't guaranteed to be listening yet when this script starts
% transmitting. Until the first ACM feedback ever arrives, transmit
% calibration bursts at the most robust MODCOD instead of real data, so
% S2 has something to lock onto regardless of when it comes online, and
% commit to real data only once that feedback proves the link works
% (see the calibration branch in the main loop below, and the matching
% deferred-forwarding logic in S2_Reciever.m).
linkEstablished = ~config.useSDR;
calibMODCOD = 1;
if config.useSDR
    cfgDVBS2.MODCOD = calibMODCOD;
    cfgDVBS2.DFL = getDFL(calibMODCOD, cfgDVBS2.FECFrame);
    currentMODCOD = calibMODCOD;
end

%% TX dispatch setup
if config.useSDR
    % comm.SDRuTransmitter requires an INTEGER InterpolationFactor -- the
    % USRP's digital upsampling stage only works in discrete integer
    % steps, and chanBW/RolloffFactor/SamplesPerSymbol rarely divide
    % MasterClockRate exactly. Round to the nearest achievable factor
    % and recompute Fsamp to the rate the hardware will actually run
    % at, so it accurately reflects reality rather than the
    % originally-intended, slightly different rate.

    radioTx = comm.SDRuTransmitter( ...
        'Platform', config.usrp.platform, ...
        'IPAddress', config.usrp.txIPAddress, ...
        'MasterClockRate', config.usrp.masterClockRate, ...
        'InterpolationFactor', config.usrp.inter_decimateFactor, ...
        'CenterFrequency', config.usrp.centerFrequency, ...
        'Gain', config.usrp.txGain);
    fprintf('S1: transmitting via USRP at %s (CenterFrequency=%.3f MHz)\n', ...
        config.usrp.txIPAddress, config.usrp.centerFrequency/1e6);
    cleanupTx = onCleanup(@() release(radioTx));
else
    fprintf('S1: connecting to S2a''s simulated-channel server at %s:%d ...\n', ...
        config.simChannelHost, config.simChannelPort);
    simChannelClient = dvbs2TCPConnectRetry( ...
        config.simChannelHost, config.simChannelPort, 'S2a''s simulated-channel server');
    fprintf('S1: connected.\n');
    % tcpclient/tcpserver objects close their connection when their
    % variable is cleared or goes out of scope -- unlike comm System
    % objects (radioTx above), they don't have a release()/delete() call
    % to hook via onCleanup, so no explicit cleanup object is set up
    % here; ending the script (or `clear simChannelClient`) is
    % sufficient.
end

%% Return link: RF uplink, or TCP loopback
% config.uplink.useRF decides where the ACM feedback and ARQ retransmit
% requests come from. Either way they arrive as the SAME 5 serialized bytes
% -- S2 and S3 produce them with dvbs2SerializeFeedback and
% dvbs2SerializeRetransmitRequest regardless -- so everything downstream of
% this point is identical and the deserializers below are unchanged.
%
%   RF    S2a hosts the two servers and relays what arrives over 500 MHz;
%         this process recovers the messages off the air.
%   TCP   this process hosts the servers itself, as it always did.
uplinkRF = config.useSDR && config.uplink.useRF;
uplinkFbQueue = {};
uplinkRtQueue = {};
uplinkOverruns = 0;
uplinkDecodes = 0;
uplinkPeakDB = -Inf;
uplinkEsNodB = NaN;
% Samples the uplink radio has produced but this process has not collected
% yet, carried between iterations so the drain below never over- or
% under-reads. See the drain in the main loop for why this exists.
uplinkDebtSamples = 0;
% Airtime of the burst transmitted on the PREVIOUS iteration, which is what
% the drain credits itself from. Zero on the first pass -- there is no
% previous burst, so nothing is read and the radio's buffer simply starts
% filling. It self-corrects on the second iteration.
lastBurstSec = 0;

if uplinkRF
    % ccsdsUplinkReceive and its DC blocker both hold state across calls.
    clear ccsdsUplinkReceive ccsdsUplinkDCSuppress;

    u = config.uplink;
    radioUplinkRx = comm.SDRuReceiver( ...
        'Platform', config.usrp.platform, ...
        'IPAddress', u.rxIPAddress, ...
        'MasterClockRate', config.usrp.masterClockRate, ...
        'DecimationFactor', u.inter_decimateFactor, ...
        'CenterFrequency', u.centerFrequency, ...
        'Gain', u.rxGain, ...
        'OutputDataType', 'double', ...
        'SamplesPerFrame', u.rxFrameLength);
    cleanupUplinkRx = onCleanup(@() release(radioUplinkRx));
    fprintf('S1: return link over RF -- receiving on %s at %.3f MHz, gain %g dB\n', ...
        u.rxIPAddress, u.centerFrequency/1e6, u.rxGain);
else
    fprintf('S1: opening ACM feedback listener on port %d ...\n', config.feedbackPort);
    feedbackServer = dvbs2TCPServerRetry(config.feedbackHost, config.feedbackPort, "S1's feedback server");
end
acmState = [];
% One-shot latch so the "S2 has connected" event is logged exactly once
% rather than every loop iteration. Without this there is no way to tell
% from S1's console whether a long calibration phase means "S2 is
% connected but hasn't decoded anything yet" or "S2 never connected at
% all" -- two very different problems that otherwise look identical.
% Over RF there is no connection to wait for -- the receiving side either
% transmits or it does not -- so the latch starts closed and the message
% never fires.
feedbackClientLogged = uplinkRF;
% Return-link liveness clock: reset by ANY message from the receiving side,
% feedback or retransmit request. See the loss check in the main loop.
lastReturnTic = tic;
returnLinkLost = false;

%% Retransmit-request listener (this transmitter is the TCP server; the processing unit connects as client)
% Selective-repeat ARQ: a request arrives whenever the receiving side
% detects a gap in the packet sequence or a packet's own CRC-8 fails.
% Requests are queued (FIFO) and serviced one per burst boundary, ahead
% of the next normal sequential burst -- see the main loop below.
if ~uplinkRF
    fprintf('S1: opening retransmit-request listener on port %d ...\n', config.retransmitPort);
    retransmitServer = dvbs2TCPServerRetry(config.retransmitHost, config.retransmitPort, "S1's retransmit-request server");
end
retransmitQueue = struct('StartIdx', {}, 'EndIdx', {});

%% Data source setup
syncByte = de2bi(hex2dec('47'), 8, 'left-msb')';   % MPEG-TS sync byte (0x47), prepended to every packet

% globalPktIdx is a running count of every packet ever transmitted this
% run, independent of burst/MODCOD boundaries. Each packet's payload
% embeds its own globalPktIdx (Functions/dvbs2ReferencePacketPayload.m)
% so the receiving side can independently regenerate the matching
% reference for bit-for-bit BER comparison from the index alone --
% deliberately NOT relying on replaying a single long RNG draw sequence
% in lockstep across processes, which would break under frame loss,
% reordering, or MinNumPackets changing with MODCOD between bursts.
globalPktIdx = 0;

frameSeqNum = 0;
burstNum = 0;
calibBurstNum = 0;

% TRANSMIT FIFO -- how this script sends a variable-length waveform through
% a fixed-length radio.
%
% comm.SDRuTransmitter locks its input size on the first call and the only
% way to change it is release(), which tears down the UHD session for about
% 6.5 s. A DVB-S2 FECFRAME is a fixed 64800 BITS, so its SYMBOL count falls
% as the modulation gets denser:
%
%       QPSK    33282 sym    66564 samples    1.000x
%       8PSK    22194 sym    44388 samples    0.667x
%       16APSK  16686 sym    33372 samples    0.501x
%       32APSK  13338 sym    26676 samples    0.401x
%
% so every MODCOD change used to force a release, and modcodSet was capped
% at QPSK to avoid it.
%
% There is no padding scheme that fixes this at the FRAME level: a dummy
% PLFRAME is 3330 symbols, and (33282-22194)/3330 = 3.33,
% (33282-13338)/3330 = 5.99 -- never an integer, for any pair. A common
% multiple of the four frame lengths runs to minutes of airtime.
%
% So the fixed length is enforced one level down, on SAMPLES rather than
% frames. Generated frames go into this queue and the radio is always handed
% exactly txBlockSamples from the front of it. Frames land wherever they
% land inside a block, which costs nothing: S2 searches for the SOF and
% buffers across chunk boundaries, so it never assumed frame alignment in
% the first place.
%
% NOTE THE MISSING flushFilter. The generator's RRC transmit filter holds
% FilterSpanInSymbols*SamplesPerSymbol = 20 samples of memory, and
% flushFilter drains it. That is right for an ISOLATED burst, whose final
% symbols would otherwise be truncated mid-pulse -- but wrong here.
% Measured: a flushed join drops to |x| = 0.000 for ~20 samples, while an
% unflushed one runs straight through at full amplitude, because the filter
% state carries into the next call exactly as a continuous-mode modulator
% behaves. Dropping the flush also makes each call return precisely
% frameLength*sps samples, which is what keeps the arithmetic below exact.
txBlockSamples = config.tx.blockSamples;
txFifo = complex(zeros(0,1));

% Run totals for the closing profile. Over a 2-minute run the per-burst
% lines scroll away, so the summary has to say by itself how far up the ACM
% ladder this got and whether the FIFO stayed bounded.
modcodHistogram = zeros(1, 28);
maxFifoCarry = 0;
dummySamplesSent = 0;

% Measured cost of generating one PLFRAME, tracked as a running average and
% used to decide how many real frames fit in this block's generation budget.
% Seeded optimistically at the QPSK figure; it converges within a few bursts
% and then follows the MODCOD up the ladder on its own, which is what makes
% the filler self-regulating rather than something to tune per modulation.
genSecPerFrame = 0.025;
genBudgetSec = (txBlockSamples / Fsamp) * config.tx.genBudgetFraction;

% Pre-warm the dummy-filler cache. Building and filtering a block's worth of
% dummy PLFRAMEs takes ~1.5 s the first time and ~1.5 ms every time after,
% so it is done HERE rather than on the first block that needs filler --
% which would be a 1.5 s stall mid-run, causing exactly the underrun burst
% the filler exists to prevent.
dvbs2DummyFiller(txBlockSamples, cfgDVBS2.SamplesPerSymbol, cfgDVBS2.RolloffFactor, 10);

fprintf('\nS1: starting transmit loop (initial MODCOD %d, %d-sample radio blocks) ...\n', ...
    currentMODCOD, txBlockSamples);

% config.maxFrames only bounds NEW-DATA transmission: once frameSeqNum
% reaches it, this loop keeps running rather than exiting, so any
% retransmit request arriving afterward (e.g. for a gap discovered near
% the tail of the run) still gets serviced -- an outright stop here
% would strand those requests with no transmitter left to answer them.
% Stop this script manually (Ctrl+C) once no more retransmit requests
% are arriving.
newDataBudgetLogged = false;

% Real-time accounting: airtimeSec is how many seconds of RF this process
% has generated, wall-clock is how long that took. In sim mode the pacing
% at the bottom of the loop holds this near 1.0 by design, so a value
% ABOVE 1.0 means something downstream is applying backpressure rather
% than that generation itself is too slow.
runTic = tic;
airtimeSec = 0;

% Where the wall time actually goes. Every entry is seconds accumulated
% inside one specific call, so the breakdown at the end of the run
% distinguishes a blocking radio read from expensive DSP from a transmitter
% being correctly paced by its radio -- three causes that produce identical
% "TX underrun" warnings and cannot be told apart any other way.
prof = struct('uplinkRadio', 0, 'uplinkDSP', 0, 'waveformGen', 0, ...
    'radioTxTime', 0, 'reads', 0, 'emptyReads', 0, 'iters', 0, ...
    'underruns', 0);

while true

    %% Stop after config.runDurationSec and report where the time went
    if toc(runTic) >= config.runDurationSec
        wall = toc(runTic);
        fprintf('\n=== S1 PROFILE === %.1f s wall | %.1f s airtime | RT factor %.3f | %d iterations\n', ...
            wall, airtimeSec, wall/max(airtimeSec,eps), prof.iters);
        accounted = prof.uplinkRadio + prof.uplinkDSP + prof.waveformGen + prof.radioTxTime;
        fprintf('  uplink radio reads   %7.2f s  %5.1f%%   %d reads (%d empty), %.1f ms each\n', ...
            prof.uplinkRadio, 100*prof.uplinkRadio/wall, prof.reads, prof.emptyReads, ...
            1e3*prof.uplinkRadio/max(prof.reads,1));
        fprintf('  uplink DSP           %7.2f s  %5.1f%%\n', ...
            prof.uplinkDSP, 100*prof.uplinkDSP/wall);
        fprintf('  waveform generation  %7.2f s  %5.1f%%\n', ...
            prof.waveformGen, 100*prof.waveformGen/wall);
        fprintf('  radioTx              %7.2f s  %5.1f%%\n', ...
            prof.radioTxTime, 100*prof.radioTxTime/wall);
        fprintf('  everything else      %7.2f s  %5.1f%%\n', ...
            wall - accounted, 100*(wall - accounted)/wall);

        % WHICH MODCODs WERE ACTUALLY TRANSMITTED, and how many frames at
        % each. With the transmit FIFO in place the radio never sees a frame
        % length, so this is the line that shows how far up the ladder the
        % ACM loop actually climbed -- and, paired with the
        % waveform-generation percentage above, whether S1 could keep up once
        % it got there. Denser frames occupy less airtime for the same LDPC
        % encode cost, so generation load rises with MODCOD.
        seenMC = find(modcodHistogram > 0);
        if ~isempty(seenMC)
            fprintf('  MODCODs transmitted:');
            for m = seenMC
                fprintf('  %d(%s)x%d', m, localModName(m), modcodHistogram(m));
            end
            fprintf('\n');
        end
        fprintf('  tx FIFO carry-over: max %d samples (< one PLFRAME) | radio block %d, never resized\n', ...
            maxFifoCarry, txBlockSamples);
        % How much of the carrier was filler rather than data. This is the
        % throughput cost of keeping the link up at the top of the ladder --
        % if it is large, either genBudgetFraction is too tight or the ACM is
        % climbing higher than this machine can generate for.
        fprintf('  dummy filler: %.1f%% of transmitted airtime | generation %.1f ms/frame measured\n', ...
            100*dummySamplesSent/max(dummySamplesSent + prof.iters*txBlockSamples, 1), ...
            1e3*genSecPerFrame);
        fprintf('  --\n');
        fprintf('  TX underruns %d of %d bursts | uplink: %d decoded, %d overruns, peak %.1f dB\n', ...
            prof.underruns, prof.iters, uplinkDecodes, uplinkOverruns, uplinkPeakDB);
        fprintf('  per iteration: %.1f ms wall, %.1f ms of airtime produced\n\n', ...
            1e3*wall/max(prof.iters,1), 1e3*airtimeSec/max(prof.iters,1));
        break;
    end
    prof.iters = prof.iters + 1;

    % Yield to MATLAB's event queue before polling the servers below.
    % tcpserver accepts its client ASYNCHRONOUSLY, and that accept is only
    % processed when the event queue is serviced. In SDR mode this loop
    % otherwise never yields -- radioTx() blocks inside a MEX call and
    % fprintf does not service the queue -- so a pending accept is never
    % processed and feedbackServer.Connected stays false indefinitely,
    % even though the operating system has long since completed the TCP
    % handshake (netstat shows the connection ESTABLISHED between the
    % right two PIDs, and the receiver's writes land in the socket
    % buffer unread).
    %
    % Sim mode never exposed this: there linkEstablished starts true, so
    % the calibration branch below is skipped and the real-time pacing
    % pause() at the bottom of the loop services the queue every burst.
    % 'limitrate' caps this at roughly 20 Hz, which is far more than
    % enough to notice a one-time connection without adding measurable
    % cost to the transmit path.
    drawnow limitrate;

    %% Return-link liveness.
    % Feedback is EVENT DRIVEN, so silence normally means "the receiver's
    % statistics have not moved" -- genuine information, not a fault. What
    % makes that readable is the heartbeat: the receiver reports at least
    % every config.acm.heartbeatSec even with nothing to say, so silence
    % beyond config.acm.linkLossSec can only mean the return link is down.
    %
    % On loss the transmitter drops to the most robust MODCOD. It cannot
    % tell whether the forward link is also degrading, and with no feedback
    % arriving it has no way to find out, so the conservative choice is the
    % only defensible one. Real traffic keeps flowing -- reverting to
    % calibration bursts would throw away payload for no gain.
    if linkEstablished
        sinceReturn = toc(lastReturnTic);
        if sinceReturn > config.acm.linkLossSec && ~returnLinkLost
            returnLinkLost = true;
            fprintf(2, 'S1: no return message for %.1f s -- return link presumed LOST; dropping to MODCOD %d.\n', ...
                sinceReturn, calibMODCOD);
            if currentMODCOD ~= calibMODCOD
                release(cfgDVBS2);
                cfgDVBS2.MODCOD = calibMODCOD;
                cfgDVBS2.DFL = getDFL(calibMODCOD, cfgDVBS2.FECFrame);
                currentMODCOD = calibMODCOD;
            end
        end
    end

    %% Drain the return link, whichever one it is
    % Both paths end with fbBytes and rtBytes holding raw serialized message
    % bytes, or empty. Everything after this point is source-agnostic.
    if uplinkRF
        % DRAIN BY ELAPSED TIME, NOT BY A FIXED COUNT.
        %
        % The uplink radio produces samples at a fixed 200 000/s whether
        % anyone collects them or not, so how many are waiting depends on ONE
        % thing: how long since the last visit. Asking for a fixed number
        % ignores that, and asking for too many is far worse than it sounds,
        % because comm.SDRuReceiver WAITS for a frame it does not have yet.
        %
        % MEASURED, with the old fixed 16 reads:
        %
        %   asked    16 x 8192 = 131 072 samples = 655 ms of uplink airtime
        %   existed  one burst  =  79 880 samples = 399 ms
        %   waiting  256 ms, every single iteration
        %
        % Those 256 ms are spent NOT calling radioTx(), while the 2 GHz
        % transmit buffer keeps emptying into the air at 666 667 samples/s.
        % It runs dry: "TX underrun on burst N" for every N, an RT factor of
        % 1.66 against the predicted 655/399 = 1.64, a downlink carrier ~39%
        % dead, and downstream of that S2's Gardner loop random-walking
        % through the holes until it truncates every chunk. One 256 ms wait,
        % three broken log files.
        %
        % Two radios in one loop is what makes this possible at all --
        % over-draining a RECEIVE buffer starves a TRANSMIT buffer that has
        % nothing to do with it. Neither hop showed it when tested alone.
        %
        % CREDITED FROM DOWNLINK AIRTIME, NOT FROM WALL-CLOCK TIME, and that
        % distinction is the whole fix.
        %
        % The obvious version measures elapsed time with tic/toc and converts
        % it to samples. It does not work, because elapsed time INCLUDES the
        % time spent waiting for uplink samples -- the measurement contains
        % the thing it is measuring:
        %
        %   wait 470 ms -> dtSec 640 ms -> "15.6 frames arrived" -> drain 15
        %               -> wait 490 ms -> dtSec 660 ms -> ...
        %
        % Measured with that version: 14.4 reads per iteration where only
        % 9.75 arrive, 73.3% of S1's wall time inside radioUplinkRx(), and
        % 93 of 94 bursts underrunning.
        %
        % lastBurstSec cannot be inflated that way. It is the airtime of the
        % waveform just handed to the radio, fixed by the waveform length and
        % the sample rate -- if this process runs slowly the burst is still
        % 399 ms. And it is the RIGHT quantity, because both radios share one
        % wall clock: 399 ms of downlink airtime and 399 ms of uplink airtime
        % occupy the same 399 ms of real time. Self-limiting by construction:
        % however slow the loop gets, one iteration can only ever credit one
        % burst.
        %
        % WHY THE LOOP THEN FITS. The transmit and the waveform generation
        % below take ~151 ms during which nothing is read, and the radio
        % quietly buffers 151 ms of uplink meanwhile. So of the 399 ms that
        % must be consumed, 151 ms is already waiting (those reads cost only
        % the ~3 ms call overhead) and just 248 ms has to be blocked for:
        % 248 + 151 = 399 ms, exactly real time.
        %
        % THE DEBT CARRIES because 399 ms of uplink is 9.75 reads, never a
        % whole number. Flooring alone would collect 9 and leave 0.75 behind
        % every time -- 6272 samples per iteration accumulating until the
        % radio's own buffer overflows. Keeping the remainder makes the
        % sequence 9, 10, 10, 10, 9 ... which averages exactly 9.75. Per
        % iteration it is still a floor, so it can never ask for samples that
        % do not exist; across iterations there is no drift.
        %
        % The cap bounds the backlog to one drain cycle. If this process does
        % fall behind, the uplink buffer overflows and control messages are
        % lost -- which the 2 s heartbeat re-sends. That is deliberately the
        % cheaper failure: starving the downlink instead costs the carrier,
        % which unlocks S2's timing loop and took 95% of its frames.
        uplinkDebtSamples = min( ...
            uplinkDebtSamples + lastBurstSec * config.uplink.sampleRate, ...
            config.uplink.maxDrainReads * config.uplink.rxFrameLength);
        nUplinkReads = floor(uplinkDebtSamples / config.uplink.rxFrameLength);
        uplinkDebtSamples = uplinkDebtSamples - nUplinkReads * config.uplink.rxFrameLength;

        for k = 1:nUplinkReads
            tRead = tic;
            [uRx, uLen, uOvr] = radioUplinkRx();
            prof.uplinkRadio = prof.uplinkRadio + toc(tRead);
            prof.reads = prof.reads + 1;
            if uOvr
                uplinkOverruns = uplinkOverruns + 1;
            end
            if uLen == 0
                % An empty return means the object does NOT block waiting for
                % a frame it hasn't got. Counting these is what tells us
                % which behaviour we are dealing with.
                prof.emptyReads = prof.emptyReads + 1;
                break;
            end
            tDSP = tic;
            [~, uInfo] = ccsdsUplinkReceive(uRx(1:uLen), config);
            prof.uplinkDSP = prof.uplinkDSP + toc(tDSP);
            uplinkDecodes = uplinkDecodes + uInfo.decodes;
            uplinkPeakDB = max(uplinkPeakDB, uInfo.bestCarrierDB);
            if isfinite(uInfo.esNodB), uplinkEsNodB = uInfo.esNodB; end

            % One uplink, two message types -- routed on the 3-bit type
            % field that heads every control message, which is exactly what
            % it is there for.
            for p = 1:numel(uInfo.payloads)
                payload = uInfo.payloads{p};
                switch dvbs2MessageType(payload)
                    case 0, uplinkFbQueue{end+1} = payload; %#ok<SAGROW>
                    case 1, uplinkRtQueue{end+1} = payload; %#ok<SAGROW>
                end
            end
        end

        % FEEDBACK: NEWEST WINS, older ones are discarded. A report is a
        % statistical summary of the link right now, so an older one is not
        % merely redundant, it is misleading -- feeding a backlog of them to
        % the ACM policy in one go would make it react to conditions that
        % have already passed. Retransmit requests are the opposite: each
        % names different packets, so all of them are queued and serviced.
        if ~isempty(uplinkFbQueue)
            fbBytes = uplinkFbQueue{end};
            uplinkFbQueue = {};
        else
            fbBytes = [];
        end

        if ~isempty(uplinkRtQueue)
            rtBytes = uplinkRtQueue{1};
            uplinkRtQueue(1) = [];
        else
            rtBytes = [];
        end

        returnLinkUp = true;   % no connection to wait for over the air
    else
        returnLinkUp = feedbackServer.Connected;
        if returnLinkUp
            fbBytes = dvbs2TCPFrameTryRead(feedbackServer);
        else
            fbBytes = [];
        end
        if retransmitServer.Connected
            rtBytes = dvbs2TCPFrameTryRead(retransmitServer);
        else
            rtBytes = [];
        end
    end

    %% Check for ACM feedback (non-blocking) and possibly switch MODCOD
    if returnLinkUp
        if ~feedbackClientLogged
            fprintf('S1: S2 has connected to the ACM feedback server -- now listening for feedback.\n');
            feedbackClientLogged = true;
        end
        if ~isempty(fbBytes)
            feedback = dvbs2DeserializeFeedback(fbBytes);
            lastReturnTic = tic;
            if returnLinkLost
                returnLinkLost = false;
                fprintf('S1: return link RECOVERED; ACM resuming.\n');
            end
            if ~linkEstablished
                % First feedback ever received -- S2 has proven it's
                % actually receiving (this can only be a real,
                % successfully decoded run of calibration frames). Pick
                % the starting MODCOD from those samples' own mean and
                % spread, via the same ladder the steady-state policy
                % uses, so the two cannot disagree. No trend term and no
                % current MODCOD to protect: this is a cold start, so
                % there is no prior state and nothing to flap against.
                bootMu = feedback.MeanSNRdB;
                bootSigma = feedback.SigmaSNRdB;
                bootMODCOD = dvbs2SelectMODCOD(bootMu, bootSigma, 0, NaN, config);

                linkEstablished = true;
                release(cfgDVBS2);
                cfgDVBS2.MODCOD = bootMODCOD;
                cfgDVBS2.DFL = getDFL(bootMODCOD, cfgDVBS2.FECFrame);
                currentMODCOD = bootMODCOD;
                fprintf('S1: link established (S2 SNR mean=%.2f dB sigma=%.2f dB over %d frames) -- starting real traffic at MODCOD %d\n', ...
                    bootMu, bootSigma, feedback.Count, currentMODCOD);
            else
                [recommendedMODCOD, acmState] = dvbs2ACMPolicy( ...
                    feedback, currentMODCOD, acmState, config);
                if feedback.Count == 0
                    % "Alive, but decoded nothing." Distinct from silence and
                    % far more alarming -- the forward link is failing.
                    fprintf(2, 'S1: receiver reports ZERO frames decoded -- forward link is not being received.\n');
                end
                fprintf(['S1: report %d frames (mean %.2f sigma %.2f) | window mu=%.2f sigma=%.2f ' ...
                    'trend=%+.2f dB/%gs | RSSI=%.2f -> policy: MODCOD %d\n'], ...
                    feedback.Count, feedback.MeanSNRdB, feedback.SigmaSNRdB, ...
                    acmState.Mu, acmState.Sigma, acmState.TrendAdjDB, ...
                    config.acm.trendHorizonSec, feedback.RSSIdB, recommendedMODCOD);

                if recommendedMODCOD ~= currentMODCOD
                    fprintf('S1: *** switching MODCOD %d -> %d at burst boundary ***\n', ...
                        currentMODCOD, recommendedMODCOD);
                    release(cfgDVBS2);
                    cfgDVBS2.MODCOD = recommendedMODCOD;
                    cfgDVBS2.DFL = getDFL(recommendedMODCOD, cfgDVBS2.FECFrame);
                    currentMODCOD = recommendedMODCOD;
                end
            end
        end
    end

    %% Check for retransmit requests (non-blocking), queue any that arrive
    % rtBytes was filled above from whichever return link is in use; over
    % TCP it stays empty unless the retransmit server has a client.
    if returnLinkUp
        if ~isempty(rtBytes)
            request = dvbs2DeserializeRetransmitRequest(rtBytes);
            % An ARQ request proves the return link is up just as well as a
            % feedback report does, so it resets the liveness clock too.
            lastReturnTic = tic;
            if returnLinkLost
                returnLinkLost = false;
                fprintf('S1: return link RECOVERED (via retransmit request).\n');
            end

            % VALIDATE BEFORE QUEUEING. A retransmit request is derived
            % from packet indices the receiving side read out of
            % possibly-corrupted payloads, so it is untrusted input --
            % and the range it names is later expanded into an actual
            % vector (requestedRange below). An implausible EndIdx once
            % produced a 2.18-billion-element range and killed this
            % process with an out-of-memory error, taking the
            % transmitter down mid-run. config.retransmit.maxRequestRange
            % was always documented as the guard against exactly that,
            % but was only ever enforced on the sending side.
            requestLen = request.EndIdx - request.StartIdx + 1;
            if request.EndIdx < request.StartIdx
                warning('S1:MalformedRetransmitRequest', ...
                    'Dropping malformed retransmit request [%d, %d] (EndIdx before StartIdx).', ...
                    request.StartIdx, request.EndIdx);
            elseif requestLen > config.retransmit.maxRequestRange
                warning('S1:OversizedRetransmitRequest', ...
                    'Dropping retransmit request [%d, %d]: %d packets exceeds config.retransmit.maxRequestRange (%d).', ...
                    request.StartIdx, request.EndIdx, requestLen, config.retransmit.maxRequestRange);
            elseif globalPktIdx > 0 && request.StartIdx >= globalPktIdx
                % Nothing at or beyond globalPktIdx has ever been
                % transmitted, so this cannot name a packet that was
                % actually lost -- it is a corrupted index, not a gap.
                warning('S1:UnsentRetransmitRequest', ...
                    'Dropping retransmit request [%d, %d]: no packet at or beyond index %d has been transmitted yet.', ...
                    request.StartIdx, request.EndIdx, globalPktIdx);
            else
                fprintf('S1: retransmit request received for packets [%d, %d]\n', ...
                    request.StartIdx, request.EndIdx);
                retransmitQueue(end+1) = struct('StartIdx', request.StartIdx, 'EndIdx', request.EndIdx); %#ok<SAGROW>
            end
        end
    end

    %% While not yet linked (SDR mode only), transmit calibration bursts
    % at calibMODCOD instead of real data -- content is irrelevant since
    % S2 never forwards these past itself (see S2_Reciever.m), only
    % their PLHEADER/pilots matter, to give S2 something to lock onto
    % and measure regardless of when it comes online.
    if ~linkEstablished
        calibBurstNum = calibBurstNum + 1;
        pktPayloadLen = cfgDVBS2.UPL - 8;
        numPkts = cfgDVBS2.MinNumPackets * ...
            localFramesPerBurst(cfgDVBS2, txBlockSamples, numel(txFifo));
        calibIndices = 0 : numPkts - 1;
        data = dvbs2GeneratePacketBurst(calibIndices, pktPayloadLen, config.dataSeed, syncByte);
        tGen = tic;
        txFifo = [txFifo; cfgDVBS2(data)]; %#ok<AGROW>
        prof.waveformGen = prof.waveformGen + toc(tGen);

        txWaveform = txFifo(1:txBlockSamples);
        txFifo = txFifo(txBlockSamples+1:end);
        tTx = tic;
        underrun = radioTx(txWaveform);
        prof.radioTxTime = prof.radioTxTime + toc(tTx);
        if underrun
            % Counted, not warned. Printing one warning per burst put
            % hundreds of console writes into the measurement, which is slow
            % enough to be its own confound -- and the count is what matters.
            prof.underruns = prof.underruns + 1;
            if prof.underruns == 1
                fprintf(2, 'S1: TX underruns have started (calibration burst %d); counting silently from here.\n', ...
                    calibBurstNum);
            end
        end
        lastBurstSec = numel(txWaveform)/Fsamp;
        airtimeSec = airtimeSec + lastBurstSec;
        if mod(calibBurstNum, 50) == 0
            % Report the feedback link's state alongside the burst count:
            % a stalled calibration phase with the link DOWN is a
            % connection problem, whereas the same stall with the link UP
            % means S2 is connected but not yet decoding
            % config.calibLockFramesRequired consecutive frames.
            if uplinkRF
                % Over RF the useful reading is not "is a socket open" but
                % "is anything arriving" -- the squared-spectrum peak, which
                % reads 13-15 dB on noise and 30-50 dB on a live carrier.
                feedbackStateStr = sprintf('RF uplink peak %.1f dB, Es/No %.1f dB, %d decoded, %d overruns', ...
                    uplinkPeakDB, uplinkEsNodB, uplinkDecodes, uplinkOverruns);
            elseif feedbackServer.Connected
                feedbackStateStr = 'feedback link UP';
            else
                feedbackStateStr = 'feedback link DOWN -- S2 has not connected';
            end
            fprintf('S1: still calibrating (%d bursts sent, waiting for S2; %s) ...\n', ...
                calibBurstNum, feedbackStateStr);
        end
        continue;
    end

    %% Decide what (if anything) to transmit this iteration: a queued
    % retransmit request has strict priority; otherwise the next normal
    % sequential burst, unless the new-data budget is already spent, in
    % which case idle briefly and check again rather than exiting.
    isRetransmit = ~isempty(retransmitQueue);
    if ~isRetransmit && frameSeqNum >= config.maxFrames
        if ~newDataBudgetLogged
            fprintf('S1: new-data budget (%d frames) reached; staying active to service any retransmit requests ...\n', ...
                config.maxFrames);
            newDataBudgetLogged = true;
        end
        % No burst this iteration, so the uplink drain must be credited from
        % the idle time instead -- 0.1 s of real time is still 0.1 s of
        % uplink airtime arriving at the radio. Leaving lastBurstSec at its
        % previous value would credit a burst that was never sent.
        lastBurstSec = 0.1;
        pause(0.1);
        continue;
    end

    %% Generate one burst of PLFRAMEs -- either a retransmit burst (if a
    % request is queued, serviced with strict priority over new data at
    % this burst boundary) or the next normal sequential burst.
    burstStartTic = tic;   % see the pacing note below (sim mode only)
    burstNum = burstNum + 1;
    pktPayloadLen = cfgDVBS2.UPL - 8;
    % Frames per burst is derived from the CURRENT modulation and from how
    % much is already queued, rather than fixed: enough to cover one radio
    % block, no more. 4 frames at QPSK (exactly as before), 6 at 8PSK,
    % dropping to 5 on the occasional iteration where the carried-over
    % remainder has built up. That is what keeps the FIFO bounded below one
    % frame instead of growing by the per-iteration surplus, WITHOUT ever
    % skipping a burst -- skipping one would silently consume packet indices
    % or drop a retransmit request.
    framesPerBurst = localFramesPerBurst(cfgDVBS2, txBlockSamples, numel(txFifo));

    % CAP BY WHAT GENERATION CAN AFFORD. Above about 8PSK, generating enough
    % real frames to fill a block takes longer than the block's own airtime,
    % so the radio starves while this loop is busy encoding. Whatever is not
    % generated here is made up with dummy PLFRAMEs below: the carrier stays
    % continuous and the receiver's loops stay locked, at the cost of
    % throughput rather than of the link.
    maxRealFrames = max(1, floor(genBudgetSec / max(genSecPerFrame, eps)));
    framesPerBurst = min(framesPerBurst, maxRealFrames);

    numPkts = cfgDVBS2.MinNumPackets * framesPerBurst;

    if isRetransmit
        request = retransmitQueue(1);
        retransmitQueue(1) = [];
        requestedRange = request.StartIdx : request.EndIdx;

        if numel(requestedRange) > numPkts
            % Request is bigger than one burst can carry: service the
            % first numPkts now, re-queue the remainder for a later
            % burst boundary.
            pktIndices = requestedRange(1:numPkts);
            retransmitQueue(end+1) = struct('StartIdx', requestedRange(numPkts+1), 'EndIdx', request.EndIdx); %#ok<SAGROW>
        elseif numel(requestedRange) < numPkts
            % Pad forward with the subsequent sequential range to fill a
            % valid burst size. Harmless even where this overlaps with
            % what globalPktIdx will send later via normal traffic --
            % packet content is deterministic from its index alone, so
            % re-sending an already-fine index just costs a little
            % bandwidth, not correctness.
            padCount = numPkts - numel(requestedRange);
            pktIndices = [requestedRange, requestedRange(end) + (1:padCount)];
        else
            pktIndices = requestedRange;
        end

        fprintf('S1: sending RETRANSMIT burst for packets [%d..%d] (requested [%d..%d])\n', ...
            pktIndices(1), pktIndices(end), request.StartIdx, request.EndIdx);
    else
        pktIndices = globalPktIdx : globalPktIdx + numPkts - 1;
        globalPktIdx = globalPktIdx + numPkts;
    end

    data = dvbs2GeneratePacketBurst(pktIndices, pktPayloadLen, config.dataSeed, syncByte);

    tGen = tic;
    txFifo = [txFifo; cfgDVBS2(data)]; %#ok<AGROW>
    genElapsed = toc(tGen);
    prof.waveformGen = prof.waveformGen + genElapsed;

    % Running average of the per-frame cost, so the budget above tracks the
    % MODCOD automatically. Weighted 0.2 on the newest measurement: fast
    % enough to follow an ACM step within a few bursts, slow enough that one
    % slow burst does not collapse the frame count.
    genSecPerFrame = 0.8*genSecPerFrame + 0.2*(genElapsed / framesPerBurst);

    % TOP UP WITH DUMMY PLFRAMEs. Only ever reached when the generation
    % budget capped framesPerBurst -- at QPSK there is nothing to top up.
    % Cheap by construction: no BBFRAME, no BCH, no LDPC, just a cached,
    % pre-filtered run of PLS-0 frames sliced to length.
    if numel(txFifo) < txBlockSamples
        shortfall = txBlockSamples - numel(txFifo);
        txFifo = [txFifo; dvbs2DummyFiller(shortfall, ...
            cfgDVBS2.SamplesPerSymbol, cfgDVBS2.RolloffFactor, 10)]; %#ok<AGROW>
        dummySamplesSent = dummySamplesSent + shortfall;
    end

    % Hand the radio a fixed-size block off the front of the queue. This is
    % the line that makes ACM across modulations possible: whatever MODCOD
    % the frames above were built at, and however long they are, radioTx
    % only ever sees txBlockSamples and is never released.
    txWaveform = txFifo(1:txBlockSamples);
    txFifo = txFifo(txBlockSamples+1:end);

    modcodHistogram(currentMODCOD) = modcodHistogram(currentMODCOD) + framesPerBurst;
    maxFifoCarry = max(maxFifoCarry, numel(txFifo));

    %% Transmit dispatch
    if config.useSDR
        tTx = tic;
        underrun = radioTx(txWaveform);
        prof.radioTxTime = prof.radioTxTime + toc(tTx);
        if underrun
            prof.underruns = prof.underruns + 1;
            if prof.underruns == 1
                fprintf(2, 'S1: TX underruns have started (burst %d, MODCOD %d); counting silently from here.\n', ...
                    burstNum, currentMODCOD);
            end
        end
    else
        % Sim mode: stream the CLEAN transmitted samples to S2a, which
        % owns the simulated channel model. That mirrors SDR mode above,
        % where this script hands its waveform straight to the radio and
        % everything downstream of the antenna -- propagation, front-end
        % effects -- belongs to the receiving side. Sent as a continuous
        % byte stream with no message framing, since it stands in for a
        % continuous RF/sample stream rather than discrete messages.
        write(simChannelClient, dvbs2ComplexToBytes(txWaveform), 'uint8');

        % Pace to real-time: a real radio can only ever send samples as
        % fast as they actually occupy air time (length(txWaveform)/Fsamp
        % seconds for this burst); a simulated TCP link has no such
        % limit and, left unthrottled, generates+sends far faster than
        % the receiver's much heavier per-frame processing (matched
        % filter, frame sync, PLHEADER decode, fine correction) can
        % keep up with. That
        % mismatch is what floods the TCP link and the receive chain --
        % pacing here keeps sim mode a fair stand-in for real hardware
        % AND keeps this from overwhelming a resource-constrained
        % machine running all three scripts at once.
        burstDurationSec = length(txWaveform) / Fsamp;
        elapsedSec = toc(burstStartTic);
        if elapsedSec < burstDurationSec
            pause(burstDurationSec - elapsedSec);
        end
    end

    % Retransmit bursts don't count toward the new-data frame budget --
    % config.maxFrames tracks how much NEW content to send, not
    % retransmission overhead needed to actually deliver it (see the
    % maxFrames interaction note in dvbs2TestbedConfig.m).
    if ~isRetransmit
        frameSeqNum = frameSeqNum + framesPerBurst;
    end

    % Now a constant, because every radio write is txBlockSamples long. That
    % simplifies the uplink drain credit this feeds: the amount of downlink
    % airtime produced per iteration no longer moves when the MODCOD does.
    lastBurstSec = numel(txWaveform)/Fsamp;
    airtimeSec = airtimeSec + lastBurstSec;
    if mod(burstNum, 10) == 0
        if uplinkRF
            % The uplink's own health, alongside the downlink's. Overruns
            % here mean radioTx() is blocking longer than the uplink drain
            % can absorb -- the one interaction that integration introduced
            % and neither hop showed on its own.
            fprintf(['S1: burst %d transmitted (MODCOD %d, %d frames so far) | RT factor %.3f | ' ...
                'uplink %d decoded, peak %.1f dB, Es/No %.1f dB, %d overruns\n'], ...
                burstNum, currentMODCOD, frameSeqNum, toc(runTic)/max(airtimeSec,eps), ...
                uplinkDecodes, uplinkPeakDB, uplinkEsNodB, uplinkOverruns);
            uplinkPeakDB = -Inf;   % per-interval, so a dying link shows up
        else
            fprintf('S1: burst %d transmitted (MODCOD %d, %d frames so far) | RT factor %.3f\n', ...
                burstNum, currentMODCOD, frameSeqNum, toc(runTic)/max(airtimeSec,eps));
        end
    end
end

%% ------------------------------------------------------------------
%% Local functions
%% ------------------------------------------------------------------

function n = localFramesPerBurst(cfg, blockSamples, fifoLen)
%LOCALFRAMESPERBURST How many PLFRAMEs to build to cover one radio block.
%
%   Sized from what is still MISSING (blockSamples - fifoLen) rather than
%   from the block alone, which is what keeps the transmit FIFO bounded.
%   Generating a fixed count would overshoot slightly every iteration -- six
%   8PSK frames exceed a 266256-sample block by 72 samples -- and that
%   surplus would accumulate without limit. Sizing against the shortfall
%   makes the burst drop by one frame whenever the carry-over has grown
%   enough to cover it, so the queue never holds more than one frame.
%
%   Never returns 0: a burst is always generated. Skipping one would consume
%   packet indices or pop a retransmit request without transmitting either.
    n = max(1, ceil((blockSamples - fifoLen) / localPLFrameSamples(cfg)));
end

function n = localPLFrameSamples(cfg)
%LOCALPLFRAMESAMPLES Length of one PLFRAME, in samples, at the current MODCOD.
%
%   A FECFRAME is a fixed number of BITS, so its symbol count falls as the
%   modulation gets denser -- which is the whole reason the FIFO exists.
%   Verified against hardware: QPSK gives 33282 symbols, exactly what S2
%   reports decoding.
%
%       PLHEADER          90 symbols
%       payload           numSlots * 90
%       pilots            36 symbols after every 16th slot, except at the
%                         very end of the frame
%
%   The modulation is read from the object rather than mapped from the
%   MODCOD index, so this cannot drift out of step with what the generator
%   is actually producing.
    switch string(info(cfg).ModulationScheme)
        case "QPSK",   bitsPerSym = 2;
        case "8PSK",   bitsPerSym = 3;
        case "16APSK", bitsPerSym = 4;
        case "32APSK", bitsPerSym = 5;
        otherwise
            error('S1:UnknownModulation', ...
                'No PLFRAME size known for modulation "%s".', ...
                info(cfg).ModulationScheme);
    end

    if string(cfg.FECFrame) == "short"
        fecFrameBits = 16200;
    else
        fecFrameBits = 64800;
    end

    numSlots = fecFrameBits / (bitsPerSym * 90);
    numSymbols = 90 + numSlots*90;
    if cfg.HasPilots
        numSymbols = numSymbols + floor((numSlots - 1)/16) * 36;
    end

    n = numSymbols * cfg.SamplesPerSymbol;
end

function name = localModName(modcod)
%LOCALMODNAME Modulation for a legacy DVB-S2 MODCOD index, for the summary.
%   Ranges per ETSI EN 302 307-1 table 12.
    if     modcod >= 1  && modcod <= 11, name = 'QPSK';
    elseif modcod >= 12 && modcod <= 17, name = '8PSK';
    elseif modcod >= 18 && modcod <= 23, name = '16APSK';
    elseif modcod >= 24 && modcod <= 28, name = '32APSK';
    else,                                name = '?';
    end
end
