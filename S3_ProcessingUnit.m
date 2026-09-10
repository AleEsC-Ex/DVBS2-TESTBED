%S3_PROCESSINGUNIT DVB-S2 bit recovery + BER/PER measurement.
%
%   Receives corrected PLFRAME symbols + PLHEADER metadata over TCP
%   (Functions/dvbs2DeserializePLFrame.m), runs LDPC/BCH decoding and
%   BBHEADER/MPEG-TS packet recovery (Functions/AEC_dvbs2BitRecover.m,
%   using a real per-frame noise-variance estimate rather than a
%   simulation-only static value), and measures:
%     - PER (Packet Error Rate): directly from the per-packet CRC-8
%       results AEC_dvbs2BitRecover.m already returns, no reference
%       data needed.
%     - BER (Bit Error Rate): each recovered packet has a 32-bit index
%       embedded in its payload (Functions/dvbs2ReferencePacketPayload.m);
%       this script reads that index back out and independently
%       regenerates the expected payload for comparison, bit-for-bit.
%       This is self-synchronizing -- no shared RNG replay history to
%       keep in lockstep across processes, so it stays correct across
%       any dropped/reordered frame or MODCOD-dependent packet-count
%       change.
%
%   Run this alongside the transmitter and receiver scripts, each as its
%   own MATLAB instance, IN ANY ORDER (see dvbs2TCPConnectRetry.m).

clear; clc;

% Functions/ is organized into subfolders by purpose (core DSP/PHY
% functions stay directly under Functions/; TCP/, Serialization/, and
% Testbed/ hold this testbed's supporting code) -- genpath adds all of
% them recursively, computed from this script's own location so it
% works regardless of MATLAB's current folder when this is run.
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Functions')));

config = dvbs2TestbedConfig();

plHeaderSymbols = 90;   % PLHEADER length in symbols, fixed by the DVB-S2 standard

% RUN CLOCK, STARTED HERE RATHER THAN AFTER THE CONNECTIONS.
%
% The other three scripts start their clock once their upstream is attached,
% and for them that happens almost immediately: S2a connects to S2b as soon
% as S2b's server is listening. S3 is different -- S2b only connects to S3
% AFTER link establishment, which has taken anywhere from 7 to 100 chunks
% across the runs in this project. Starting the clock there meant S3's
% 120 s began tens of seconds after everyone else's and it stopped long
% after them.
%
% Measuring from launch instead puts S3 on the same clock as the rest.
runTicS3 = tic;

%% Frame hand-off server (this processing unit is the TCP server; the receiver connects as client)
fprintf('S3: opening frame server on port %d, waiting for S2b to connect ...\n', config.framePort);
frameServer = dvbs2TCPServerRetry(config.frameHost, config.framePort, "S3's frame server");
% Bounded wait. Without this S3 blocks here forever when S2b never
% establishes the link -- and because the wait sits BEFORE the main loop,
% the duration guard inside that loop is never reached. That is the case
% where S3 appears to ignore runDurationSec entirely.
while ~frameServer.Connected
    if toc(runTicS3) >= config.runDurationSec
        fprintf(['S3: S2b never connected within %g s -- no frames to process.\n' ...
            '    (S2b only connects once it has established the link; check S2b''s log.)\n'], ...
            config.runDurationSec);
        return;
    end
    pause(0.1);
end
fprintf('S3: S2b connected.\n');

%% Retransmit-request client, connecting to the transmitter's retransmit-request server
% Selective-repeat ARQ: sends a request whenever this script detects a
% gap in the packet sequence or a packet's own CRC-8 fails. Independent
% of the frame link above.
% WHO ACTUALLY HOSTS THIS PORT DEPENDS ON THE RETURN-LINK MODE, and the
% message has to say so. It used to read "connecting to S1a" unconditionally,
% which was written when S1a hosted the port directly over TCP and never
% updated when S2a took it over for the RF uplink. That stale string sent an
% entire debugging session looking at the wrong process.
%
%   RF uplink   S2a hosts it and relays what arrives over 500 MHz to S1a
%   TCP         S1a hosts it itself, as it always did
retransmitHostName = "S1a";
if config.useSDR && config.uplink.useRF
    retransmitHostName = "S2a (relayed to S1a over the RF uplink)";
end
fprintf('S3: connecting to %s''s retransmit-request server on port %d ...\n', ...
    retransmitHostName, config.retransmitPort);
retransmitClient = dvbs2TCPConnectRetry( ...
    config.retransmitHost, config.retransmitPort, retransmitHostName + "'s retransmit-request server");
fprintf('S3: connected to %s.\n', retransmitHostName);

%% Running statistics
framesReceived = 0;
framesLost = 0;

% LINK STATE, NOW THAT AN OPEN SOCKET NO LONGER MEANS AN ESTABLISHED LINK.
% S2b connects during its own initialisation rather than after acquisition,
% so frameServer.Connected goes true almost immediately -- while S2b is
% still running its calibration frames and deliberately sending nothing.
% The first frame to arrive is therefore the link-established signal, and
% nothing in the processing loop runs before it.
linkEstablished = false;
linkEstablishedSec = NaN;   % how long acquisition took, for the profile

% Run totals for the closing profile. Indexed PLS+1 so PLS 0 lands at 1.
% Both arrays are filled for EVERY frame, decoded or lost, because
% phyParams survives a payload failure -- which is what makes the
% per-MODCOD loss breakdown possible at all.
plsSeen = zeros(1, 128);
plsLost = zeros(1, 128);
retransmitRequestsSent = 0;
% Requests whose WRITE failed, as opposed to merely being attempted. The
% distinction matters: S3 previously reported 82 "sent" while S2a read 0,
% and "sent" only ever meant the call was made. A non-zero count here says
% the break is on this side of the socket; a zero count says the bytes left
% S3 and the search moves to S2a.
retransmitSendFailures = 0;
totalPacketsOK = 0;
totalPacketsSeen = 0;
totalBitErrors = 0;
totalBitsCompared = 0;

% Exact identifiers of what was lost, not just counts -- this is what
% you'd hand to a retransmission request.
%   lostFrameSeqNums:  the frame sequence number for every PLFRAME that
%                       failed at the physical layer / BBHEADER (i.e.
%                       AEC_dvbs2BitRecover.m never got as far as
%                       extracting individual packets from it).
%   lostPacketIndices: the global packet index (embedded by
%                       dvbs2ReferencePacketPayload.m) of every packet
%                       that WAS extracted but failed its own CRC-8.
% LIMITATION: for a frame in lostFrameSeqNums, we do NOT know which
% packet indices it contained -- the packet boundaries only exist
% inside the BBFRAME payload, which is never reached when the frame
% fails before that point. Recovering that would require embedding
% frame-to-packet-range metadata outside the (possibly corrupted)
% PLHEADER at the transmitter, which isn't implemented. In practice
% this matters less than it sounds: nearly every current frame loss
% traces to one known, unfixed bug (the PLSC FECFRAME-bit misdecode
% under CFO) -- fixing that directly shrinks lostFrameSeqNums rather
% than needing to recover packet indices from inside it.
lostFrameSeqNums = [];
lostPacketIndices = [];

%% Selective-repeat ARQ state
% nextExpectedPktIdx: used only to DETECT new gaps -- once a gap is
% found this advances past it immediately, so it does NOT by itself
% guarantee the gap ever gets filled (see missingIndices below).
% Starts empty rather than assuming 0, since this script may be started
% after the transmitter has already been running a while (this
% testbed's scripts are designed to start in any order) -- the first
% packet index ever seen becomes the baseline instead of treating
% everything before it as a fake gap.
nextExpectedPktIdx = [];
% missingIndices: every individual global packet index currently
% believed outstanding (requested but not yet confirmed by a passing
% CRC). This is what actually drives retries: cleared only on a
% passing CRC, and periodically re-requested below if still non-empty
% after config.retransmit.recheckEveryFrames processed frames -- this
% is what makes a LOST RETRANSMIT burst recoverable, since
% nextExpectedPktIdx alone would never re-detect the same gap twice.
missingIndices = [];
framesSinceRecheck = 0;

fprintf('\nS3: starting processing loop ...\n');

% Deliberately not gated on config.maxFrames: the transmitter only
% treats that as a NEW-DATA budget and keeps running afterward to
% service retransmit requests, and the receiver follows along for as
% long as the transmitter does -- this script needs to do the same, or
% it would stop counting frames (new + retransmitted) right around when
% the new-data budget is spent, potentially before a resend it's still
% waiting on ever arrives. Stops via the existing disconnect detection
% below instead.
while true

    %% Wait for the next PLFRAME, but not forever
    %
    % This used to be a straight blocking read, ended only by S2b closing the
    % socket. When that close is not detected -- which is what happened on
    % the 120 s run -- S3 waits indefinitely and its profile is never
    % printed, so the whole run produces no report from the one process that
    % knows whether the PAYLOAD survived.
    %
    % Polling instead lets the duration guard actually fire. The grace
    % period exists because S3 sits at the end of the chain: S2b may still be
    % draining frames it decoded before its own clock ran out, and cutting
    % S3 off at exactly runDurationSec would discard them.
    % dvbs2TCPFrameTryRead returns [] rather than raising on a closed
    % connection, so the disconnect is detected from the server object
    % instead -- and only after a try-read comes back empty, so any frames
    % still buffered when S2b closed are drained first.
    s3Stop = false;
    while true
        plBytes = dvbs2TCPFrameTryRead(frameServer);
        if ~isempty(plBytes)
            break;
        end
        if ~frameServer.Connected
            fprintf('S3: S2b disconnected (finished or stopped); ending processing loop.\n');
            s3Stop = true;
            break;
        end
        if toc(runTicS3) >= config.runDurationSec + 10
            fprintf('S3: run duration reached; ending processing loop.\n');
            s3Stop = true;
            break;
        end
        pause(0.005);
    end
    if s3Stop
        break;
    end

    % A frame arrived, so S2b has opened its valve and the PHY link is up.
    % Everything below this point -- decoding, BER/PER accounting, gap
    % detection and ARQ -- is gated behind this, so none of it can run on
    % an idle-but-connected socket.
    if ~linkEstablished
        linkEstablished = true;
        linkEstablishedSec = toc(runTicS3);
        fprintf('S3: first frame received after %.1f s -- link established, processing starts now.\n', ...
            linkEstablishedSec);
    end

    [frameSeqNum, plSymbols_corrected, phyParams, ~, noiseVarEstimate] = ...
        dvbs2DeserializePLFrame(plBytes);

    framesReceived = framesReceived + 1;

    %% Periodic re-request of anything still outstanding (paced by
    % frames processed, not wall-clock time). Runs regardless of
    % whether THIS frame decodes -- during a bad patch with many lost
    % frames in a row is exactly when a stuck retransmit needs
    % re-requesting most. This is what makes a lost RETRANSMIT burst
    % recoverable, since the gap-detection check further down only
    % re-fires on a NEW gap, not a still-outstanding one.
    framesSinceRecheck = framesSinceRecheck + 1;
    if framesSinceRecheck >= config.retransmit.recheckEveryFrames && ~isempty(missingIndices)
        lo = min(missingIndices);
        % Cap the span even though the indices feeding it are now
        % anchored to CRC-verified packets: this asks for one contiguous
        % RANGE, so a few far-apart outstanding indices would otherwise
        % request everything between them. Anything left over is picked
        % up by the next recheck.
        hi = min(max(missingIndices), lo + config.retransmit.maxRequestRange - 1);
        fprintf('S3: re-requesting still-missing packets in [%d,%d] (%d indices outstanding)\n', ...
            lo, hi, numel(missingIndices));
        retransmitRequestsSent = retransmitRequestsSent + 1;
        if ~dvbs2SendRetransmitRequest(retransmitClient, lo, hi)
            retransmitSendFailures = retransmitSendFailures + 1;
        end
        framesSinceRecheck = 0;
    end

    % Recorded BEFORE the decode attempt, so a frame that fails is still
    % attributed to the MODCOD that produced it -- which is the whole point
    % of the per-MODCOD table in the closing profile.
    plsIdx = double(phyParams.PLSDecimalCode) + 1;
    plsSeen(plsIdx) = plsSeen(plsIdx) + 1;

    %% Bit recovery, using the receiver's real per-frame noise-variance estimate
    rxDataFrame = plSymbols_corrected(plHeaderSymbols + 1 : end);
    [dataBits, isFrameLost, crc_status] = AEC_dvbs2BitRecover(rxDataFrame, phyParams, noiseVarEstimate);

    if isFrameLost
        plsLost(plsIdx) = plsLost(plsIdx) + 1;
        framesLost = framesLost + 1;
        lostFrameSeqNums(end+1) = frameSeqNum; %#ok<SAGROW>
        % MODCOD included here, not just in the closing per-MODCOD table,
        % so a lost frame is identifiable from THIS line alone -- no need
        % to cross-reference the closing summary or S2b's own log by
        % frameSeqNum just to find out what it was. Particularly useful
        % for spotting a MODCOD S1a never actually transmitted, which is
        % the signature of a PLSC misdecode rather than a genuine loss at
        % that MODCOD (see dvbs2PLHeaderRecover.m).
        fprintf(['S3: frame %d -> LOST (physical layer / BBHEADER error), ' ...
            'PLS=%d (MODCOD %d). Frames lost so far: %d/%d\n'], ...
            frameSeqNum, phyParams.PLSDecimalCode, floor(phyParams.PLSDecimalCode/4), framesLost, framesReceived);
        continue;
    end

    %% PER: directly from the per-packet CRC-8 results, no reference needed
    totalPacketsOK = totalPacketsOK + sum(crc_status);
    totalPacketsSeen = totalPacketsSeen + numel(crc_status);

    %% BER: extract each packet's embedded index, regenerate the
    % expected payload, compare bit-for-bit.
    UPL = config.dvbs2.UPL;
    numPkts = numel(crc_status);
    pktPayloadLen = UPL - 8;

    if numel(dataBits) == UPL * numPkts
        pktMatrix = reshape(dataBits, UPL, numPkts);

        % A packet's embedded 32-bit index is only trustworthy when that
        % packet's OWN CRC-8 passed. Reading an index out of a corrupted
        % payload yields a corrupted index, and acting on one poisons
        % every consumer downstream of it: it enters missingIndices where
        % no real packet can ever clear it, it makes the periodic recheck
        % above span an absurd range (which is what previously killed the
        % transmitter with a 16 GB allocation), and it makes the BER
        % comparison regenerate the WRONG reference payload, inflating
        % the error count instead of measuring it.
        %
        % Packets within one burst always carry a CONTIGUOUS index range
        % -- dvbs2GeneratePacketBurst.m is handed a contiguous vector for
        % both normal and retransmit bursts -- so a single CRC-verified
        % packet anywhere in the frame anchors every other packet's index
        % by its position, with no need to trust their payloads at all.
        anchorK = find(crc_status, 1);
        frameBaseIdx = [];
        if ~isempty(anchorK)
            anchorPayload = pktMatrix(9:end, anchorK);
            anchorIdx = bi2de(anchorPayload(1:32)', 'left-msb');
            frameBaseIdx = anchorIdx - (anchorK - 1);
            if frameBaseIdx < 0
                % CRC-8 lets roughly 1 in 256 corrupted packets through,
                % so even an anchor can occasionally be wrong. A negative
                % base is proof this one is -- discard it rather than
                % derive a whole frame's indices from it.
                frameBaseIdx = [];
            end
        end

        if isempty(frameBaseIdx)
            % No trustworthy index anywhere in this frame. PER is already
            % counted above; skip everything index-derived rather than
            % inventing positions from corrupted payloads. An empty range
            % skips the loop below without needing to nest it.
            warning('S3:NoTrustedIndex', ...
                'Frame %d: no packet passed CRC, so no trustworthy packet index -- skipping BER and ARQ bookkeeping for this frame.', ...
                frameSeqNum);
            pktRange = [];
        else
            pktRange = 1:numPkts;
        end

        for k = pktRange
            payload = pktMatrix(9:end, k);   % skip the 8-bit sync byte
            % Derived from the CRC-verified anchor by position, NOT read
            % from this packet's own (possibly corrupted) payload.
            pktIdx = frameBaseIdx + (k - 1);
            expectedPayload = dvbs2ReferencePacketPayload(config.dataSeed, pktIdx, pktPayloadLen);

            totalBitErrors = totalBitErrors + sum(payload ~= expectedPayload);
            totalBitsCompared = totalBitsCompared + pktPayloadLen;

            %% Selective-repeat ARQ: gap detection against the packet
            % sequence, independent of this packet's own CRC result.
            if isempty(nextExpectedPktIdx)
                nextExpectedPktIdx = pktIdx;   % baseline: first packet ever seen
            end
            if pktIdx > nextExpectedPktIdx
                gapLo = nextExpectedPktIdx;
                gapHi = pktIdx - 1;
                if (gapHi - gapLo + 1) > config.retransmit.maxRequestRange
                    % Implausibly large gap. With indices now anchored to
                    % a CRC-verified packet this should be rare, but a
                    % false CRC pass could still produce one.
                    warning('S3:ImplausibleGap', ...
                        'Ignoring implausible gap [%d,%d] (%d packets) -- likely a corrupted index, not requesting retransmit.', ...
                        gapLo, gapHi, gapHi - gapLo + 1);
                else
                    missingIndices = union(missingIndices, gapLo:gapHi);
                    fprintf('S3: gap detected -- packets [%d,%d] missing, requesting retransmit\n', gapLo, gapHi);
                    retransmitRequestsSent = retransmitRequestsSent + 1;
        if ~dvbs2SendRetransmitRequest(retransmitClient, gapLo, gapHi)
            retransmitSendFailures = retransmitSendFailures + 1;
        end
                end
                nextExpectedPktIdx = pktIdx + 1;
            elseif pktIdx == nextExpectedPktIdx
                nextExpectedPktIdx = pktIdx + 1;
            end
            % pktIdx < nextExpectedPktIdx: a late/retransmitted or
            % duplicate packet -- neither a new gap nor moves the
            % baseline backward.

            if crc_status(k)
                % Only a PASSING CRC actually confirms this index is
                % resolved -- a packet can arrive (satisfying the gap
                % check above) with corrupted content, which must stay
                % outstanding.
                missingIndices(missingIndices == pktIdx) = [];
            else
                % This packet's own CRC-8 failed even though the frame it
                % came from decoded overall. pktIdx is still reliable
                % (it came from the anchor, not from this payload), so it
                % is usable directly in a retransmission request.
                lostPacketIndices(end+1) = pktIdx; %#ok<SAGROW>
                if ~ismember(pktIdx, missingIndices)
                    missingIndices(end+1) = pktIdx; %#ok<SAGROW>
                end
                retransmitRequestsSent = retransmitRequestsSent + 1;
        if ~dvbs2SendRetransmitRequest(retransmitClient, pktIdx, pktIdx)
            retransmitSendFailures = retransmitSendFailures + 1;
        end
            end
        end
    else
        % Recovered bit count doesn't match UPL*numPkts -- something
        % upstream is inconsistent for this frame; skip BER for it
        % rather than reshaping into garbage, but still count PER above.
        warning('S3:UnexpectedBitCount', ...
            'Frame %d: recovered %d bits, expected %d (UPL*numPkts) -- skipping BER for this frame.', ...
            frameSeqNum, numel(dataBits), UPL*numPkts);
    end

    %% Live stats
    currentBER = totalBitErrors / max(totalBitsCompared, 1);
    currentPER = 1 - totalPacketsOK / max(totalPacketsSeen, 1);
    fprintf(['S3: frame %d | PLS=%d | SNR(fwd)=%.2f dB | Packets %d/%d OK | ' ...
        'cum BER=%.3e | cum PER=%.3e (framesLost=%d/%d)\n'], ...
        frameSeqNum, phyParams.PLSDecimalCode, 10*log10(1/max(noiseVarEstimate, eps)), ...
        sum(crc_status), numel(crc_status), currentBER, currentPER, framesLost, framesReceived);
end

fprintf('\n=== S3 PROFILE === %.1f s wall | %d frames received, %d lost (%.1f%%)\n', ...
    toc(runTicS3), framesReceived, framesLost, 100*framesLost/max(framesReceived,1));
% Acquisition cost, separated from processing time. The socket to S2b is now
% open from initialisation, so this is the genuine PHY acquisition delay
% rather than a TCP connect time -- and it is dead time on S3's run clock.
if isnan(linkEstablishedSec)
    fprintf('  link: NEVER established -- socket connected but S2b sent no frames\n');
else
    fprintf('  link: established %.1f s into the run (%.0f%% of the clock spent waiting)\n', ...
        linkEstablishedSec, 100*linkEstablishedSec/max(toc(runTicS3), eps));
end
fprintf('  BER %.3e over %d bits | PER %.3e over %d packets\n', ...
    totalBitErrors/max(totalBitsCompared,1), totalBitsCompared, ...
    1 - totalPacketsOK/max(totalPacketsSeen,1), totalPacketsSeen);

% PER-MODCOD LOSS. The most useful line in this report, and the one the
% other three scripts cannot produce: S2b knows which MODCOD it decoded and
% S1a knows which it sent, but only here is it known whether the PAYLOAD
% survived. phyParams arrives with every frame including the ones that fail,
% so a lost frame still carries the PLS code that produced it.
%
% This is what exposes a rung the ACM should not be using. On the first
% full-ladder run the aggregate loss was 4.2%, which looked acceptable --
% but it was 1% at 16APSK and 45% at 32APSK, and only a per-MODCOD split
% shows that.
seenPLS = find(plsSeen > 0);
if ~isempty(seenPLS)
    fprintf('  per-MODCOD delivery:\n');
    fprintf('    %-8s %-9s %8s %8s %9s\n', 'MODCOD', 'mod', 'frames', 'lost', 'loss %%');
    for p = seenPLS
        modcod = floor((p-1)/4);
        tot = plsSeen(p);
        lost = plsLost(p);
        fprintf('    %-8d %-9s %8d %8d %8.1f%%\n', ...
            modcod, localModName(modcod), tot, lost, 100*lost/max(tot,1));
    end
end

fprintf('  ARQ: %d retransmit requests attempted, %d write failures | %d indices outstanding\n\n', ...
    retransmitRequestsSent, retransmitSendFailures, numel(missingIndices));

fprintf('S3: stopping (received %d frames).\n', framesReceived);
fprintf('S3: final BER=%.3e over %d bits, PER=%.3e over %d packets, %d/%d frames lost.\n', ...
    totalBitErrors/max(totalBitsCompared,1), totalBitsCompared, ...
    1 - totalPacketsOK/max(totalPacketsSeen,1), totalPacketsSeen, ...
    framesLost, framesReceived);

fprintf('\n=== Loss detail (for retransmission requests) ===\n');
fprintf('Lost frame sequence numbers (%d): %s\n', ...
    numel(lostFrameSeqNums), mat2str(lostFrameSeqNums));
fprintf('Lost packet global indices, from otherwise-decoded frames (%d): %s\n', ...
    numel(lostPacketIndices), mat2str(lostPacketIndices));
fprintf(['NOTE: packet indices for the %d lost FRAMES above are not\n' ...
    'recoverable (the frame failed before its packet boundaries could\n' ...
    'be extracted at all) -- only whole-frame retransmission is possible\n' ...
    'for those; the packet-index list above only covers CRC failures\n' ...
    'inside frames that otherwise decoded.\n'], numel(lostFrameSeqNums));

fprintf('\n=== Selective-repeat ARQ status ===\n');
if isempty(missingIndices)
    fprintf('missingIndices is empty -- every requested retransmission was eventually confirmed.\n');
else
    fprintf(['%d packet indices still outstanding at end of run (retransmit\n' ...
        'requests sent but not yet confirmed by a passing CRC): %s\n'], ...
        numel(missingIndices), mat2str(missingIndices));
end


function name = localModName(modcod)
%LOCALMODNAME Modulation for a legacy DVB-S2 MODCOD index, for the profile.
%   Ranges per ETSI EN 302 307-1 table 12.
    if     modcod >= 1  && modcod <= 11, name = 'QPSK';
    elseif modcod >= 12 && modcod <= 17, name = '8PSK';
    elseif modcod >= 18 && modcod <= 23, name = '16APSK';
    elseif modcod >= 24 && modcod <= 28, name = '32APSK';
    else,                                name = 'dummy';
    end
end

