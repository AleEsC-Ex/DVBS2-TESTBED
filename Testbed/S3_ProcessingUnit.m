% S3_PROCESSINGUNIT DVB-S2 bit recovery + BER/PER measurement.

clear; clc;

% Functions/ is organized into subfolders by purpose (core DSP/PHY
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Functions')));

config = dvbs2TestbedConfig();

plHeaderSymbols = 90;   % PLHEADER length in symbols, fixed by the DVB-S2 standard

% RUN CLOCK, STARTED HERE RATHER THAN AFTER THE CONNECTIONS.
runTicS3 = tic;

%% Frame hand-off server (this processing unit is the TCP server; the receiver connects as client)
fprintf('S3: opening frame server on port %d, waiting for S2b to connect ...\n', config.framePort);
frameServer = dvbs2TCPServerRetry(config.frameHost, config.framePort, "S3's frame server");
% Bounded wait. Without this S3 blocks here forever when S2b never
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
linkEstablished = false;
linkEstablishedSec = NaN;   % how long acquisition took, for the profile

% Run totals for the closing profile. Indexed PLS+1 so PLS 0 lands at 1.
plsSeen = zeros(1, 128);
plsLost = zeros(1, 128);
retransmitRequestsSent = 0;
% Requests whose WRITE failed, as opposed to merely being attempted. The
retransmitSendFailures = 0;
totalPacketsOK = 0;
totalPacketsSeen = 0;
totalBitErrors = 0;
totalBitsCompared = 0;

% Exact identifiers of what was lost, not just counts -- this is what
lostFrameSeqNums = [];
lostPacketIndices = [];

%% Selective-repeat ARQ state
% nextExpectedPktIdx: used only to DETECT new gaps -- once a gap is
nextExpectedPktIdx = [];
% missingIndices: every individual global packet index currently
missingIndices = [];
framesSinceRecheck = 0;

fprintf('\nS3: starting processing loop ...\n');

% Deliberately not gated on config.maxFrames: the transmitter only
while true

    %% Wait for the next PLFRAME, but not forever
    % This used to be a straight blocking read, ended only by S2b closing the
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
    framesSinceRecheck = framesSinceRecheck + 1;
    if framesSinceRecheck >= config.retransmit.recheckEveryFrames && ~isempty(missingIndices)
        lo = min(missingIndices);
        % Cap the span even though the indices feeding it are now
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
        anchorK = find(crc_status, 1);
        frameBaseIdx = [];
        if ~isempty(anchorK)
            anchorPayload = pktMatrix(9:end, anchorK);
            anchorIdx = bi2de(anchorPayload(1:32)', 'left-msb');
            frameBaseIdx = anchorIdx - (anchorK - 1);
            if frameBaseIdx < 0
                % CRC-8 lets roughly 1 in 256 corrupted packets through,
                frameBaseIdx = [];
            end
        end

        if isempty(frameBaseIdx)
            % No trustworthy index anywhere in this frame. PER is already
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

            if crc_status(k)
                % Only a PASSING CRC actually confirms this index is
                missingIndices(missingIndices == pktIdx) = [];
            else
                % This packet's own CRC-8 failed even though the frame it
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
% LOCALMODNAME Modulation for a legacy DVB-S2 MODCOD index, for the profile.
    if     modcod >= 1  && modcod <= 11, name = 'QPSK';
    elseif modcod >= 12 && modcod <= 17, name = '8PSK';
    elseif modcod >= 18 && modcod <= 23, name = '16APSK';
    elseif modcod >= 24 && modcod <= 28, name = '32APSK';
    else,                                name = 'dummy';
    end
end

