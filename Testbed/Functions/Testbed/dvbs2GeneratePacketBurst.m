function data = dvbs2GeneratePacketBurst(pktIndices, pktPayloadLen, dataSeed, syncByte)
%DVBS2GENERATEPACKETBURST Build a serialized packet bitstream for an explicit list of packet indices.
%
%   data = dvbs2GeneratePacketBurst(pktIndices, pktPayloadLen, dataSeed, syncByte)
%   builds one [syncByte; payload] packet per entry of PKTINDICES (in the
%   given order, via dvbs2ReferencePacketPayload.m) and concatenates them
%   into the column-major [sync;payload] bitstream cfgDVBS2(data) expects.
%
%   Used by the transmitter for BOTH normal sequential transmission
%   (pktIndices = a running counter's next block) and retransmission
%   bursts (pktIndices = an explicit, possibly non-sequential
%   requested-and-padded range) -- since a packet's content is 100%
%   deterministic from its own index (dvbs2ReferencePacketPayload.m),
%   there is no difference in how a "new" vs. "resent" packet is built,
%   only in which indices are asked for.
%
% Inputs:
%   pktIndices    - Row or column vector of global packet indices, in
%                   the order they should be transmitted.
%   pktPayloadLen - Payload length in bits per packet (UPL - 8).
%   dataSeed      - Shared base seed (config.dataSeed from dvbs2TestbedConfig.m).
%   syncByte      - 8x1 (or 1x8) MPEG-TS sync byte bit vector (0x47),
%                   prepended to every packet.
%
% Output:
%   data - Column vector of bits, ready to pass to cfgDVBS2(data).

    pktIndices = pktIndices(:).';
    numPkts = numel(pktIndices);
    syncByte = syncByte(:);

    pkts = zeros(numel(syncByte) + pktPayloadLen, numPkts);
    for p = 1:numPkts
        pkts(:, p) = [syncByte; dvbs2ReferencePacketPayload(dataSeed, pktIndices(p), pktPayloadLen)];
    end
    data = pkts(:);
end
