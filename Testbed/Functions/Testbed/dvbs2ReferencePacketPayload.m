function payloadBits = dvbs2ReferencePacketPayload(dataSeed, globalPktIdx, pktPayloadLen)
%DVBS2REFERENCEPACKETPAYLOAD Deterministic, self-synchronizing test-packet payload.
%
%   payloadBits = dvbs2ReferencePacketPayload(dataSeed, globalPktIdx, pktPayloadLen)
%   builds one packet's payload bits as [32-bit globalPktIdx][deterministic
%   fill bits], where the fill is regenerated from a seed offset by
%   globalPktIdx itself (rng(dataSeed + globalPktIdx)). This makes every
%   packet's expected content independently re-derivable from its own
%   embedded index alone: the transmitter calls this while generating
%   each transmitted packet, and the receiving side calls it again for
%   each RECOVERED packet (after reading back the index it embeds) to
%   build the matching reference for bit-for-bit BER comparison.
%
%   This is deliberately NOT a simple "call rng(dataSeed) once, then
%   draw packets in sequence" scheme -- that would require both sides to
%   replay an identical, ordered draw history in lockstep across two
%   separate processes, which silently breaks under any frame loss,
%   reordering, or MODCOD-dependent packet-count change between bursts
%   (MinNumPackets varies with MODCOD, so burst sizes in packets aren't
%   constant once ACM starts switching). Seeding per-packet from its own
%   index instead makes every packet independently verifiable no matter
%   what happened to any other packet.
%
% Inputs:
%   dataSeed      - Shared base seed (config.dataSeed from dvbs2TestbedConfig.m).
%   globalPktIdx  - This packet's position in the overall transmitted
%                   stream (0-based, monotonically increasing across the
%                   whole run, independent of burst/MODCOD boundaries).
%   pktPayloadLen - Total payload length in bits (must be > 32).
%
% Output:
%   payloadBits - Column vector of pktPayloadLen bits:
%                 [32-bit big-endian globalPktIdx; deterministic fill bits].

    indexBits = de2bi(double(globalPktIdx), 32, 'left-msb')';
    % rng() requires a seed in [0, 2^32-1]; wrap rather than error on
    % arbitrarily long runs with very large packet indices.
    rng(mod(dataSeed + globalPktIdx, 2^32 - 1));
    fillBits = randi([0 1], pktPayloadLen - 32, 1);
    payloadBits = [indexBits(:); fillBits(:)];
end
