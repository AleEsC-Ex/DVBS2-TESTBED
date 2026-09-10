function request = dvbs2DeserializeRetransmitRequest(payloadBytes)
%DVBS2DESERIALIZERETRANSMITREQUEST Unpack an ARQ request (see dvbs2SerializeRetransmitRequest.m).
%
%   request = dvbs2DeserializeRetransmitRequest(payloadBytes) returns a
%   struct with fields StartIdx and EndIdx, the inclusive global packet
%   index range being requested.
%
%   By construction the span can never exceed 8192 packets, because the
%   format carries a 13-bit count rather than a second absolute index.

    bits = dvbs2UnpackBits(payloadBytes);
    if numel(bits) < 40
        error('dvbs2DeserializeRetransmitRequest:Truncated', ...
            'Message is %d bits; an ARQ request needs 40.', numel(bits));
    end

    msgType = readUInt(bits, 1, 3);
    if msgType ~= 1
        error('dvbs2DeserializeRetransmitRequest:WrongType', ...
            'Message type is %d, expected 1 (ARQ retransmit request).', msgType);
    end

    request.StartIdx = readUInt(bits, 4, 24);
    request.EndIdx = request.StartIdx + readUInt(bits, 28, 13);
end

function v = readUInt(bits, pos, n)
    v = sum(bits(pos:pos+n-1).' .* 2.^(n-1:-1:0));
end
