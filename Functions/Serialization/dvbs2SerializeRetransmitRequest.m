function payloadBytes = dvbs2SerializeRetransmitRequest(startIdx, endIdx)
%DVBS2SERIALIZERETRANSMITREQUEST Pack a selective-repeat ARQ request compactly.
%
%   payloadBytes = dvbs2SerializeRetransmitRequest(startIdx, endIdx) encodes
%   a request for global packet indices STARTIDX..ENDIDX (inclusive), using
%   this fixed BIT layout:
%
%     [3]   message type = 1 (ARQ retransmit request)
%     [24]  startIdx  -- 0 .. 16 777 215
%     [13]  count-1   -- 1 .. 8192 packets
%
%   40 bits, packed into 5 bytes.
%
%   WHY A COUNT RATHER THAN A SECOND ABSOLUTE INDEX: two 32-bit indices
%   would be 64 bits, but config.retransmit.maxRequestRange already caps a
%   request at 5000 packets. Encoding the span in 13 bits therefore costs
%   nothing usable AND makes an oversized request unrepresentable rather
%   than merely rejected on arrival -- a corrupted index can no longer ask
%   the transmitter to build a two-billion-element range, which is the bug
%   that previously killed S1 with a 16 GB allocation.
%
%   TimestampSec was dropped: it was written on every request and read by
%   nothing.
%
%   See dvbs2DeserializeRetransmitRequest.m for the matching decode.

    startIdx = round(startIdx);
    endIdx = round(endIdx);

    if endIdx < startIdx
        error('dvbs2SerializeRetransmitRequest:Malformed', ...
            'EndIdx (%d) precedes StartIdx (%d).', endIdx, startIdx);
    end
    if startIdx < 0 || startIdx > 16777215
        error('dvbs2SerializeRetransmitRequest:IndexRange', ...
            'StartIdx %d is outside the 24-bit range this format carries.', startIdx);
    end

    count = endIdx - startIdx + 1;
    if count > 8192
        error('dvbs2SerializeRetransmitRequest:RangeTooLarge', ...
            'Request spans %d packets; the 13-bit count field allows 8192.', count);
    end

    bits = [uintBits(1, 3); uintBits(startIdx, 24); uintBits(count-1, 13)];
    payloadBytes = dvbs2PackBits(bits);
end

function b = uintBits(v, n)
    b = double(bitget(uint32(v), n:-1:1)).';
end
