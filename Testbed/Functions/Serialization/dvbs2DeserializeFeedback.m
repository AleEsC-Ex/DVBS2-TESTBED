function feedback = dvbs2DeserializeFeedback(payloadBytes)
%DVBS2DESERIALIZEFEEDBACK Unpack an ACM feedback report (see dvbs2SerializeFeedback.m).
%
%   feedback = dvbs2DeserializeFeedback(payloadBytes) returns a struct with
%   fields Count, MeanSNRdB, SigmaSNRdB and RSSIdB summarising the frames
%   the receiver decoded since its previous report.
%
%   Count == 0 means the receiver is alive but decoded nothing in the
%   interval; MeanSNRdB and SigmaSNRdB carry no information in that case.
%
%   Throws dvbs2DeserializeFeedback:WrongType if the 3-bit type field does
%   not say "ACM feedback" -- on the return link both message types share
%   one channel, so the type is checked rather than assumed.

    bits = dvbs2UnpackBits(payloadBytes);
    if numel(bits) < 39
        error('dvbs2DeserializeFeedback:Truncated', ...
            'Message is %d bits; an ACM feedback report needs 39.', numel(bits));
    end

    msgType = readUInt(bits, 1, 3);
    if msgType ~= 0
        error('dvbs2DeserializeFeedback:WrongType', ...
            'Message type is %d, expected 0 (ACM feedback).', msgType);
    end

    feedback.RSSIdB     = readUInt(bits, 4, 10)/8 - 96;
    feedback.Count      = readUInt(bits, 14, 8);
    feedback.MeanSNRdB  = readInt( bits, 22, 10)/16;
    feedback.SigmaSNRdB = readUInt(bits, 32, 8)/16;
end

function v = readUInt(bits, pos, n)
    v = sum(bits(pos:pos+n-1).' .* 2.^(n-1:-1:0));
end

function v = readInt(bits, pos, n)
    v = readUInt(bits, pos, n);
    if v >= 2^(n-1), v = v - 2^n; end
end
