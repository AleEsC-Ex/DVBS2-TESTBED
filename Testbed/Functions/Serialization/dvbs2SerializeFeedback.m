function payloadBytes = dvbs2SerializeFeedback(feedback)
%DVBS2SERIALIZEFEEDBACK Pack an ACM feedback report into a compact bit layout.
%
%   payloadBytes = dvbs2SerializeFeedback(feedback) encodes FEEDBACK (fields
%   Count, MeanSNRdB, SigmaSNRdB, RSSIdB) for transport over either the TCP
%   link or the DBPSK return link, using this fixed BIT layout:
%
%     [3]   message type = 0 (ACM feedback)
%     [10]  RSSIdB      unsigned, (rssi + 96) * 8   -> -96..0 dB, 0.125 dB
%     [8]   Count       frames summarised, 0..255
%     [10]  MeanSNRdB   signed, mean * 16           -> +-32 dB, 0.0625 dB
%     [8]   SigmaSNRdB  unsigned, sigma * 16        -> 0..16 dB, 0.0625 dB
%
%   39 bits, packed into 5 bytes, REGARDLESS of how many frames were
%   summarised.
%
%   WHY STATISTICS RATHER THAN RAW SAMPLES: dvbs2ACMPolicy.m consumes
%   exactly a mean, a standard deviation and a trend. Mean and sigma of a
%   set of batches recombine EXACTLY from per-batch (n, mean, sigma) --
%
%       n = sum(ni),  mu = sum(ni*mui)/n
%       sigma^2 = sum(ni*(sigmai^2 + mui^2))/n - mu^2
%
%   -- so nothing the policy uses is lost. What is gained is that the
%   message size stops depending on how many frames were averaged:
%   summarising 75 frames costs the same 5 bytes as summarising 5. That is
%   what makes long, event-driven reporting intervals affordable.
%
%   The trend is then computed ACROSS batches rather than within one, which
%   is if anything better: batch means are far less noisy than individual
%   frame estimates, whose own spread is about 0.4 dB.
%
%   COUNT = 0 IS MEANINGFUL. It says "the receiver is alive but decoded no
%   frames in this interval" -- a distinct condition from silence, and one
%   the transmitter should react to by dropping to a robust MODCOD.
%
%   See dvbs2DeserializeFeedback.m for the matching decode.

    bits = [ ...
        uintBits(0, 3); ...
        uintBits(clampRound((feedback.RSSIdB + 96)*8, 0, 1023), 10); ...
        uintBits(clampRound(feedback.Count, 0, 255), 8); ...
        intBits( clampRound(feedback.MeanSNRdB*16, -512, 511), 10); ...
        uintBits(clampRound(feedback.SigmaSNRdB*16, 0, 255), 8)];

    payloadBytes = dvbs2PackBits(bits);
end

function q = clampRound(x, lo, hi)
    if ~isfinite(x), x = lo; end
    q = min(max(round(x), lo), hi);
end

function b = uintBits(v, n)
    b = double(bitget(uint32(v), n:-1:1)).';
end

function b = intBits(v, n)
    if v < 0, v = v + 2^n; end
    b = uintBits(v, n);
end
