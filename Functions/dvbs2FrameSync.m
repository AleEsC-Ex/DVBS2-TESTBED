function [frameStartIdx, peakMetric, corrProfile] = dvbs2FrameSync(rxSymbols, sofRef, searchLen)
%DVBS2FRAMESYNC Locate PLHEADER start via differential correlation against SOF.
%
%   [frameStartIdx, peakMetric, corrProfile] = dvbs2FrameSync(rxSymbols, sofRef, searchLen)
%
%   rxSymbols  - timing-recovered symbols at 1 sample/symbol (output of
%                dvbs2MatchedFilterTimingSync)
%   sofRef     - reference SOF symbols from dvbs2SOFReference()
%   searchLen  - number of symbol start-positions to search over. Must be
%                at least one full PL frame length for the SHORTEST frame
%                you expect (short FECFRAME, highest-order modulation has
%                the fewest symbols/frame), so the true frame boundary is
%                guaranteed to fall inside the search window.
%
%   frameStartIdx - index into rxSymbols where the PLHEADER (SOF) begins
%   peakMetric     - normalized correlation peak height in [0,1]. Compare
%                     against a detection threshold (start around 0.5-0.7
%                     and tune against your EsNo operating point) to
%                     decide whether a frame was actually found or this
%                     is a false trigger from noise.
%   corrProfile    - full correlation magnitude profile vs. search
%                     position, useful for plotting/debugging if
%                     detection is unreliable.

N = length(sofRef);
if length(rxSymbols) < searchLen + N - 1
    error("rxSymbols too short for the requested searchLen + SOF length.");
end

% Differential (one-lag) products of the reference SOF
sofDiff = sofRef(2:end) .* conj(sofRef(1:end-1));

corrProfile = zeros(searchLen,1);

refEnergy = sum(abs(sofDiff).^2);

for n = 1:searchLen
    seg = rxSymbols(n:n+N-1);
    segDiff = seg(2:end) .* conj(seg(1:end-1));

    numerator = abs(sum(segDiff .* conj(sofDiff)));
    segEnergy = sum(abs(segDiff).^2);

    corrProfile(n) = numerator / ...
        sqrt(refEnergy * segEnergy + eps);
end

[peakMetric, frameStartIdx] = max(corrProfile);

end