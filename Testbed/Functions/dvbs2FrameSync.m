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
%
%   VECTORIZED, NOT LOOPED -- and why that matters here specifically. S2b's
%   receive loop escalates searchLen to 2500 after a few consecutive misses
%   and does not reset it back down until a real frame is found (see
%   S2b_Reciever.m), so during a pure-noise acquisition stretch this function
%   gets called repeatedly at its most expensive width. Measured on
%   hardware: those chunks ran at RT factor 18-27 (i.e. 18-27x slower than
%   real time) versus ~1 once locked, entirely attributable to this
%   function's previous per-position MATLAB for-loop -- the total
%   arithmetic (searchLen*(N-1) multiply-adds) was unchanged, but every one
%   of up to 2500 iterations paid MATLAB's interpreter dispatch overhead on
%   top of a few microseconds of real work.
%
%   THE REWRITE, CONCEPTUALLY: every per-position "segDiff" the old loop
%   built was just a sliding window into ONE differential sequence,
%   rxDiff = rxSymbols(2:end).*conj(rxSymbols(1:end-1)). So the numerator
%   (a sliding dot product of rxDiff against sofDiff) is a correlation, and
%   the denominator (a sliding sum of |rxDiff|^2) is a sliding-window
%   energy -- both textbook operations with fast compiled primitives
%   (conv, in both cases below), computed once for the whole search range
%   instead of position by position.
%
%   THE FLIP. conv computes convolution, which time-reverses its second
%   argument by definition -- that is NOT what a correlation search wants
%   (sofDiff has no symmetry, so convolving against it as-is searches for
%   the SOF pattern backwards, which is simply not present in the signal).
%   Pre-reversing (flipud) and conjugating sofDiff before handing it to
%   conv cancels conv's built-in reversal, leaving plain correlation --
%   this is exactly the standard matched-filter construction (a matched
%   filter's impulse response is the conjugate time-reversal of the
%   template it detects).
%
%   EQUIVALENCE, MEASURED NOT ASSUMED: this replacement was checked against
%   the original loop implementation on both random and realistic (actual
%   dvbs2SOFReference) input across the searchLen values S2b actually uses
%   (100, 1000, 2500) before replacing it -- see the verification run
%   referenced in the commit/session notes. config.frameSyncPeakThreshold
%   was tuned against the old numeric output, so corrProfile had to match
%   to floating-point precision, not just "look right".

N = length(sofRef);
if length(rxSymbols) < searchLen + N - 1
    error("rxSymbols too short for the requested searchLen + SOF length.");
end

sofDiff = sofRef(2:end) .* conj(sofRef(1:end-1));
sofDiff = sofDiff(:);
refEnergy = sum(abs(sofDiff).^2);

rxDiff = rxSymbols(2:end) .* conj(rxSymbols(1:end-1));
rxDiff = rxDiff(:);

% Correlation via convolution: pre-flip+conjugate the kernel so conv's own
% time-reversal cancels out (see header comment). 'valid' keeps only fully-
% overlapping positions, i.e. exactly the same n=1..(length(rxDiff)-N+2)
% range the old loop could have covered.
numeratorFull = conv(rxDiff, conj(flipud(sofDiff)), 'valid');

% Sliding-window energy: same 'valid' convolution, this time against a
% length-(N-1) all-ones kernel (symmetric, so no flip needed) -- computes
% sum(|rxDiff(n:n+N-2)|^2) for every n in one call, aligned index-for-index
% with numeratorFull above since both are 'valid' convolutions of two
% length-(N-1) kernels against the same length-(L-1) input.
segEnergyFull = conv(abs(rxDiff).^2, ones(N-1,1), 'valid');

corrProfile = abs(numeratorFull(1:searchLen)) ./ ...
    sqrt(refEnergy * segEnergyFull(1:searchLen) + eps);
corrProfile = corrProfile(:);

[peakMetric, frameStartIdx] = max(corrProfile);

end
