function coarseFreqEstHz = dvbs2CoarseFreqEst(rxSymbols, frameStartIdx, sofRef, symRate)
%DVBS2COARSEFREQEST Two-stage coarse carrier frequency offset (CFO) estimate.
%
%   coarseFreqEstHz = dvbs2CoarseFreqEst(rxSymbols, frameStartIdx, sofRef, symRate)
%   estimates the residual carrier frequency offset by correlating the
%   received SOF (Start Of Frame) symbols against the known reference
%   SOF pattern, using a two-stage Luise & Reggiannini (L&R) estimator
%   (see lrEstimate.m). This is meant to run BEFORE any fine
%   frequency/phase correction, to pull a potentially large CFO
%   (up to several hundred kHz) down into the narrow capture range that
%   the fine estimators (dvbs2FineFreqEst / dvbs2NonPilotFineFreqPhase)
%   expect.
%
%   Stage 1 uses a low lag order (wide capture range, higher variance)
%   to acquire most of the offset; stage 2 re-estimates the much
%   smaller residual left after stage 1 with a higher lag order (narrow
%   range, lower variance) for a more precise final estimate. Because
%   only 26 SOF symbols are available per frame, the two-stage approach
%   gets a better range/variance trade-off than a single fixed-lag
%   estimate would.
%
% Inputs:
%   rxSymbols     - Vector of received symbols at 1 sample/symbol
%                   (typically the output of dvbs2MatchedFilterTimingSync).
%   frameStartIdx - Index into rxSymbols where the PLHEADER (SOF) begins,
%                   as returned by dvbs2FrameSync.
%   sofRef        - Reference SOF symbols from dvbs2SOFReference().
%   symRate       - Symbol rate in Hz, used to convert the normalized
%                   frequency estimate (cycles/symbol) to Hz.
%
% Output:
%   coarseFreqEstHz - Estimated carrier frequency offset in Hz. Apply
%                     this correction to the received signal (or use it
%                     as the starting point for a fine frequency
%                     estimate) before further processing.

    N = length(sofRef);
    seg = rxSymbols(frameStartIdx : frameStartIdx+N-1);

    % Residual after removing the known reference SOF modulation: what
    % remains is (ideally) a pure phase ramp caused by the CFO.
    y = seg .* conj(sofRef);

    M1 = 3;                        % wide capture, same as before
    freqEst1 = lrEstimate(y, M1);  % cycles/symbol

    % Remove the stage-1 estimate before refining, so stage 2 only has
    % to resolve the small residual left over.
    n = (0:N-1).';
    yCorr = y .* exp(-1j*2*pi*freqEst1*n);

    M2 = min(N-1, 15);             % refine: much lower variance,
                                    % narrower range -- fine since
                                    % residual after stage 1 is small
    freqEst2 = lrEstimate(yCorr, M2);

    coarseFreqNorm = freqEst1 + freqEst2;   % cycles/symbol
    coarseFreqEstHz = coarseFreqNorm * symRate;
end
