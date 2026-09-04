function [rxOut, freqOffsetEst] = dvbs2RawCFOCompensate(rxIn, sampleRate, modulation, freqResolutionHz)
%DVBS2RAWCFOCOMPENSATE Raw-sample-domain coarse CFO estimate/correction.
%
%   [rxOut, freqOffsetEst] = dvbs2RawCFOCompensate(rxIn, sampleRate, modulation)
%   blindly estimates and removes the carrier frequency offset (CFO)
%   from a block of raw, oversampled receive samples, using
%   comm.CoarseFrequencyCompensator. Unlike dvbs2CoarseFreqEst (which
%   works on already timing-recovered SYMBOLS, using the known 26-symbol
%   SOF pattern), this operates BLIND (no known reference) directly on
%   raw samples, and is meant to run BEFORE the matched-filter/Gardner
%   symbol-timing-recovery stage (dvbs2MatchedFilterTimingSync), not
%   after it.
%
%   WHY THIS STAGE EXISTS: a non-data-aided symbol timing recovery loop
%   (e.g. Gardner) can be biased by an uncorrected carrier offset,
%   producing a mistimed sampling instant and the intersymbol
%   interference that comes with it -- an error mechanism no amount of
%   downstream FREQUENCY/PHASE correction (dvbs2CoarseFreqEst,
%   dvbs2FineFreqEst, dvbs2PhaseCompensate) can undo, since those only
%   rotate already-mistimed symbols. Removing most of the CFO before
%   timing recovery keeps the Gardner loop's input clean. Applying it
%   before the matched filter specifically (rather than after the
%   filter but before symbol sync) also avoids the RRC filter's
%   passband clipping part of an off-center spectrum, which matters
%   once the offset is large relative to the channel's excess bandwidth
%   (e.g. real LEO Doppler, which can be much larger than a lab CFO).
%
%   This is a coarse, wide-capture-range, feed-forward BLOCK estimate
%   (comm.CoarseFrequencyCompensator is not a closed-loop FLL/PLL: each
%   call independently estimates one offset for that whole input block
%   via FFT/correlation, with no loop-filter state carried between
%   calls). dvbs2CoarseFreqEst (run afterwards, per-frame, on the
%   26-symbol SOF once frame sync has located it) then refines whatever
%   residual this stage leaves behind -- a standard coarse-then-fine
%   acquisition split.
%
% Inputs:
%   rxIn        - Column vector of raw complex baseband samples at sps
%                 samples/symbol (e.g. straight out of AGC, BEFORE the
%                 matched filter).
%   sampleRate  - Sample rate in Hz of rxIn (e.g. Fsym*sps).
%   modulation  - Modulation type string passed to
%                 comm.CoarseFrequencyCompensator (e.g. "QPSK" -- the
%                 dominant modulation for the payload; the 90-symbol
%                 pi/2-BPSK PL header is a negligible fraction of any
%                 realistically sized block and does not meaningfully
%                 bias the estimate).
%   freqResolutionHz - (Optional, default 1000) FFT resolution requested
%                 of comm.CoarseFrequencyCompensator. This directly sets
%                 the QUANTISATION of the estimate: the compensator picks
%                 an FFT length of 2^nextpow2(sampleRate/(res*M)), giving
%                 a step of sampleRate/FFTLength/M. At 2 Msps, QPSK and
%                 res=1000 that step is 976.5625 Hz -- large enough that
%                 a true offset sitting between two bins makes successive
%                 estimates dither between them, injecting a ~977 Hz step
%                 into every downstream stage. res=100 takes the FFT to
%                 8192 points and the step to ~61 Hz for about 1.5x the
%                 compute. Smaller values cost more FFT and require the
%                 input block to be at least one FFT long.
%
% Outputs:
%   rxOut         - CFO-corrected samples, same size as rxIn.
%   freqOffsetEst - Estimated frequency offset for this block, in Hz.
%
%   TUNING NOTE: comm.CoarseFrequencyCompensator's Algorithm,
%   FrequencyResolution and MaximumFrequencyOffset properties trade off
%   capture range, accuracy and computation cost. The defaults below
%   are a reasonable starting point for a lab-scale CFO; revisit
%   MaximumFrequencyOffset once real LEO Doppler figures (which can
%   exceed this simulation's CFO by a wide margin) are known.

persistent cfoComp lastConfig

if nargin < 4 || isempty(freqResolutionHz)
    freqResolutionHz = 1000;
end

% comm.CoarseFrequencyCompensator locks its expected input length on
% first call and errors on any later call with a different length
% unless released first -- include it here so a chunk-to-chunk length
% change (e.g. from a partially-filled receive buffer) reconfigures
% instead of crashing.
thisConfig = {sampleRate, modulation, length(rxIn), freqResolutionHz};

if isempty(cfoComp) || ~isequal(thisConfig, lastConfig)
    if ~isempty(cfoComp)
        release(cfoComp);
    end
    cfoComp = comm.CoarseFrequencyCompensator( ...
        Modulation=modulation, ...
        SampleRate=sampleRate, ...
        FrequencyResolution=freqResolutionHz);
    lastConfig = thisConfig;
end

[rxOut, freqOffsetEst] = cfoComp(rxIn(:));

end
