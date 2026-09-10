function [symbolsOut, timingErr, lostLock] = dvbs2MatchedFilterTimingSync(rxIn, sps, rolloff, filtSpan)
%DVBS2MATCHEDFILTERTIMINGSYNC Matched filter + Gardner symbol timing recovery.
%
%   [symbolsOut, timingErr, lostLock] = dvbs2MatchedFilterTimingSync(rxIn, sps, rolloff, filtSpan)
%
%   rxIn      - received samples at SPS samples/symbol (complex baseband,
%               after your channel model / SDR front end)
%   sps       - samples per symbol (matches cfgDVBS2.SamplesPerSymbol)
%   rolloff   - RRC roll-off factor (matches cfgDVBS2.RolloffFactor)
%   filtSpan  - matched filter span in symbols (10-20 is typical; larger
%               span = better filtering, more delay/latency)
%
%   symbolsOut - recovered symbols at 1 sample/symbol, timing-corrected.
%                EMPTY when lostLock is true -- see below.
%   timingErr  - Gardner TED error signal, useful for diagnosing whether
%                the loop has converged (should settle near zero)
%   lostLock   - true if the Gardner loop has run away far enough that its
%                output length no longer matches the input. The symbols
%                from such a call are meaningless, so this function
%                returns nothing and resets the loop; the caller should
%                discard whatever it had buffered and wait for the next
%                chunk. See the block comment above the check below.

persistent rxFilter symSync lastConfig

% comm.SymbolSynchronizer (and comm.RaisedCosineReceiveFilter) lock
% their expected input length on first call and error on a later call
% with a different length unless released first -- include it here so a
% chunk-to-chunk length change (e.g. from a partially-filled receive
% buffer) reconfigures instead of crashing. This does reset the Gardner
% loop's timing-lock state, but only on the chunks where it actually
% happens.
thisConfig = [sps, rolloff, filtSpan, numel(rxIn)];

if isempty(rxFilter) || ~isequal(thisConfig, lastConfig)
    if ~isempty(rxFilter)
        release(rxFilter);
        release(symSync);
    end
    rxFilter = comm.RaisedCosineReceiveFilter( ...
        RolloffFactor=rolloff, ...
        FilterSpanInSymbols=filtSpan, ...
        InputSamplesPerSymbol=sps, ...
        DecimationFactor=1);   % keep output at sps -- symbol synchronizer needs oversampled input

    symSync = comm.SymbolSynchronizer( ...
        TimingErrorDetector="Gardner (non-data-aided)", ...
        SamplesPerSymbol=sps);

    % comm.SymbolSynchronizer warns on every truncating call. We detect
    % that condition ourselves below and act on it, so the warning is pure
    % console noise -- and at hundreds of occurrences per run, formatting
    % and printing it is not free.
    warning('off', 'comm:SymbolSynchronizer:SymbolDropping');

    lastConfig = thisConfig;
end

filtOut = rxFilter(rxIn(:));
[symbolsOut, timingErr] = symSync(filtOut);

% LOSS-OF-LOCK CHECK.
%
% The Gardner loop has no restoring force when there is nothing real to
% track -- on noise, or across a discontinuity in the sample stream, its
% timing estimate random-walks instead of settling. Left alone it reaches
% one of two absorbing states, and it comes back from NEITHER on its own,
% not even once a clean signal returns:
%
%   HIGH RAIL: it decides sps -> 1 and asks for one output symbol per
%   input sample. comm.SymbolSynchronizer clamps the output at
%   MaxOutputExpansionFactor = 11/10 of nominal and silently drops the
%   rest. Seen on hardware: a 66564-sample chunk (33282 QPSK symbols at
%   2 sps) capped at 36611, dropping exactly 66564 - 36611 = 29953 -- one
%   per input sample, the signature of full runaway.
%
%   LOW RAIL: it decides sps is very large and emits almost nothing.
%   Measured decay over consecutive noise chunks: 21648, 7915, 2879, 1097,
%   497, 235, 22, 4, 0, 0. This one is self-reinforcing for a clear
%   reason -- the TED only produces an error sample when a symbol is
%   emitted, so a loop that has stopped emitting has nothing left to
%   correct itself with.
%
% Hence a TWO-SIDED test on output length rather than just the cap. The
% band is wide because a locked loop is extremely consistent: measured
% 33281-33283 against a nominal 33282, and even a 20 ppm sample-clock
% difference between two unsynchronised USRPs moves it by well under one
% symbol. Anything 8% off nominal is the loop failing, not the link.
nominalOutputLen = numel(filtOut) / sps;
lockTolerance = 0.08;

lostLock = abs(numel(symbolsOut) - nominalOutputLen) > lockTolerance * nominalOutputLen;

if lostLock
    % Symbols from a loop in this state are garbage at an unknown sample
    % phase -- worse than nothing, since returning them would only poison
    % the caller's frame-search buffer. Reset so the NEXT chunk starts
    % re-acquiring from scratch, which is what actually restores lock.
    %
    % Only symSync is reset. The RRC filter's state is legitimate sample
    % history that stays valid across the discarded chunk, so resetting it
    % would introduce a needless transient into the following one.
    reset(symSync);
    symbolsOut = symbolsOut([]);   % empty, preserving class/complexity
    timingErr = timingErr([]);
    return;
end

% comm.RaisedCosineReceiveFilter normalizes its own coefficients to unit
% energy independently of the matched transmit filter; paired together,
% the combined response gains the received symbol amplitude by
% sqrt(sps) relative to the actual transmitted symbol power. Correct
% that here so every downstream consumer (frame sync correlation,
% PLHEADER decode, pilot-based SNR estimation, bit recovery) receives
% symbols on the same amplitude scale the transmitted constellation
% actually uses.
symbolsOut = symbolsOut / sqrt(sps);

end