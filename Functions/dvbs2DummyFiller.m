function filler = dvbs2DummyFiller(nSamples, sps, rolloff, filtSpan)
%DVBS2DUMMYFILLER Pulse-shaped dummy PLFRAMEs, for keeping the carrier fed.
%
%   filler = dvbs2DummyFiller(nSamples, sps, rolloff, filtSpan)
%
%   nSamples - at least this many samples are returned (a whole number of
%              dummy PLFRAMEs, so usually slightly more)
%   sps      - samples per symbol
%   rolloff  - RRC roll-off, matching the real transmit filter
%   filtSpan - RRC span in symbols, matching the real transmit filter
%
%   filler   - complex column vector, RRC-shaped, ready to push into the
%              transmit FIFO alongside real PLFRAMEs
%
%   WHAT A DUMMY PLFRAME IS. DVB-S2 defines one for exactly this situation:
%   the modulator has to keep transmitting but has no data ready. It is a
%   PLHEADER carrying PLS code 0 (MODCOD 0) followed by 36 slots of 90
%   scrambled QPSK symbols -- 90 + 3240 = 3330 symbols, always, whatever
%   MODCOD real traffic is using. It carries nothing, and a conforming
%   receiver identifies it from the header and skips it.
%
%   WHY THIS TESTBED NEEDS IT. LDPC encoding costs the same per frame at any
%   MODCOD (always 64800 bits), but a denser frame occupies LESS airtime, so
%   the generation duty cycle climbs as the ACM ladder is climbed. Measured
%   on hardware at 27.7 ms per frame:
%
%       QPSK     99.8 ms airtime/frame   28%
%       8PSK     66.6 ms                 42%
%       16APSK   50.1 ms                 55%
%       32APSK   40.0 ms                 69%
%
%   At 32APSK, generating real frames consumes 69% of the airtime they
%   occupy, and with the uplink receiver taking another ~28% there is
%   nothing left to feed the radio: TX underruns went from 6/126 bursts to
%   97/274 on the first full-ladder run. No scheduling trick fixes that,
%   because the work genuinely exceeds the time. The only way out is to do
%   LESS work, and a dummy frame is the cheap substitute -- no BBFRAME, no
%   BCH, no LDPC, no mapping. Throughput drops; the carrier, and therefore
%   the receiver's timing and carrier loops, stay up.
%
%   THE PRICE OF GETTING THE FILLER WRONG. It has to be spectrally
%   indistinguishable from real traffic. Transmitting zeros would drop the
%   carrier -- the very thing this exists to prevent. Transmitting a
%   repeating pattern would be worse than zeros: a periodic symbol sequence
%   puts discrete spectral lines into the signal, right where the
%   band-ratio detector looks and where the timing and CFO loops could lock
%   onto something false. So every dummy frame below gets INDEPENDENT random
%   QPSK, and the PL scrambler is applied exactly as it is to real payload.

    persistent cacheStream cacheKey

    dummySymbols = 90 + 36*90;          % 3330, fixed by the standard
    key = [sps, rolloff, filtSpan];

    nFramesNeeded = ceil(nSamples / (dummySymbols * sps));

    % Rebuild only when the geometry changes or a bigger run is asked for.
    % In steady state this is a slice of an already-filtered stream, which is
    % the entire point -- filler that cost as much as real frames would
    % defeat the exercise.
    if isempty(cacheStream) || ~isequal(key, cacheKey) || ...
            numel(cacheStream) < nFramesNeeded * dummySymbols * sps

        nFramesBuild = max(nFramesNeeded, 8);

        % PLS code 0. satcom.internal.dvbs.plHeader documents MODCOD as
        % [1,28] but accepts 0 and returns the correct 90-symbol,
        % unit-modulus header -- verified against the same decoder
        % dvbs2PLHeaderRecover calls, which reports IsDummyFrame for it.
        dummyHeader = satcom.internal.dvbs.plHeader('S2', 0, false, 64800);
        dummyHeader = dummyHeader(:);

        % PL scrambling restarts at each PLFRAME boundary, so every dummy
        % uses the same prefix of the sequence -- which is why the SYMBOLS
        % have to differ frame to frame, or the stream would be periodic.
        scramInt = satcom.internal.dvbs.plScramblingIntegerSequence(0);
        complexMap = [1; 1j; -1; -1j];
        scram = complexMap(scramInt(1:36*90) + 1);
        scram = scram(:);

        symbols = complex(zeros(nFramesBuild * dummySymbols, 1));
        for k = 0:nFramesBuild-1
            payload = pskmod(randi([0 3], 36*90, 1), 4, pi/4) .* scram;
            symbols(k*dummySymbols + (1:dummySymbols)) = [dummyHeader; payload];
        end

        % Filtered as ONE contiguous run so the joins between consecutive
        % dummies are seamless, for the same reason S1a no longer calls
        % flushFilter between real frames. Only the very end of the cached
        % stream carries a partial pulse, and that is where it is sliced.
        txFilt = comm.RaisedCosineTransmitFilter( ...
            RolloffFactor = rolloff, ...
            FilterSpanInSymbols = filtSpan, ...
            OutputSamplesPerSymbol = sps);
        cacheStream = txFilt(symbols);
        cacheKey = key;
    end

    filler = cacheStream(1 : nFramesNeeded * dummySymbols * sps);
end
