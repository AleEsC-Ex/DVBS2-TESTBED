function det = ccsdsUplinkASMDetect(y, config, spsIn)
%CCSDSUPLINKASMDETECT Stage 4 -- find the CLTU start sequence, and with it
%everything else acquisition needs.
%
%   det = ccsdsUplinkASMDetect(y, config) correlates a matched-filtered,
%   coarse-frequency-corrected window against the known 64-symbol CCSDS
%   start sequence and returns the best match.
%
%   det = ccsdsUplinkASMDetect(y, config, spsIn) says how many samples per
%   symbol Y carries, when that is not config.uplink.samplesPerSymbol --
%   which is the case when the optional Gardner timing loop is enabled, as
%   it resamples to 2. Note this also sets the resolution of the timing
%   this function recovers: at 25 samples per symbol the peak lands within
%   1/25 of a symbol, at 2 within a half.
%
%   det fields:
%     .found        whether the peak cleared config.uplink.asmThreshold
%     .metric       normalised correlation at the peak, 0 to 1
%     .index        sample index in Y of the first start-sequence symbol
%     .phaseRad     carrier phase measured at that point
%     .symbols      the whole CLTU, cltuSymbols complex values, derotated
%     .amplitude    signal amplitude estimated off the start sequence
%     .noiseVar     noise variance per real dimension, same source
%     .esNodB       the two combined, as an Es/No estimate
%     .tailMetric   an independent second opinion, see below
%
%   ONE CORRELATION SETTLES FOUR THINGS. This is why a burst-mode receiver
%   with a preamble is so much simpler than a continuous one, and why the
%   seven-stage chain has no timing-recovery stage in it:
%
%     WHERE the burst is    the index of the correlation peak
%     SYMBOL TIMING         the same index, to one sample -- a 25th of a
%                           symbol, which is why the correlation is run at
%                           the full sample rate instead of at symbol rate
%     CARRIER PHASE         the ANGLE of the complex peak
%     THE PI AMBIGUITY      also the angle. A suppressed-carrier BPSK
%                           receiver cannot tell a symbol from its negation
%                           -- squaring in stage 2 throws that away with the
%                           factor of two. Derotating by this angle forces
%                           the start sequence to come back positive, which
%                           is the only thing that can resolve it, because
%                           it is the only part of the burst whose value is
%                           known in advance.
%
%   A timing-recovery LOOP would be the wrong tool regardless: at 320
%   symbols a burst is over before a loop of any sensible bandwidth has
%   pulled in. Preamble correlation gets the answer in one shot and gets a
%   better one.
%
%   WHY THE METRIC IS NORMALISED BY LOCAL ENERGY. It makes the statistic
%   independent of gain, which matters because there is no AGC in front of
%   this and the level depends on antenna spacing on the day. The value is
%   then bounded and predictable: for a true burst it tends to
%   1/sqrt(1 + N0/Es), so 0.78 at 2 dB Es/No and 0.71 at 0 dB, while noise
%   peaks over a window this long sit near 0.31 to 0.39. A threshold of 0.5
%   separates them with margin at both ends.
%
%   THE TAIL IS A FREE SECOND OPINION. The CLTU ends with 128 more known
%   symbols, and they are checked too -- not as a gate (LDPC's own parity
%   check in stage 6 is a far stronger one) but because a high start metric
%   with a low tail metric is the signature of a burst that was found but
%   then lost, which is worth being able to see in a log.

    fr = ccsdsUplinkFraming(config);
    if nargin < 3 || isempty(spsIn)
        sps = config.uplink.samplesPerSymbol;
    else
        sps = spsIn;
    end

    det = struct('found', false, 'metric', 0, 'index', -1, 'phaseRad', 0, ...
        'symbols', complex(zeros(0,1)), 'amplitude', 0, 'noiseVar', Inf, ...
        'esNodB', -Inf, 'tailMetric', 0);

    y = complex(y(:));

    nASM  = numel(fr.startSymbols);
    nCLTU = fr.cltuSymbols;
    span  = (nCLTU - 1)*sps + 1;      % samples the whole CLTU occupies
    if numel(y) < span
        return;
    end

    % Zero-stuffed correlation template: the start-sequence symbols spaced
    % sps apart, zero in between. Correlating against this at every sample
    % offset is exactly "try every symbol phase", but as one convolution.
    kLen = (nASM - 1)*sps + 1;
    kern = zeros(kLen, 1);
    kern(1:sps:end) = fr.startSymbols;

    % Only offsets where the ENTIRE CLTU still fits in the window are
    % admissible -- a peak found near the end would give a truncated burst.
    lastStart = numel(y) - span + 1;

    num = conv(y, flipud(kern), 'valid');            % complex correlation
    den = conv(abs(y).^2, flipud(kern ~= 0), 'valid'); % energy at the same taps

    num = num(1:lastStart);
    den = den(1:lastStart);

    metric = abs(num) ./ sqrt(nASM * max(den, eps));
    [pk, idx] = max(metric);

    det.metric = pk;
    det.index = idx;
    det.phaseRad = angle(num(idx));

    if pk < config.uplink.asmThreshold
        return;
    end
    det.found = true;

    % Slice the whole CLTU at the winning phase and remove the measured
    % carrier phase. After this the start sequence is positive real, which
    % is what fixes the sign of every data symbol behind it.
    sym = y(idx + (0:nCLTU-1)*sps) * exp(-1j*det.phaseRad);
    det.symbols = sym;

    % Amplitude and noise, measured on the 64 symbols whose values are
    % known. Doing it here rather than from the silence between bursts is
    % deliberate: this estimate is taken through the identical filter chain,
    % at the identical instants, on the identical burst -- so it needs no
    % assumption about how the noise bandwidth relates to the signal's, and
    % it is immune to any gain change between the guard interval and the
    % burst.
    asmRx = real(sym(1:nASM));
    a = mean(asmRx .* fr.startSymbols);
    resid = sym(1:nASM) - a*fr.startSymbols;
    sigma2 = mean(abs(resid).^2) / 2;      % per real dimension

    det.amplitude = a;
    det.noiseVar = max(sigma2, eps);
    det.esNodB = 10*log10(max(a^2 / (2*det.noiseVar), eps));

    tailRx = real(sym(end-numel(fr.tailSymbols)+1:end));
    det.tailMetric = mean(sign(tailRx) == fr.tailSymbols);
end
