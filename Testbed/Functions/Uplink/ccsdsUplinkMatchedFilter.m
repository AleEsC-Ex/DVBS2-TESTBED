function y = ccsdsUplinkMatchedFilter(x, config)
%CCSDSUPLINKMATCHEDFILTER Stage 3 -- root-raised-cosine matched filter.
%
%   y = ccsdsUplinkMatchedFilter(x, config) filters a window of complex
%   baseband samples with the same root-raised-cosine response the
%   transmitter shaped with, and compensates the filter's group delay so
%   y(n) lines up with x(n).
%
%   The cascade of the transmitter's RRC and this one is a full raised
%   cosine, which is zero at every symbol instant but its own -- so provided
%   the sampling phase is right, the symbols do not interfere with each
%   other. That is the reason for splitting the shaping across the two ends
%   rather than doing it all at the transmitter: only this way is the
%   receive filter also the MATCHED filter, and only the matched filter
%   maximises SNR at the decision instant.
%
%   Worth being concrete about how much that is: it rejects everything
%   outside the 10.8 kHz occupied bandwidth of a 200 kHz sampled stream,
%   which is 12.7 dB of noise power. Every stage after this one sees a
%   signal 12.7 dB cleaner than the radio delivered.
%
%   IT MUST COME AFTER THE COARSE FREQUENCY CORRECTION AND BEFORE THE
%   COSTAS LOOP, and both halves of that are load-bearing. Before: a burst
%   offset by 25 kHz lies outside this filter's passband and would simply be
%   deleted. After: the Costas loop's phase detector works on whatever noise
%   it is given, and giving it the 12.7 dB cleaner stream is the difference
%   between a loop that tracks and one that wanders.
%
%   Implemented with conv and a fixed coefficient vector rather than
%   comm.RaisedCosineReceiveFilter, because a System object locks its input
%   length on first call and the windows here are not all the same size --
%   the last one before the buffer runs dry is short. Releasing per call to
%   work around that costs more than the filter does.

    persistent h lastKey

    sps  = config.uplink.samplesPerSymbol;
    span = config.uplink.filterSpanSymbols;
    beta = config.uplink.rolloffFactor;

    thisKey = [sps, span, beta];
    if isempty(h) || ~isequal(thisKey, lastKey)
        h = rcosdesign(beta, span, sps, 'sqrt');
        h = h(:) / sqrt(sum(h.^2));   % unit energy, so noise variance is preserved
        lastKey = thisKey;
    end

    x = complex(x(:));
    if numel(x) < numel(h)
        y = complex(zeros(size(x)));
        return;
    end

    d = (numel(h) - 1)/2;             % h has odd length span*sps+1
    yFull = conv(x, h);
    y = yFull(d+1 : d+numel(x));
end
