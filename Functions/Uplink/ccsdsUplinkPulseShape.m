function samples = ccsdsUplinkPulseShape(symbols, config)
%CCSDSUPLINKPULSESHAPE Root-raised-cosine shaping for the continuous uplink.
%
%   samples = ccsdsUplinkPulseShape(symbols, config) upsamples a block of
%   BPSK symbols by samplesPerSymbol, filters it with a root-raised-cosine,
%   and scales the result to config.uplink.txRMS.
%
%   FILTER STATE IS CARRIED ACROSS CALLS, which is the whole reason this is
%   its own function. Call `clear ccsdsUplinkPulseShape` to reset it between
%   runs.
%
%   WHY THE STATE MATTERS SO MUCH HERE. Under PLOP-2 the transmitter emits
%   one unbroken stream, handed to the radio in blocks. The RRC impulse
%   response is 251 samples long -- ten symbols -- so every symbol's energy
%   spreads well past its own block boundary. Filtering each block
%   independently would chop those tails off and restart them, putting a
%   discontinuity at every single block edge: spectral splatter outside the
%   channel, and an amplitude notch the receiver's matched filter cannot
%   undo. Carrying the state makes the output identical to filtering the
%   entire session in one go.
%
%   This is the transmit twin of ccsdsUplinkMatchedFilter.m. The cascade of
%   the two is a full raised cosine, which is zero at every symbol instant
%   but its own -- so given correct sampling phase, symbols do not interfere
%   with each other. Splitting the shaping across the two ends is what makes
%   the receive filter also the MATCHED filter, and only the matched filter
%   maximises SNR at the decision instant.
%
%   WHY RMS AND NOT PEAK. The old burst transmitter normalised each burst to
%   a peak of 0.7. A continuous stream has no end to take a maximum over, and
%   normalising per block would make the gain jump about from block to block
%   -- an amplitude modulation the receiver would have to track for no
%   reason. Setting a fixed RMS instead gives a constant, predictable drive
%   level, so the radio's gain setting means the same thing at every instant.
%
%   The consequence to be aware of: this waveform is NOT constant envelope.
%   Root-raised-cosine shaping gives it roughly 4 dB of peak-to-average with
%   occasional larger excursions, so at the default 0.3 RMS typical peaks sit
%   near 0.48 and rare ones near 0.75 -- inside the +-1 the radio accepts,
%   with headroom deliberately left for the tail of the distribution.

    persistent h zi lastKey

    sps  = config.uplink.samplesPerSymbol;
    span = config.uplink.filterSpanSymbols;
    beta = config.uplink.rolloffFactor;

    thisKey = [sps, span, beta];
    if isempty(h) || ~isequal(thisKey, lastKey)
        h = rcosdesign(beta, span, sps, 'sqrt');
        % Unit energy makes the filter's noise behaviour predictable; the
        % sqrt(sps) puts the OUTPUT at unit RMS for +-1 symbols, because
        % upsampling by sps spreads each symbol's energy over sps samples.
        h = h(:) / sqrt(sum(h.^2)) * sqrt(sps);
        zi = zeros(numel(h)-1, 1);
        lastKey = thisKey;
    end

    symbols = real(symbols(:));
    if isempty(symbols)
        samples = complex(zeros(0,1));
        return;
    end

    % Upsample: one symbol, then sps-1 zeros. The filter turns each of those
    % impulses into a pulse, and overlapping pulses sum -- which is what
    % pulse shaping is.
    up = zeros(numel(symbols)*sps, 1);
    up(1:sps:end) = symbols;

    [y, zi] = filter(h, 1, up, zi);

    samples = complex(config.uplink.txRMS * y);
end
