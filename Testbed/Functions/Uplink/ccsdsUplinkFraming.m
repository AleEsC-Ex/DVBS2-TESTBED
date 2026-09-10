function fr = ccsdsUplinkFraming(config)
%CCSDSUPLINKFRAMING Everything the receiver must know about the CLTU on the air.
%
%   fr = ccsdsUplinkFraming(config) returns the fixed CCSDS Telecommand
%   constants that the receiver needs in order to find, align and decode a
%   burst produced by ccsdsUplinkBurst.m.
%
%   fields:
%     .startBits      start sequence (ASM), as 0/1
%     .startSymbols   the same, BPSK-mapped to -1/+1
%     .tailBits       tail sequence, as 0/1
%     .tailSymbols    the same, BPSK-mapped
%     .codewordLength bits in one codeblock on the air
%     .infoLength     information bits it carries
%     .randomizer     the randomizing sequence, codewordLength bits
%     .cltuSymbols    total CLTU length in symbols
%     .codewordOffset symbols from the start of the CLTU to the codeblock
%
%   THIS IS THE RECEIVER'S HALF OF ccsdsUplinkTCConfig.m. That function
%   builds the object the transmitter hands to the toolbox; this one states
%   the same waveform in the terms a receiver works in -- a correlation
%   template, a bit count, a randomizing sequence. Both are derived from the
%   same config fields, so the two ends cannot drift apart.
%
%   THE CONSTANTS ARE NOT GUESSES. Every one of them was read back out of
%   ccsdsTCWaveform itself and is re-verified against it by UplinkRxTest.m,
%   which regenerates a burst and checks the start sequence, tail,
%   randomizer and codeword geometry bit for bit. That matters because a
%   wrong constant here does not produce an error -- it produces a receiver
%   that decodes nothing, which is indistinguishable from a dead radio.
%
%   BPSK MAPPING: bit 1 -> +1, bit 0 -> -1. Confirmed by generating a
%   waveform and reading the start sequence back as 034776C7272895B0.

    persistent cached key

    u = config.uplink;
    thisKey = string(u.channelCoding) + "/" + string(u.ldpcCodewordLength);
    if ~isempty(cached) && isequal(key, thisKey)
        fr = cached;
        return;
    end

    if string(u.channelCoding) ~= "LDPC"
        error('ccsdsUplinkFraming:UnsupportedCoding', ...
            ['This receiver implements LDPC only, and config.uplink.' ...
             'channelCoding is "%s". BCH would need its own start ' ...
             'sequence (EB90), tail and decoder; it is deliberately not ' ...
             'half-implemented here.'], u.channelCoding);
    end

    n = u.ldpcCodewordLength;
    if n ~= 128 && n ~= 512
        error('ccsdsUplinkFraming:BadCodewordLength', ...
            'config.uplink.ldpcCodewordLength must be 128 or 512, not %d.', n);
    end

    % CCSDS 231.0-B start sequence for the LDPC codes: 64 bits, chosen for
    % its autocorrelation rather than for any structural reason. This is the
    % ONLY thing in the burst the receiver knows in advance, so it carries
    % the whole of acquisition: burst position, symbol timing, carrier phase
    % and the BPSK pi ambiguity all come out of correlating against it.
    startBits = localHexToBits('034776C7272895B0');

    % Tail sequence, deliberately NOT a valid codeword, so a decoder cannot
    % mistake it for data. 128 bits for both LDPC codeword lengths.
    tailBits = localHexToBits('55555556AAAAAAAA5555555555555555');

    fr = struct();
    fr.startBits      = startBits;
    fr.startSymbols   = 2*startBits - 1;
    fr.tailBits       = tailBits;
    fr.tailSymbols    = 2*tailBits - 1;
    fr.codewordLength = n;
    fr.infoLength     = n/2;
    fr.randomizer     = ccsdsUplinkRandomizer(n);
    fr.codewordOffset = numel(startBits);
    fr.cltuSymbols    = numel(startBits) + n + numel(tailBits);

    cached = fr;
    key = thisKey;
end

function bits = localHexToBits(h)
% One hex digit at a time, most significant bit first. Written with bitget
% rather than de2bi so it carries no Communications Toolbox dependency of
% its own -- this file is the receiver's list of constants and should stay
% readable with nothing loaded.
    v = sscanf(h, '%1x');
    bits = reshape(double(bitget(repmat(v, 1, 4), repmat(4:-1:1, numel(v), 1))).', [], 1);
end
