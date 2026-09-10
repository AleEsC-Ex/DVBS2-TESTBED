function [payloadBytes, ok, info] = ccsdsUplinkDecodeCodeblock(symbols, amplitude, noiseVar, config)
%CCSDSUPLINKDECODECODEBLOCK Stages 6 and 7 -- derandomize, then LDPC decode.
%
%   [payloadBytes, ok, info] = ccsdsUplinkDecodeCodeblock(symbols,
%   amplitude, noiseVar, config) takes the codeblock's worth of derotated
%   symbols sliced out of a CLTU and returns the information bytes it
%   carries. OK is the LDPC parity check: true means every one of the 64
%   parity equations is satisfied.
%
%   info fields: .iterations, .parityViolations, .llrRange, .infoBits.
%
%   THE ORDER OF THE TWO STAGES IS NOT INTERCHANGEABLE, and it is worth
%   saying why because the intuitive order is the wrong one. CCSDS applies
%   the randomizer AFTER encoding, to the finished codeblock, so what is on
%   the air is (codeword XOR randomizer) and that is NOT itself a codeword.
%   Feed it to the decoder first and every parity equation fails. Measured
%   directly: H*(received)' is non-zero, H*(received XOR randomizer)' is
%   zero. Derandomizing first is therefore mandatory, which is exactly the
%   order in the receiver spec.
%
%   DERANDOMIZING IN THE LLR DOMAIN, NOT THE BIT DOMAIN. Undoing a XOR on
%   hard bits would mean slicing first and throwing away the soft
%   information the LDPC decoder lives on -- rate 1/2 LDPC gains something
%   like 2 dB from soft input, so that would give away most of the reason
%   for choosing LDPC. Flipping the SIGN of an LLR is the same operation on
%   a soft value: it says "the bit you thought was probably 1 is probably
%   0", with the confidence intact.
%
%   LLR SCALING IS LOAD-BEARING, AND THE OLD CHAIN GOT IT WRONG. The
%   decoder needs log(P(0)/P(1)) = -2*a*r/sigma^2, so it needs the amplitude
%   AND the noise variance, both in the same units as the symbols. Get the
%   scale wrong and belief propagation is either starved (over-cautious
%   LLRs, it cannot converge) or over-confident (it converges on the wrong
%   codeword). The previous receiver passed a variance measured somewhere
%   else in the chain and its LDPC mode measured WORSE than BCH, which is
%   backwards for a rate-1/2 code against a 56-of-64 one. Here both numbers
%   come from ccsdsUplinkASMDetect.m, measured on the start sequence of this
%   same burst through this same filter.
%
%   THE SIGN. This link maps bit 1 to +1, so a POSITIVE symbol means bit 1
%   means a NEGATIVE log-likelihood ratio. Getting this backwards inverts
%   every bit and decodes nothing at all.
%
%   OK IS THE ONLY REAL VALIDITY GATE ON THIS LINK. There is no CRC in a
%   CLTU. What there is instead is 64 parity equations, and a false
%   detection satisfying all of them by chance is a 2^-64 event. So a
%   parity-clean decode can be acted on, and a parity-failed one discarded,
%   with no payload-level sanity checking needed at all.

    fr = ccsdsUplinkFraming(config);
    [~, decCfg] = ccsdsUplinkLDPCMatrix(fr.codewordLength);

    % payloadBytes stays empty unless the parity check passes, so a caller
    % that ignores OK still cannot act on an unverified codeblock.
    payloadBytes = uint8([]);
    info = struct('iterations', 0, 'parityViolations', fr.codewordLength/2, ...
        'llrRange', 0, 'infoBits', zeros(0,1));

    symbols = symbols(:);
    if numel(symbols) ~= fr.codewordLength
        error('ccsdsUplinkDecodeCodeblock:WrongLength', ...
            'Expected %d codeblock symbols, got %d.', ...
            fr.codewordLength, numel(symbols));
    end

    r = real(symbols);
    llr = -2 * amplitude * r / max(noiseVar, eps);

    % Derandomize: flip the sign wherever the randomizer bit is 1.
    llr = llr .* (1 - 2*fr.randomizer);

    % Clip before decoding. A symbol that happens to land far out combined
    % with a small variance estimate produces an enormous LLR, and one
    % infinitely-confident wrong bit can drag belief propagation into a
    % confident wrong answer that still fails parity -- wasting the burst.
    % Clipping costs nothing on correct bits, which are nowhere near it.
    llr = max(min(llr, 30), -30);

    [bits, iters, finalChecks] = ldpcDecode(llr, decCfg, ...
        config.uplink.ldpcMaxIterations);

    info.iterations = iters;
    info.parityViolations = sum(finalChecks ~= 0);
    info.llrRange = max(abs(llr));
    info.infoBits = double(bits(:));

    ok = info.parityViolations == 0;
    if ok
        payloadBytes = dvbs2PackBits(info.infoBits);
    end
end
