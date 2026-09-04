function seq = ccsdsUplinkRandomizer(n)
%CCSDSUPLINKRANDOMIZER The CCSDS Telecommand randomizing sequence.
%
%   seq = ccsdsUplinkRandomizer(n) returns the first N bits of the CCSDS
%   231.0-B randomizer as a column of 0/1 doubles.
%
%   h(x) = x^8 + x^6 + x^4 + x^3 + x^2 + x + 1, all-ones initial state,
%   which gives FF 39 9E 5A 68 E9 06 F5 ... -- the sequence printed in the
%   standard, and bit-for-bit what ccsdsTCWaveform produces. (Note this is
%   NOT the x^8+x^7+x^5+x^3+1 polynomial used by CCSDS TELEMETRY; the two
%   randomizers are different and swapping them silently destroys every
%   codeblock.)
%
%   WHAT IT IS FOR. The randomizer guarantees bit transitions on the air
%   whatever the payload, which is what lets the receiver's timing and
%   carrier recovery work on a run of identical command bytes. It is
%   mandatory with LDPC coding for exactly that reason, and MATLAB applies
%   it for LDPC whether or not ccsdsTCConfig.HasRandomizer is set.
%
%   IT IS APPLIED AFTER ENCODING, to the whole codeblock -- measured, not
%   assumed: the encoded-then-randomized block fails the parity check while
%   the derandomized one passes it. So the receiver must DERANDOMIZE BEFORE
%   DECODING, not after. In soft-decision terms that is a sign flip on every
%   LLR whose randomizer bit is 1, which costs nothing and keeps the soft
%   information intact -- see ccsdsUplinkDecodeCodeblock.m.

    persistent cachedSeq

    if ~isscalar(n) || n < 1 || n ~= floor(n)
        error('ccsdsUplinkRandomizer:BadLength', 'N must be a positive integer.');
    end

    % The generator is a fixed sequence, so extend the cache rather than
    % re-running the register on every call.
    if isempty(cachedSeq) || numel(cachedSeq) < n
        state = ones(1, 8);          % state(1) newest, state(8) oldest
        s = zeros(max(n, 512), 1);
        for i = 1:numel(s)
            s(i) = state(8);
            % Taps corresponding to h(x) above.
            fb = mod(state(2) + state(4) + state(5) + state(6) + ...
                     state(7) + state(8), 2);
            state = [fb, state(1:7)];
        end
        cachedSeq = s;
    end

    seq = cachedSeq(1:n);

    % Cheap self-check: the first byte of the sequence is FF and the second
    % is 39. A tap or shift-direction slip changes these immediately, and
    % would otherwise show up only as a receiver that never decodes.
    if n >= 16
        head = sprintf('%d', seq(1:16));
        if ~strcmp(head, '1111111100111001')
            error('ccsdsUplinkRandomizer:SelfCheckFailed', ...
                'Randomizer starts %s, expected 1111111100111001 (FF39).', head);
        end
    end
end
