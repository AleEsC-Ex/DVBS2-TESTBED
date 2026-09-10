function symbols = ccsdsUplinkSymbols(kind, arg, config)
%CCSDSUPLINKSYMBOLS Generate one PLOP-2 element, as BPSK symbols.
%
%   symbols = ccsdsUplinkSymbols("acquisition", nSymbols, config)
%   symbols = ccsdsUplinkSymbols("idle",        nSymbols, config)
%   symbols = ccsdsUplinkSymbols("cltu",        payloadBytes, config)
%
%   Returns a real column of +1/-1, ONE SAMPLE PER SYMBOL. Pulse shaping and
%   rate conversion happen downstream in ccsdsUplinkPulseShape.m.
%
%   These three are everything PLOP-2 ever puts on the air:
%
%     acquisition   alternating symbols, once at the start of a session.
%                   Nothing but transitions, which is exactly what a carrier
%                   loop and a timing loop need in order to pull in.
%     idle          the same pattern, sent whenever no command is ready. It
%                   is what stops the carrier dropping between commands, and
%                   therefore what lets the receiver stay locked instead of
%                   re-acquiring every time.
%     cltu          the actual data: start sequence, codeblock, tail.
%
%   WHY ALTERNATING, AND NOT ZEROS OR A PN SEQUENCE. A run of identical
%   symbols is a constant, which a carrier loop cannot distinguish from a
%   phase offset and a timing loop cannot see edges in. Alternating symbols
%   put a transition at every single symbol boundary -- the densest possible
%   timing information -- and place the signal energy at a known offset from
%   the carrier rather than at DC. MATLAB enforces the pattern: passing
%   anything else to a "acquisition sequence" or "idle sequence" waveform
%   fails with "must be alternating ones and zeros with a starting bit of
%   either one or zero."
%
%   WHY THIS GOES THROUGH ccsdsTCWaveform AT ALL, when alternating +-1 is
%   two lines to write by hand. Because it makes the toolbox validate the
%   pattern for us, and because the CLTU branch has to use it anyway --
%   routing all three through one call means the acquisition, idle and data
%   symbols are guaranteed to share a mapping and an amplitude. Hand-rolling
%   two of the three is how the sequences end up subtly inconsistent with
%   the data they surround.

    kind = string(kind);

    switch kind
        case {"acquisition", "idle"}
            n = arg;
            if ~isscalar(n) || n < 1 || n ~= floor(n)
                error('ccsdsUplinkSymbols:BadLength', ...
                    'Sequence length must be a positive integer, got %s.', ...
                    mat2str(n));
            end
            if kind == "acquisition"
                fmt = "acquisition sequence";
            else
                fmt = "idle sequence";
            end
            % Alternating, starting from the configured bit. CCSDS allows
            % either starting value; they differ only by one symbol of
            % phase.
            bits = mod((0:n-1).' + config.uplink.plop.startBit, 2);
            w = ccsdsTCWaveform(bits, ccsdsUplinkTCConfig(config, fmt));

        case "cltu"
            payloadBytes = uint8(arg(:));
            if numel(payloadBytes) > config.uplink.maxPayloadBytes
                error('ccsdsUplinkSymbols:PayloadTooLong', ...
                    'Payload is %d bytes; config.uplink.maxPayloadBytes is %d.', ...
                    numel(payloadBytes), config.uplink.maxPayloadBytes);
            end
            w = ccsdsTCWaveform(dvbs2UnpackBits(payloadBytes), ...
                ccsdsUplinkTCConfig(config, "CLTU"));

        otherwise
            error('ccsdsUplinkSymbols:UnknownKind', ...
                'Kind must be "acquisition", "idle" or "cltu", not "%s".', kind);
    end

    % ccsdsTCWaveform hands back a complex column for BPSK even though the
    % constellation is real. Take the real part here so every downstream
    % stage is working with unambiguous +-1, and so the pulse shaper does
    % half the arithmetic.
    symbols = real(w(:));
end
