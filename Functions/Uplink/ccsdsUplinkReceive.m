function [commands, info] = ccsdsUplinkReceive(newSamples, config)
%CCSDSUPLINKRECEIVE The whole CCSDS Telecommand uplink receiver.
%
%   [commands, info] = ccsdsUplinkReceive(newSamples, config) accepts one
%   read from the uplink radio, adds it to an internal buffer, and returns
%   every command recovered from it as a cell array of structs from
%   ccsdsUplinkParseCommand.m ("report", "request" or "unknown").
%
%   Call `clear ccsdsUplinkReceive ccsdsUplinkDCSuppress` to reset between
%   runs; both hold state across calls.
%
%   THE CHAIN, and the file that owns each stage:
%
%     1  DC suppression         ccsdsUplinkDCSuppress.m     (streaming)
%     2  coarse carrier         ccsdsUplinkCoarseCFO.m      (per window)
%     3  RRC matched filter     ccsdsUplinkMatchedFilter.m
%    3b  Gardner timing loop    ccsdsUplinkGardner.m        (OPTIONAL, off)
%     4  start-sequence search  ccsdsUplinkASMDetect.m
%     5  Costas loop            ccsdsUplinkCostas.m
%     6  derandomize        \_  ccsdsUplinkDecodeCodeblock.m
%     7  LDPC decode        /
%     8  command recovery       ccsdsUplinkParseCommand.m
%
%   Stage 1 runs on the STREAM, before buffering, because its filter state
%   has to be continuous -- restarting it per window would put a transient
%   at the top of each one. Everything after it runs per window, on a copy.
%
%   TWO GATES, IN COST ORDER. Stage 2 is one FFT and stages 3-7 are perhaps
%   fifty times that, so the squared-spectrum peak decides whether the rest
%   of the chain runs at all. It is a weak test on its own -- it is really
%   asking "is there any BPSK-shaped thing in this window" -- and it is
%   meant to be: the strong test is the start-sequence correlation in stage
%   4, backed by LDPC parity in stage 7. Set config.uplink.detectThresholdDB
%   loosely enough that no real burst is thrown away here, since a burst
%   rejected at stage 2 is never seen again, while a noise window that gets
%   through merely wastes a few milliseconds.
%
%   WHY THE WINDOWS OVERLAP. Stage 4 finds the burst inside the window by
%   itself, so a window only has to CONTAIN a whole CLTU, not be aligned to
%   it. The requirement is therefore just
%
%       searchWindowSamples - searchStrideSamples >= the CLTU's span
%
%   which guarantees that for any arrival time, some window holds the whole
%   burst. At the configured 12000 and 4000 against a 7976-sample CLTU there
%   are 24 samples to spare, and the check below enforces it rather than
%   trusting that arithmetic.
%
%   AND WHY THE STEP NEVER CHANGES, even after a decode. An earlier version
%   skipped a whole window after a successful decode, to avoid reporting the
%   same burst from both windows that contain it -- and lost every burst
%   that began inside the window it skipped. Duplicates are suppressed by
%   POSITION instead: two detections within half a burst length of each
%   other in the sample stream are the same transmission. Position, not
%   payload -- the transmitter is perfectly entitled to send the same
%   command twice, and comparing bytes would silently swallow the repeat.

    persistent buf streamPos recentPos

    u = config.uplink;
    W = u.searchWindowSamples;
    stride = u.searchStrideSamples;

    commands = {};
    info = struct('windows', 0, 'detections', 0, 'asmLocks', 0, ...
        'decodes', 0, 'parityFails', 0, 'duplicates', 0, ...
        'bestCarrierDB', -Inf, 'bestASMMetric', 0, 'offsetHz', NaN, ...
        'esNodB', NaN, 'residualHz', NaN, 'tailMetric', NaN, ...
        'dcLevel', 0, 'buffered', 0, 'payloads', {{}});

    % A window that cannot hold a whole CLTU never decodes anything, and
    % fails silently -- stage 4 simply never finds a peak. Checked here
    % rather than trusted to the arithmetic in the config, because the CLTU
    % length follows from the coding scheme and codeword length.
    fr = ccsdsUplinkFraming(config);
    cltuSpan = (fr.cltuSymbols - 1)*u.samplesPerSymbol + 1;
    if W - stride < cltuSpan
        error('ccsdsUplinkReceive:WindowTooSmall', ...
            ['searchWindowSamples (%d) minus searchStrideSamples (%d) is ' ...
             '%d, below the %d samples a %d-symbol CLTU spans. A burst ' ...
             'arriving between two windows would be cut in half by both.'], ...
            W, stride, W - stride, cltuSpan, fr.cltuSymbols);
    end

    if isempty(buf)
        buf = complex(zeros(0,1));
        streamPos = 0;
        recentPos = [];
    end

    %% Stage 1 -- on the stream, before anything else sees it
    [clean, dcLevel] = ccsdsUplinkDCSuppress(newSamples, config);
    info.dcLevel = dcLevel;
    buf = [buf; clean];

    while numel(buf) >= W
        win = buf(1:W);
        info.windows = info.windows + 1;
        winPos = streamPos;                 % stream index of win(1)

        %% Stage 2 -- coarse carrier, and the cheap detection gate
        [fEst, carrierDB] = ccsdsUplinkCoarseCFO(win, config);
        info.bestCarrierDB = max(info.bestCarrierDB, carrierDB);

        if carrierDB >= u.detectThresholdDB
            info.detections = info.detections + 1;

            n = (0:numel(win)-1).';
            derot = win .* exp(-1j*2*pi*fEst*n/u.sampleRate);

            %% Stage 3 -- matched filter
            mf = ccsdsUplinkMatchedFilter(derot, config);

            %% Stage 3b -- optional Gardner timing loop (off by default)
            if u.timingSyncEnabled
                mf = ccsdsUplinkGardner(mf, config);
                spsNow = 2;
            else
                spsNow = u.samplesPerSymbol;
            end

            %% Stage 4 -- start sequence: framing, timing, phase, sign
            det = ccsdsUplinkASMDetect(mf, config, spsNow);
            info.bestASMMetric = max(info.bestASMMetric, det.metric);

            if det.found
                info.asmLocks = info.asmLocks + 1;
                % det.index counts samples in whatever rate stage 4 saw, so
                % it has to be scaled back to the stream's own rate before
                % it can be compared with a position in the stream.
                absPos = winPos + det.index * (u.samplesPerSymbol/spsNow);

                if localSeenAt(absPos, recentPos, u.burstSamples/2)
                    info.duplicates = info.duplicates + 1;
                else
                    recentPos(end+1) = absPos; %#ok<AGROW>

                    %% Stage 5 -- Costas loop over the burst
                    [tracked, costas] = ccsdsUplinkCostas(det.symbols, config);

                    %% Stages 6 and 7 -- derandomize, LDPC decode
                    codeSyms = tracked(fr.codewordOffset + (1:fr.codewordLength));
                    [payload, ok, ~] = ccsdsUplinkDecodeCodeblock( ...
                        codeSyms, det.amplitude, det.noiseVar, config);

                    info.offsetHz = fEst;
                    info.esNodB = det.esNodB;
                    info.residualHz = costas.residualHz;
                    info.tailMetric = det.tailMetric;

                    if ok
                        info.decodes = info.decodes + 1;
                        info.payloads{end+1} = payload;
                        %% Stage 8 -- command recovery
                        commands{end+1} = ccsdsUplinkParseCommand(payload); %#ok<AGROW>
                    else
                        info.parityFails = info.parityFails + 1;
                    end
                end
            end
        end

        buf = buf(stride+1:end);
        streamPos = streamPos + stride;
    end

    % Forget detections that are far enough back that no window still
    % overlaps them, so the list cannot grow without bound.
    if ~isempty(recentPos)
        recentPos = recentPos(recentPos > streamPos - 4*W);
    end

    info.buffered = numel(buf);
end

function tf = localSeenAt(pos, recentPos, tol)
    tf = ~isempty(recentPos) && any(abs(recentPos - pos) < tol);
end
