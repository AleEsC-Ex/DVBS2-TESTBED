function [snrDB, noiseVarEstimate, snrInfo] = dvbs2SNREstimate(plSymbols_corrected, fp, refHeader, config)
%DVBS2SNRESTIMATE Cross-checked per-frame SNR for ACM and LDPC LLR scaling.
%
%   [snrDB, noiseVarEstimate, snrInfo] = ...
%       dvbs2SNREstimate(plSymbols_corrected, fp, refHeader, config)
%
%   Measures the SNR of ONE corrected PLFRAME twice, against two
%   independent known-symbol references, and reports whether they agree.
%
% HOW A KNOWN-SYMBOL SNR ESTIMATE WORKS
%
%   Some symbols in a PLFRAME are not data: the transmitter is obliged to
%   send exact, specified values for them. The pilots are one such set,
%   the PLHEADER is another. For those symbols the receiver knows what
%   was sent, so it can subtract it:
%
%       residual = rxSymbol - refSymbol
%
%   In a noiseless, perfectly corrected receiver the residual is zero.
%   Everything that is left is impairment -- thermal noise, phase error,
%   timing error, quantisation. Averaging its squared magnitude over N
%   such symbols estimates the noise power directly:
%
%       noiseVar    = mean(|rx - ref|^2)
%       signalPower = mean(|ref|^2)             (= 1; both sets are unit modulus)
%       SNR         = signalPower / noiseVar
%
%   No training sequence is transmitted for this. The symbols were going
%   to be sent anyway; knowing them is what makes them usable as a ruler.
%
% WHY AVERAGING, AND WHAT IT DOES NOT DO
%
%   Each individual residual is one draw from a random process, so a
%   single symbol tells you almost nothing. The relative standard error
%   of the averaged noise power falls as 1/sqrt(N):
%
%       N =   26 (SOF only)      -> 19.6%  -> +-0.78 dB
%       N =   90 (full PLHEADER) -> 10.5%  -> +-0.44 dB
%       N =  792 (22 pilot blocks) -> 3.6% -> +-0.15 dB
%
%   Averaging suppresses RANDOM scatter. It does NOT protect against a
%   corrupted symbol -- a mean is exactly what one wild sample ruins, and
%   with 792 terms a single symbol 100x too large still moves the answer.
%   Robustness against a bad GROUP is a separate job, done below by the
%   median across pilot blocks. Two different problems, two different
%   tools.
%
% WHY TWO REFERENCES
%
%   The pilot estimate is the more precise, but it is not independent of
%   the receive chain: fp comes from the decoded PLSC, and the 22 pilot
%   blocks are spread across the full 99.8 ms frame, so any residual
%   frequency error rotates the later blocks away from their reference.
%   Measured on hardware: six consecutive frames reporting -1.65 to
%   -0.33 dB on a 16 dB link, with healthy frame sync and a confident
%   header decode. That range is not noise. If rx and ref are completely
%   uncorrelated but carry equal power P,
%
%       E|rx - ref|^2 = P + P - 0 = 2P    ->    SNR -> -3.01 dB
%
%   and every bad reading observed sits between -3 and 0 dB: the
%   estimator working correctly on a broken reference.
%
%   The PLHEADER cannot fail that way. It spans 90 symbols in 270 us
%   rather than 99.8 ms, so it accumulates ~370x less phase rotation for
%   the same frequency error, and it needs no pilot layout at all. It is
%   the less precise reference and the more reliable one -- which is why
%   they are compared rather than chosen between.
%
% Inputs:
%   plSymbols_corrected - Column vector, one full frequency/phase
%                         corrected PLFRAME (header + payload + pilots).
%   fp                  - Pilot structure from dvbs2PilotStructure.m.
%   refHeader           - 90x1 ideal PLHEADER from
%                         dvbs2PLHeaderReference.m. Pass [] to skip the
%                         cross-check and use pilots alone.
%   config              - Reads config.snr.maxDisagreementDB. Optional.
%
% Outputs:
%   snrDB            - The arbitrated SNR to report upstream.
%   noiseVarEstimate - Matching noise variance. Pass this as nVar to
%                      AEC_dvbs2BitRecover.m.
%   snrInfo          - Diagnostics: .PilotSNRdB, .HeaderSNRdB,
%                      .BlockSNRdB (per pilot block), .BlockTrendDB
%                      (least-squares slope across blocks, dB per block),
%                      .DisagreementDB, .Trusted, .Source.

    if nargin < 3, refHeader = []; end
    if nargin < 4, config = struct; end

    maxDisagreementDB = 3;
    if isfield(config, 'snr') && isfield(config.snr, 'maxDisagreementDB')
        maxDisagreementDB = config.snr.maxDisagreementDB;
    end

    snrInfo = struct('PilotSNRdB', NaN, 'HeaderSNRdB', NaN, ...
        'BlockSNRdB', [], 'BlockTrendDB', NaN, 'DisagreementDB', NaN, ...
        'Trusted', true, 'Source', "pilots", 'HeaderPhaseDeg', NaN);

    %% ---- Reference 1: the pilots, measured PER BLOCK ----------------
    %
    % The 792 pilot symbols arrive as 22 separate 36-symbol blocks spaced
    % evenly through the frame. Collapsing all of them into one mean, as
    % this function used to, throws away WHERE in the frame each residual
    % came from. Reshaping to 36-by-22 costs nothing and recovers it:
    %
    %   flat scatter about a common mean  -> ordinary noise
    %   monotone rise or fall             -> something drifting across
    %                                        the frame (residual CFO is
    %                                        the obvious candidate)
    %   one block bad, the rest clean     -> an impulse or a burst
    %
    % and it allows a MEDIAN across blocks instead of a mean over
    % everything. The median discards a corrupted block rather than
    % averaging it in. That costs a little precision -- the median of 22
    % values is ~64% as efficient as their mean, so +-0.19 dB instead of
    % +-0.15 dB -- and buys immunity to exactly the failure mode a mean
    % has none against.
    pilotNoiseVar = NaN;
    if ~isempty(fp.pilotInd)
        rxPilots = plSymbols_corrected(fp.pilotInd);
        residual = rxPilots - fp.refPilots;

        nBlocks = fp.numPilotBlocks;
        blockNoiseVar = mean(abs(reshape(residual, 36, nBlocks)).^2, 1).';

        signalPower = mean(abs(fp.refPilots).^2);
        snrInfo.BlockSNRdB = 10*log10(signalPower ./ max(blockNoiseVar, eps));

        pilotNoiseVar = median(blockNoiseVar);
        snrInfo.PilotSNRdB = 10*log10(signalPower / max(pilotNoiseVar, eps));

        % Least-squares slope of per-block SNR against block index, on a
        % centred abscissa so the intercept drops out. Reported in dB per
        % block; over 22 blocks a slope of -0.5 means the last block sits
        % ~10 dB below the first, which no stationary channel does.
        if nBlocks >= 3
            x = (1:nBlocks).' - (nBlocks + 1)/2;
            snrInfo.BlockTrendDB = sum(x .* snrInfo.BlockSNRdB) / sum(x.^2);
        end
    end

    %% ---- Reference 2: the PLHEADER ----------------------------------
    %
    % Measured on the SAME corrected symbols, so the two estimates differ
    % only in which reference they trust, not in what they are looking at.
    %
    % One caveat on independence: dvbs2PhaseCompensate uses the SOF as one
    % of its 23 fit anchors, so the first 26 symbols contribute about 3%
    % of the weight in a 2-parameter line fit that was then subtracted
    % from them. That is a fraction of a hundredth of a dB. The 64 PLSC
    % symbols are not anchors at all and are entirely unaffected.
    headerNoiseVar = NaN;
    if ~isempty(refHeader)
        refHeader = refHeader(:);
        rxHeader = plSymbols_corrected(1:numel(refHeader));

        % DEROTATE AGAINST THE HEADER'S OWN SOF FIRST.
        %
        % Without this the header estimate reads several dB low, and on some
        % frames 15 dB low. The cause is upstream: dvbs2PhaseCompensate fits
        % its phase line through 23 anchors -- the SOF plus 22 pilot blocks
        % -- weighted by reliability, so the SOF carries 26 of 818 total
        % weight, about 3%. And the first pilot block does not begin until
        % symbol 1531, while the header occupies symbols 1 to 90. The
        % header's correction is therefore an EXTRAPOLATION backwards off a
        % line defined almost entirely by data starting 1500 symbols later,
        % which makes it the least accurately corrected part of the frame.
        %
        % An EVM estimator cannot tell leftover phase from noise -- it
        % measures |rx - ref|^2 and calls all of it noise -- so that
        % extrapolation error reads as a lower SNR. Measured against the
        % observed values: ~10 degrees of leftover phase explains a 17 dB
        % link reporting 13 dB, and ~46 degrees explains it reporting 2 dB.
        %
        % Confirmed on hardware by S3, which is the decisive evidence: frames
        % whose header estimate reported 1.89 dB decoded 38 of 38 packets at
        % QPSK 8/9, whose quasi-error-free point is 6.20 dB. A link 4.3 dB
        % below threshold cannot deliver zero errors, so the measurement was
        % wrong, not the link.
        %
        % The fix is that the header carries its own phase reference: the 26
        % SOF symbols are a fixed known sequence that depends on no decode
        % and no pilot layout. Measuring the offset from them and removing it
        % lets the header measure NOISE rather than inheriting another
        % stage's extrapolation error. Summing before taking the angle is the
        % same coherent average dvbs2PhaseCompensate uses on its anchors: the
        % signal parts add as N and the noise parts as sqrt(N), so the phase
        % estimate degrades gracefully instead of wrapping at low SNR.
        %
        % This models the residual as a CONSTANT offset, not a ramp. Across
        % 90 symbols (270 us) that is the right model. It costs one degree of
        % freedom out of 90, about 0.05 dB of optimistic bias.
        nSOF = 26;
        if numel(rxHeader) >= nSOF
            sofPhasor = sum(rxHeader(1:nSOF) .* conj(refHeader(1:nSOF)));
            if abs(sofPhasor) > 0
                snrInfo.HeaderPhaseDeg = rad2deg(angle(sofPhasor));
                rxHeader = rxHeader .* exp(-1j*angle(sofPhasor));
            end
        end

        headerNoiseVar = mean(abs(rxHeader - refHeader).^2);
        headerSignalPower = mean(abs(refHeader).^2);
        snrInfo.HeaderSNRdB = 10*log10(headerSignalPower / max(headerNoiseVar, eps));
    end

    %% ---- Combination: take the HIGHER of the two ----------------------
    %
    % Not a choice between them -- an argument that the higher reading is
    % always the better one.
    %
    % Both estimators compute mean(|rx - ref|^2) and call the result noise.
    % Every impairment that has not been perfectly corrected -- leftover
    % phase, leftover frequency, timing error, gain error -- ADDS to that
    % residual. Nothing can make it SMALLER than the true noise, because that
    % would require the impairment to cancel the noise, which does not
    % happen. So both estimators share one property:
    %
    %       THEY CAN ONLY EVER BE BIASED LOW.
    %
    % Neither can report a better SNR than the truth. The higher of the two
    % is therefore always the less corrupted one:
    %
    %   pilots corrupted (-1), header fine (17)   -> 17   correct
    %   header corrupted (12), pilots fine (17)   -> 17   correct
    %   genuine fade, both read 5                 ->  5   correct
    %
    % That last line is what makes this safe for ACM. A real fade raises the
    % residual against BOTH references, so the maximum follows it down. This
    % cannot mask a fade; it can only reject a corruption that hits one
    % reference and not the other.
    %
    % An earlier version picked the header whenever the two disagreed, on the
    % theory that the header was the robust reference. On hardware the
    % opposite was true 126 times in 356 frames, so that rule fed fabricated
    % values straight to the ACM policy. The maximum is right whichever
    % estimator is misbehaving, including failure modes not yet seen.
    %
    % The disagreement is still measured and still reported, but purely as a
    % diagnostic now -- it no longer decides anything.
    havePilots = ~isnan(pilotNoiseVar);
    haveHeader = ~isnan(headerNoiseVar);

    if havePilots && haveHeader
        snrInfo.DisagreementDB = abs(snrInfo.HeaderSNRdB - snrInfo.PilotSNRdB);
        snrInfo.Trusted = snrInfo.DisagreementDB <= maxDisagreementDB;

        if snrInfo.PilotSNRdB >= snrInfo.HeaderSNRdB
            snrDB = snrInfo.PilotSNRdB;
            noiseVarEstimate = pilotNoiseVar;
            snrInfo.Source = "pilots";
        else
            snrDB = snrInfo.HeaderSNRdB;
            noiseVarEstimate = headerNoiseVar;
            snrInfo.Source = "header";
        end

    elseif havePilots
        snrDB = snrInfo.PilotSNRdB;
        noiseVarEstimate = pilotNoiseVar;
        snrInfo.Source = "pilots";

    elseif haveHeader
        % No pilots in this frame. The PLHEADER carries the whole estimate,
        % and correctly so -- it is pi/2-BPSK regardless of the frame's own
        % modulation, which is what lets this branch work for 8PSK and the
        % APSK constellations. It replaces the previous fallback, which
        % sliced the payload against a hardcoded QPSK constellation and was
        % therefore wrong for every modulation except one.
        snrDB = snrInfo.HeaderSNRdB;
        noiseVarEstimate = headerNoiseVar;
        snrInfo.Source = "header";

    else
        error('dvbs2SNREstimate:NoReference', ...
            'Frame has neither pilots nor a PLHEADER reference to measure against.');
    end
end
