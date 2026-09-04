function [y, info] = ccsdsUplinkGardner(x, config)
%CCSDSUPLINKGARDNER Optional timing-recovery loop, between stages 3 and 4.
%
%   [y, info] = ccsdsUplinkGardner(x, config) resamples a matched-filtered
%   window onto the symbol timing recovered by a Gardner loop, returning
%   TWO samples per symbol.
%
%   info fields: .outputSamples, .finalRate (output samples per input
%   sample, nominally 2/samplesPerSymbol), .rateErrorPPM, .errorRMS.
%
%   Enabled by config.uplink.timingSyncEnabled, and OFF by default. Read the
%   note below before turning it on -- this is a real timing-recovery loop
%   and it works, but on this waveform it is competing with something that
%   is already better.
%
%   HOW IT WORKS. Three pieces, the standard arrangement:
%
%     interpolator   a 4-point cubic, which can produce a sample at any
%                    fractional instant between the ones the radio gave us
%     detector       Gardner's: e = Re{ conj(mid) * (curr - prev) }, over
%                    one on-symbol sample, the one before it, and the
%                    half-symbol sample between them. Its virtue is that it
%                    is INDEPENDENT OF CARRIER PHASE, which is why it can
%                    sit here, ahead of any carrier recovery.
%     loop filter    proportional-plus-integral, driving the interpolation
%                    rate. The integrator is what lets it track a sample
%                    CLOCK offset rather than just a fixed timing error.
%
%   WHY IT IS OFF BY DEFAULT, AND WHAT IT WOULD TAKE TO BE WORTH IT. Two
%   reasons, both specific to this link rather than to Gardner:
%
%   1. THE START-SEQUENCE CORRELATION ALREADY SOLVES TIMING, and solves it
%      better. It searches every sample offset at 25 samples per symbol, so
%      it lands within 1/25 of a symbol, in ONE SHOT, with no convergence
%      time. On a root-raised-cosine at rolloff 0.35 a 0.04-symbol timing
%      error costs well under a tenth of a decibel, so there is very little
%      for a loop to recover.
%
%   2. A LOOP HAS NOWHERE TO CONVERGE. A CLTU is 320 symbols. The loop
%      spends the guard interval before it running on noise, so it arrives
%      at the burst pointing somewhere arbitrary, and then has roughly
%      1/(Bn*T) symbols to pull in -- which at any bandwidth quiet enough
%      not to jitter is a large fraction of the burst. This is the same
%      reason the Costas loop is placed AFTER the correlation rather than
%      before it.
%
%   The case where it starts to earn its place is a sample-clock offset
%   large enough to drag the timing across the burst. Two free-running
%   USRPs at 2.5 ppm drift 0.0008 of a symbol over 320 symbols, which is
%   nothing -- so on THIS waveform, it is insurance against a fault rather
%   than a working part of the chain. It becomes genuinely necessary if the
%   burst grows much longer (LDPC(512,256), or several codeblocks per CLTU),
%   because then the drift has time to accumulate.
%
%   TURNING IT ON changes what stage 4 receives: two samples per symbol
%   instead of twenty-five. ccsdsUplinkReceive.m passes the right value, and
%   ccsdsUplinkASMDetect.m takes it as an argument for exactly this reason.

    sps  = config.uplink.samplesPerSymbol;
    Rs   = config.uplink.symbolRate;
    Bn   = config.uplink.gardnerLoopBandwidthHz;
    zeta = config.uplink.gardnerDampingFactor;

    x = complex(x(:));
    info = struct('outputSamples', 0, 'finalRate', 2/sps, ...
        'rateErrorPPM', 0, 'errorRMS', 0, 'clamped', false);

    if numel(x) < 8
        y = complex(zeros(0,1));
        return;
    end

    % Normalise, so the detector's gain -- and therefore the loop bandwidth
    % the gains below are computed for -- does not depend on received level.
    p = sqrt(mean(abs(x).^2));
    if p > 0
        x = x / p;
    end

    % Same second-order loop design as the Costas loop. The detector fires
    % once per SYMBOL, so the loop's sample period is the symbol period.
    T = 1/Rs;
    theta = Bn*T/(zeta + 0.25/zeta);
    d = 1 + 2*zeta*theta + theta^2;
    Kp = 4*zeta*theta/d;
    Ki = 4*theta^2/d;

    rateNom = 2/sps;          % output samples per input sample
    rate = rateNom;
    integ = 0;

    y = complex(zeros(ceil(numel(x)*rateNom) + 8, 1));
    nOut = 0;

    eta = 0;                  % modulo-1 interpolation counter
    prevSym = complex(0);     % last on-symbol sample
    midSym = complex(0);      % half-symbol sample between prev and current
    errAcc = 0;
    errCount = 0;

    for n = 3:numel(x)-1
        eta = eta - rate;
        if eta >= 0
            continue;
        end

        % Fractional position of the boundary between x(n-1) and x(n).
        mu = eta/rate + 1;
        eta = eta + 1;

        % 4-point cubic (Lagrange) interpolation, with x(n-1) at position 0
        % and x(n) at position 1.
        v = localCubic(x(n-2), x(n-1), x(n), x(n+1), mu);

        nOut = nOut + 1;
        y(nOut) = v;

        if mod(nOut, 2) == 1
            % Odd outputs are the half-symbol samples.
            midSym = v;
        else
            % Even outputs are on-symbol. Gardner's error, which needs
            % exactly these three: the symbol, the one before it, and the
            % midpoint. The subtraction is what makes it blind to a
            % constant carrier phase.
            %
            % NORMALISED BY THE SYMBOL ENERGY, which is not cosmetic. The
            % raw detector output is in units of amplitude squared, while
            % the loop gains below are derived for an error measured in
            % FRACTIONS OF A SYMBOL. Left unnormalised, a single Kp*e term
            % came to about three quarters of the nominal rate, and the loop
            % slammed into its own clamp on the first transition -- a 20%
            % rate error, which is to say a resampler that no longer
            % produces two samples per symbol at all.
            e = real(conj(midSym) * (v - prevSym)) / ...
                ((abs(prevSym)^2 + abs(v)^2)/2 + eps);
            prevSym = v;

            % e > 0 means the sampling instants are LATE: the midpoint
            % sample has already taken the sign of the new symbol. Sampling
            % faster brings them back, so the correction adds.
            integ = integ + Ki*e;
            corr = integ + Kp*e;

            % A fraction of the nominal rate, not an absolute offset. The
            % clamp is on that fraction, and 5% is already twenty thousand
            % times any clock error two USRPs can produce -- it exists to
            % stop a loop running on noise from walking off, not to
            % accommodate a real offset.
            corr = min(max(corr, -0.05), 0.05);
            rate = rateNom * (1 + corr);

            errAcc = errAcc + e^2;
            errCount = errCount + 1;
        end
    end

    y = y(1:nOut);

    info.outputSamples = nOut;
    info.finalRate = rate;
    info.rateErrorPPM = 1e6*(rate - rateNom)/rateNom;
    info.clamped = abs(rate/rateNom - 1) >= 0.0499;
    if errCount > 0
        info.errorRMS = sqrt(errAcc/errCount);
    end
end

function v = localCubic(a, b, c, d, mu)
% Cubic interpolation over four points at -1, 0, 1, 2, evaluated at MU in
% [0,1) -- i.e. between b and c. Reduces exactly to b at mu = 0 and to c at
% mu = 1, so a loop sitting still resamples the input unchanged.
    v = a * (-mu*(mu-1)*(mu-2)/6) + ...
        b * ((mu+1)*(mu-1)*(mu-2)/2) + ...
        c * (-(mu+1)*mu*(mu-2)/2) + ...
        d * ((mu+1)*mu*(mu-1)/6);
end
