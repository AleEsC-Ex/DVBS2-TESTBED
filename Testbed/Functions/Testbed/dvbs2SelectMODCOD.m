function modcod = dvbs2SelectMODCOD(mu, sigma, trendAdjDB, currentMODCOD, config)
%DVBS2SELECTMODCOD Highest MODCOD supportable by a VARIABILITY-AWARE SNR bound.
%
%   modcod = dvbs2SelectMODCOD(mu, sigma, trendAdjDB, currentMODCOD, config)
%   walks config.acm.modcodSet and returns the highest MODCOD whose
%   quasi-error-free threshold is still met by a deliberately pessimistic
%   estimate of the link's SNR:
%
%       bound = mu - max(k*sigma, marginFloorDB) + trendAdjDB
%
%   rather than by the instantaneous SNR alone. This is the single place
%   the MODCOD ladder is consulted, so the transmitter's steady-state
%   policy (dvbs2ACMPolicy.m) and its one-shot link-establishment seed
%   (S1a_Transmitter.m) cannot drift apart.
%
%   WHY mu - k*sigma RATHER THAN mu - constant: a fixed margin is
%   unrelated to how much the channel actually moves, so it is
%   simultaneously too small on a volatile link (the MODCOD is chosen
%   from a mean the channel spends half its time below, and every dip
%   past the margin costs frames) and needlessly large on a steady one.
%   Scaling the margin by the MEASURED standard deviation makes k a
%   quantity with a defensible meaning -- for a roughly normal SNR
%   distribution it sets the fraction of time the link sits above the
%   threshold (k=1 ~84%, k=2 ~97.7%, k=3 ~99.87%), i.e. the target
%   outage rate. On a steady channel sigma is small and this costs
%   nothing; protection appears only when there is real variability to
%   protect against.
%
%   CAVEAT: sigma measures the spread of the SNR ESTIMATES, which
%   includes dvbs2SNREstimate.m's own estimator noise (sparse pilot
%   blocks give real frame-to-frame variance even on a constant
%   channel), not just genuine channel variation. The two cannot be
%   separated without a better estimator, so k is deliberately
%   conservative-by-default rather than aggressive -- see
%   config.acm.sigmaK.
%
% Inputs:
%   mu            - Mean SNR in dB over the observation window.
%   sigma         - Standard deviation of the SNR in dB over that window.
%   trendAdjDB    - Predictive adjustment in dB, <= 0. Applied on top of
%                   the bound to step down BEFORE a falling channel
%                   crosses the threshold rather than after frames are
%                   already lost. Pass 0 to disable.
%   currentMODCOD - The MODCOD currently in use, or NaN if there is none
%                   yet (link establishment). The MODCOD already in use
%                   is evaluated with the more forgiving
%                   config.acm.sigmaKStay, which is what stops the
%                   selection flapping between two neighbouring rungs
%                   when sigma sits near a threshold.
%   config        - The struct from dvbs2TestbedConfig.m (config.acm.*).
%
% Output:
%   modcod        - Highest supportable MODCOD; falls back to the lowest
%                   entry in config.acm.modcodSet if none qualifies.

    modcodSet = sort(config.acm.modcodSet);
    modcod = modcodSet(1);

    for k = 1:numel(modcodSet)
        m = modcodSet(k);

        if ~isnan(currentMODCOD) && m == currentMODCOD
            kSigma = config.acm.sigmaKStay;
        else
            kSigma = config.acm.sigmaK;
        end

        % Floor the protection so a quiet or very short window (sigma
        % near zero) cannot produce an unrealistically optimistic bound.
        protection = max(kSigma * sigma, config.acm.marginFloorDB);
        bound = mu - protection + trendAdjDB;

        if bound >= config.acm.qefEsNodB(m)
            modcod = m;
        end
    end

    modcod = localCapUpwardJump(modcod, currentMODCOD, modcodSet, config);
end

function modcod = localCapUpwardJump(modcod, currentMODCOD, modcodSet, config)
%LOCALCAPUPWARDJUMP Limit how far one decision may climb the ladder.
%
%   Every MODCOD change costs S1a a radio release and reopen, because
%   comm.SDRuTransmitter locks its input length and the waveform length
%   changes with the MODCOD. That is a real gap in the transmitted carrier.
%   A link that jumps from the bottom of the ladder to the top on its first
%   decision therefore breaks itself: the receiver sees the gap, reports the
%   resulting garbage, and the policy jumps straight back down. The
%   oscillation is driven by the switching, not by the channel.
%
%   DOWNWARD MOVES ARE NEVER CAPPED. Every moment spent on a MODCOD the
%   channel can no longer support costs real frames, so a collapsing link
%   must be able to reach safety in one decision. This is the same asymmetry
%   config.acm.agreeCountDown and .minDwellSecDown already encode.
%
%   At link establishment there is no current MODCOD, so the cap is measured
%   from whatever S1a has actually been transmitting during calibration --
%   config.dvbs2.MODCOD. Without that, the very first decision would be the
%   largest jump of the whole run, which is precisely the one to avoid.

    if ~isfield(config.acm, 'maxJumpRungs') || ~isfinite(config.acm.maxJumpRungs)
        return;
    end

    if isnan(currentMODCOD)
        fromMODCOD = config.dvbs2.MODCOD;
    else
        fromMODCOD = currentMODCOD;
    end

    iFrom = find(modcodSet == fromMODCOD, 1);
    iTo = find(modcodSet == modcod, 1);
    if isempty(iFrom) || isempty(iTo) || iTo <= iFrom
        return;   % unknown rung, or a downward/no move -- leave it alone
    end

    modcod = modcodSet(min(iTo, iFrom + config.acm.maxJumpRungs));
end
