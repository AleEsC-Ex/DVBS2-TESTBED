function [recommendedMODCOD, state] = dvbs2ACMPolicy(report, currentMODCOD, state, config)
%DVBS2ACMPOLICY Recommend a MODCOD from batched SNR statistics, with hysteresis and dwell.
%
%   [recommendedMODCOD, state] = dvbs2ACMPolicy(report, currentMODCOD, state, config)
%   takes one feedback REPORT -- a struct with Count, MeanSNRdB and
%   SigmaSNRdB summarising the frames the receiver decoded since its last
%   report -- folds it into a rolling window of such reports, and maps the
%   combined statistics onto a MODCOD via dvbs2SelectMODCOD.m.
%
%   Pass [] as STATE on the first call.
%
%   WHY BATCHES RATHER THAN RAW SAMPLES: mean and standard deviation
%   recombine exactly across groups,
%
%       n = sum(ni),  mu = sum(ni*mui)/n
%       sigma^2 = sum(ni*(sigmai^2 + mui^2))/n - mu^2
%
%   so a window of per-batch summaries gives precisely the mean and sigma a
%   window of raw samples would have. Nothing the policy uses is lost, and
%   the report stays 5 bytes however many frames it covers -- which is what
%   makes event-driven reporting affordable over a narrowband return link.
%
%   The trend is the least-squares slope of the batch MEANS against their
%   arrival times. Computing it across batches rather than within one is an
%   improvement, not a compromise: a batch mean over tens of frames is far
%   less noisy than a single frame's estimate, whose own spread is ~0.4 dB.
%
%   COUNT = 0 reports ("alive, decoded nothing") are not folded into the
%   statistics -- they carry no SNR information. They still prove the return
%   link is up, which is the transmitter's separate liveness concern.
%
%   POLICY: the candidate comes from dvbs2SelectMODCOD.m, which compares
%   each rung's threshold against mu - k*sigma rather than the instantaneous
%   SNR, so the safety margin is sized by the channel's own measured
%   variability. A falling trend subtracts further headroom. A change must
%   then be recommended by several consecutive calls AND clear a minimum
%   dwell time, both asymmetric: moving up only costs throughput if delayed,
%   moving down costs frames for every moment of delay.

    if isempty(state)
        state = struct('Batches', struct('n',{},'mu',{},'sigma',{},'t',{}), ...
            'LastSwitchTime', NaT, 'PendingMODCOD', currentMODCOD, 'PendingCount', 0, ...
            'Mu', NaN, 'Sigma', NaN, 'SlopePerSec', 0, 'TrendAdjDB', 0, 'Frames', 0);
    end

    %% "Alive, decoded nothing" -- drop immediately, bypassing hysteresis.
    % Over a whole heartbeat interval that is tens of frames, so it is not a
    % glitch: the forward link is not working at the current MODCOD. The
    % danger this guards against is subtle -- a zero-count report carries no
    % statistics, so without this it would fall through to the normal path
    % and could MATURE A PENDING UPGRADE that earlier reports had been
    % building toward, raising the MODCOD at exactly the moment the receiver
    % said it could decode nothing.
    if report.Count == 0
        modcodSet = sort(config.acm.modcodSet);
        recommendedMODCOD = modcodSet(1);
        state.PendingMODCOD = recommendedMODCOD;
        state.PendingCount = 0;
        if recommendedMODCOD ~= currentMODCOD
            state.LastSwitchTime = datetime('now');
        end
        return;
    end

    %% Fold this report into the rolling window
    if report.Count > 0
        b.n = double(report.Count);
        b.mu = report.MeanSNRdB;
        b.sigma = report.SigmaSNRdB;
        b.t = posixtime(datetime('now'));
        state.Batches(end+1) = b;
        if numel(state.Batches) > config.acm.batchWindow
            state.Batches = state.Batches(end-config.acm.batchWindow+1 : end);
        end
    end

    if isempty(state.Batches)
        % Nothing usable yet -- hold whatever is being transmitted.
        recommendedMODCOD = currentMODCOD;
        return;
    end

    %% Combine the window exactly
    n  = [state.Batches.n];
    mu = [state.Batches.mu];
    sg = [state.Batches.sigma];
    N  = sum(n);
    muAll = sum(n.*mu)/N;
    varAll = sum(n.*(sg.^2 + mu.^2))/N - muAll^2;
    sigmaAll = sqrt(max(varAll, 0));

    %% Trend: least-squares slope of batch means against arrival time
    t = [state.Batches.t];
    if numel(t) >= config.acm.trendMinBatches && (max(t)-min(t)) > 0
        tc = t - mean(t);
        slope = sum(tc .* (mu - mean(mu))) / sum(tc.^2);   % dB per second
    else
        slope = 0;
    end
    % Only a FALLING trend adjusts the bound. Extrapolating a rising channel
    % would make the policy optimistic about SNR it has not observed, which
    % is the mistake that costs frames; being late to move up costs only
    % throughput.
    trendAdjDB = min(0, slope * config.acm.trendHorizonSec);

    state.Mu = muAll; state.Sigma = sigmaAll;
    state.SlopePerSec = slope; state.TrendAdjDB = trendAdjDB;
    state.Frames = N;

    %% Candidate, then hysteresis
    candidate = dvbs2SelectMODCOD(muAll, sigmaAll, trendAdjDB, currentMODCOD, config);

    isUpgrade = candidate > currentMODCOD;
    if isUpgrade
        requiredAgree = config.acm.agreeCountUp;
        requiredDwellSec = config.acm.minDwellSecUp;
    else
        requiredAgree = config.acm.agreeCountDown;
        requiredDwellSec = config.acm.minDwellSecDown;
    end

    if candidate == state.PendingMODCOD
        state.PendingCount = state.PendingCount + 1;
    else
        state.PendingMODCOD = candidate;
        state.PendingCount = 1;
    end

    dwellOK = isnat(state.LastSwitchTime) || ...
        seconds(datetime('now') - state.LastSwitchTime) >= requiredDwellSec;

    recommendedMODCOD = currentMODCOD;
    if candidate ~= currentMODCOD && state.PendingCount >= requiredAgree && dwellOK
        recommendedMODCOD = candidate;
        state.LastSwitchTime = datetime('now');
    end
end
