function [fTrackedHz, state] = dvbs2CFOTracker(fMeasHz, dtSec, state, config)
%DVBS2CFOTRACKER Closed-loop alpha-beta tracker for the carrier frequency offset.
%
%   [fTrackedHz, state] = dvbs2CFOTracker(fMeasHz, dtSec, state, config)
%   maintains a running estimate of the CFO and of its RATE of change,
%   updating both from each new (noisy) measurement instead of trusting
%   the measurement outright. Pass [] as STATE on the first call.
%
%   Each call does three things:
%     1. PREDICT  where the offset should be now:  F + Fdot*dtSec
%     2. COMPARE  that prediction against the measurement -> "miss"
%     3. NUDGE    both F and Fdot a fraction of the way toward the miss
%
%   config.cfoTrack.alpha sets what fraction of the miss corrects F, and
%   .beta the same for Fdot. Alpha near 1 follows measurements closely and
%   is jumpy; small alpha is smooth but slow to react. Beta is always much
%   smaller, because the rate is never measured directly -- it is only
%   inferred from a run of consistently-signed misses.
%
%   WHY A RATE TERM: with a static offset (both radios on one bench) Fdot
%   stays near zero and beta does nothing. Under Doppler it earns its
%   place: a tracker that estimates only frequency settles at a constant
%   lag behind a sliding offset, of roughly rate*dt*(1-alpha)/alpha. At
%   the ~770 Hz/s peak rate of an overhead LEO pass at 2 GHz, with 100 ms
%   between updates and alpha = 0.2, that is a permanent ~300 Hz error.
%   Estimating the rate removes it.
%
%   ROBUSTNESS: a measurement whose miss is implausibly large is far more
%   likely to be a bad SOF correlation than a real frequency jump, so it
%   is gated out and the prediction is coasted instead. The gate scales
%   with the running average miss, so it adapts to how noisy this link
%   actually is rather than needing a hand-set threshold. If enough
%   measurements are rejected consecutively, the assumption flips: the
%   offset really did move, so the tracker re-acquires on the latest
%   measurement rather than coasting forever on a stale estimate.
%
% Inputs:
%   fMeasHz - Measured offset in Hz for this update.
%   dtSec   - Seconds elapsed since the previous measurement. Only used
%             for the rate term; ignored on the initialising first call.
%   state   - [] on the first call, otherwise the struct returned before.
%   config  - dvbs2TestbedConfig() struct (config.cfoTrack.* is used).
%
% Outputs:
%   fTrackedHz - Filtered offset estimate to apply.
%   state      - Updated state. Fields F, Fdot, MeanAbsMiss, NumUpdates,
%                ConsecRejects, LastMiss and LastRejected are also useful
%                for logging: a run of same-signed LastMiss values means
%                the tracker is lagging something that is genuinely
%                moving, whereas random signs are just measurement noise.

    % EMA weight for the running miss scale that sets the gate. Fixed
    % rather than configurable -- it only needs to be slow enough that one
    % outlier cannot inflate the gate that is supposed to reject it.
    missScaleWeight = 0.1;

    if isempty(state)
        state = struct('F', fMeasHz, 'Fdot', 0, 'MeanAbsMiss', 0, ...
            'NumUpdates', 1, 'ConsecRejects', 0, 'LastMiss', 0, ...
            'LastRejected', false);
        fTrackedHz = fMeasHz;
        return;
    end

    % The rate update divides by dtSec, so a zero, negative or absurdly
    % small value would blow the rate estimate up and poison every later
    % prediction. Treat a bad dt as "rate unknown for this step" rather
    % than trusting the caller.
    dtValid = isfinite(dtSec) && dtSec > 0;

    %% 1. Predict, 2. Compare
    if dtValid
        fPred = state.F + state.Fdot * dtSec;
    else
        fPred = state.F;
    end
    miss = fMeasHz - fPred;

    %% Gate: is this measurement believable?
    gateHz = max(config.cfoTrack.gateFactor * state.MeanAbsMiss, ...
        config.cfoTrack.gateFloorHz);
    accept = (state.NumUpdates < config.cfoTrack.minUpdatesBeforeGating) || ...
        (abs(miss) <= gateHz);

    if ~accept
        state.ConsecRejects = state.ConsecRejects + 1;
        if state.ConsecRejects >= config.cfoTrack.maxConsecRejects
            % Too many rejects in a row to still be bad luck -- the offset
            % has genuinely moved. Restart from this measurement.
            state = struct('F', fMeasHz, 'Fdot', 0, 'MeanAbsMiss', 0, ...
                'NumUpdates', 1, 'ConsecRejects', 0, 'LastMiss', miss, ...
                'LastRejected', true);
        else
            % Coast: keep the prediction, leave the rate alone.
            state.F = fPred;
            state.LastMiss = miss;
            state.LastRejected = true;
        end
        fTrackedHz = state.F;
        return;
    end

    %% 3. Nudge
    state.F = fPred + config.cfoTrack.alpha * miss;
    if dtValid
        state.Fdot = state.Fdot + config.cfoTrack.beta * miss / dtSec;
    end

    state.MeanAbsMiss = (1 - missScaleWeight) * state.MeanAbsMiss + ...
        missScaleWeight * abs(miss);
    state.NumUpdates = state.NumUpdates + 1;
    state.ConsecRejects = 0;
    state.LastMiss = miss;
    state.LastRejected = false;

    fTrackedHz = state.F;
end
