function [outSymbols, info] = dvbs2PhaseCompensate(plSymbols, fp, refHeader)
%DVBS2PHASECOMPENSATE Pilot-aided phase trajectory correction.
%
%   [outSymbols, info] = dvbs2PhaseCompensate(plSymbols, fp, sofRef)
%   estimates and removes the phase-noise/residual-CFO trajectory
%   across one PLFRAME by measuring the phase at the SOF and at every
%   pilot block, then fitting a phase-vs-symbol-index trajectory
%   through those "anchor" points and applying it to every symbol in
%   the frame. Intended to run AFTER coarse and fine frequency
%   correction (see dvbs2CoarseFreqEst, dvbs2FineFreqEst), to clean up
%   the slow phase drift/wander that a single frequency estimate can't
%   fully capture.
%
%   Two trajectory models are computed:
%     - A weighted least-squares straight line through all anchors
%       (SOF + every pilot block), weighted by each anchor's
%       correlation magnitude (a proxy for its reliability). Used by
%       default, since it averages out noise across all anchors.
%     - A piecewise linear interpolation/extrapolation between anchors
%       (interp1(...,'extrap')), used ONLY as a fallback when a likely
%       cycle slip is detected (see below), since a single global line
%       cannot follow a phase trajectory containing a 2*pi jump.
%
%   Cycle-slip detection: if consecutive anchors (after unwrap) differ
%   by more than ~0.8*pi, that is treated as a probable cycle slip
%   rather than genuine phase drift, and the function switches from the
%   line fit to the piecewise fallback for that frame (info.PossibleCycleSlip).
%
% Inputs:
%   plSymbols - Column vector with one full, frequency-corrected
%               PLFRAME (header + payload + pilots), at 1 sample/symbol.
%   fp        - Pilot structure returned by dvbs2PilotStructure (pilot
%               indices, reference pilot values, block count). If
%               fp.pilotInd is empty (no pilots in this frame),
%               plSymbols is returned unchanged.
%   sofRef    - Reference SOF symbols from dvbs2SOFReference().
%
% Outputs:
%   outSymbols - Phase-corrected PLFRAME, same length as plSymbols.
%   info       - Diagnostic struct: per-anchor phase/position/weight
%                values, the fitted trajectory (and its line-fit
%                slope/intercept), the cycle-slip flag, and the
%                maximum anchor-to-anchor phase jump observed.

    plSymbols = plSymbols(:);
    N = length(plSymbols);

    if isempty(fp.pilotInd)
        outSymbols = plSymbols;
        info = struct;
        return;
    end

    rxPilots = plSymbols(fp.pilotInd);
    pilotResidual = rxPilots .* conj(fp.refPilots);
    pilotMatrix = reshape(pilotResidual, 36, fp.numPilotBlocks);
    pilotIndexMatrix = reshape(fp.pilotInd, 36, fp.numPilotBlocks);

    pilotBlockPhasor = sum(pilotMatrix,1).';
    pilotBlockMag = abs(pilotBlockPhasor);      % reliability weight (~36 if clean)
    pilotBlockPhase = angle(pilotBlockPhasor);
    pilotBlockCentres = mean(pilotIndexMatrix,1).';

    % HEADER ANCHORS.
    %
    % This used to take the 26-symbol SOF alone, which left the fit badly
    % under-constrained at the start of the frame. Two reasons:
    %
    %   WEIGHT. The SOF contributed 26 of 26 + 22*36 = 818, about 3%. The
    %   line was therefore defined almost entirely by pilots.
    %
    %   POSITION. The header occupies symbols 1-90; the first pilot block
    %   does not begin until symbol 1531. So the header's correction was an
    %   EXTRAPOLATION backwards off a line fitted from data starting 1500
    %   symbols later -- the least accurate part of any fit.
    %
    % Measured consequence: a median 10.5 degrees of phase left on the frame
    % start after correction. Harmless at QPSK, whose decision boundary is
    % +-45 degrees, but 32APSK's is +-11.25 -- essentially the whole margin,
    % which is why 32APSK lost 45% of its frames while reporting a healthy
    % SNR.
    %
    % Passing the full 90-symbol PLHEADER lets it be split into TWO anchors
    % rather than one. That matters more than the extra weight: a single
    % anchor near the start pins only the INTERCEPT, while two anchors 45
    % symbols apart also pin the local SLOPE, which is the quantity the old
    % extrapolation was getting wrong.
    %
    %   SOF   symbols  1-26   centre 13.5   weight ~26   needs no decode
    %   PLSC  symbols 27-90   centre 58.5   weight ~64   needs the PLS decode
    %
    % Header weight rises from 26/818 (3.2%) to 90/882 (10.2%) as a side
    % effect. Backward compatible: pass a 26-symbol sofRef and it behaves
    % exactly as before, with the SOF as a single anchor.
    refHeader = refHeader(:);
    sofLength = min(26, length(refHeader));

    hdrPhase = zeros(0,1); hdrMag = zeros(0,1); hdrCentre = zeros(0,1);

    sofPhasor = sum(plSymbols(1:sofLength) .* conj(refHeader(1:sofLength)));
    hdrPhase(end+1,1)  = angle(sofPhasor);
    hdrMag(end+1,1)    = abs(sofPhasor);
    hdrCentre(end+1,1) = mean(1:sofLength);

    if length(refHeader) > sofLength
        plscIdx = (sofLength+1 : length(refHeader)).';
        plscPhasor = sum(plSymbols(plscIdx) .* conj(refHeader(plscIdx)));
        hdrPhase(end+1,1)  = angle(plscPhasor);
        hdrMag(end+1,1)    = abs(plscPhasor);
        hdrCentre(end+1,1) = mean(plscIdx);
    end

    phasePositions = [hdrCentre; pilotBlockCentres];
    weights = [hdrMag; pilotBlockMag];
    measuredPhase = unwrap([hdrPhase; pilotBlockPhase]);

    % Sanity check: an anchor-to-anchor jump close to +/-pi after
    % unwrap almost certainly means a cycle slip, not real phase.
    jumps = abs(diff(measuredPhase));
    info.MaxAnchorJump = max(jumps);
    info.PossibleCycleSlip = info.MaxAnchorJump > 2.5;  % ~0.8*pi, tune to taste

    % --- Weighted least-squares line through ALL anchors ---
    Aw = [ones(size(phasePositions)), phasePositions] .* sqrt(weights);
    bw = measuredPhase .* sqrt(weights);
    coeffs = Aw \ bw;           % [intercept; slope]
    phi0 = coeffs(1);
    slope = coeffs(2);

    symbolPositions = (1:N).';
    phaseTrajectoryLinearFit = phi0 + slope*symbolPositions;

    if info.PossibleCycleSlip
        % Piecewise fallback: only computed when the linear fit is
        % actually distrusted, since interp1(...,'extrap') is wasted
        % work on every other (common) frame.
        phaseTrajectory = interp1(phasePositions, measuredPhase, ...
            symbolPositions, "linear", "extrap");
    else
        phaseTrajectory = phaseTrajectoryLinearFit;
    end

    outSymbols = plSymbols .* exp(-1j*phaseTrajectory);

    % RESIDUAL AFTER CORRECTION -- how much phase the line model could NOT
    % explain, measured at the anchors themselves. Distinct from FitSlope,
    % which describes what was REMOVED: a large slope with a small residual
    % is a big frequency offset cleanly corrected, whereas a small slope
    % with a large residual means the phase is not behaving like a straight
    % ramp at all and no amount of line fitting will track it.
    %
    % This is the number that predicts whether a dense constellation will
    % survive, because it is what the demodulator actually still sees.
    fitAtAnchors = phi0 + slope*phasePositions;
    anchorResidual = measuredPhase - fitAtAnchors;
    info.AnchorResidualRMSdeg = rad2deg(sqrt(mean(anchorResidual.^2)));
    info.HeaderResidualDeg = rad2deg(anchorResidual(1));

    info.SOFPhase = hdrPhase(1);
    info.HeaderPhase = hdrPhase;
    info.HeaderWeight = hdrMag;
    info.PilotBlockPhase = pilotBlockPhase;
    info.UnwrappedPhase = measuredPhase;
    info.PhasePositions = phasePositions;
    info.PhaseTrajectory = phaseTrajectory;
    info.PhaseTrajectoryLinearFit = phaseTrajectoryLinearFit;
    info.PilotBlockCentres = pilotBlockCentres;
    info.FitSlope = slope;
    info.FitIntercept = phi0;
end