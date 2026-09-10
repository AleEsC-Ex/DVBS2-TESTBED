function [normCFO, info] = dvbs2FineFreqEst(plSymbols, fp)
%DVBS2FINEFREQEST Pilot-aided fine carrier frequency offset (CFO) estimate.
%
%   [normCFO, info] = dvbs2FineFreqEst(plSymbols, fp) estimates the
%   residual carrier frequency offset left after coarse correction
%   (see dvbs2CoarseFreqEst), using the known pilot symbols scattered
%   throughout a DVB-S2 PLFRAME. It is a multi-lag Luise & Reggiannini
%   (L&R) estimator applied to the pilot-block residuals: correlation
%   lags are computed WITHIN each 36-symbol pilot block only (never
%   across the data symbols separating two blocks) and then averaged
%   over all pilot blocks in the frame, giving a much lower-variance
%   estimate than using a single block or a single lag.
%
%   Only usable for frames with pilots enabled (phyParams.HasPilots);
%   for non-pilot frames use dvbs2NonPilotFineFreqPhase instead.
%
% Inputs:
%   plSymbols - Column vector with one full, coarse-corrected PLFRAME
%               (header + payload + pilots), at 1 sample/symbol.
%   fp        - Pilot structure returned by dvbs2PilotStructure, giving
%               the pilot symbol indices (fp.pilotInd), reference pilot
%               values (fp.refPilots) and block count (fp.numPilotBlocks)
%               for this frame.
%
% Outputs:
%   normCFO - Estimated residual normalized frequency offset, in
%             cycles/symbol. Multiply by the symbol rate to get Hz.
%             Returns 0 if the frame has no pilots (fp.pilotInd empty).
%   info    - Diagnostic struct: received/residual pilot values, the
%             lag-summed correlation (AdjacentCorrelation), a
%             coherence metric useful for judging estimate reliability
%             (CorrelationCoherence, close to 1 = clean/reliable), and
%             the lag order used (M).
    plSymbols = plSymbols(:);
    if isempty(fp.pilotInd)
        normCFO = 0;
        info = struct;
        return;
    end
    if length(fp.pilotInd) ~= length(fp.refPilots)
        error("Pilot indices and reference-pilot lengths do not match.");
    end

    rxPilots = plSymbols(fp.pilotInd);
    pilotResidual = rxPilots .* conj(fp.refPilots);
    pilotMatrix = reshape(pilotResidual, 36, fp.numPilotBlocks);  % one column per block

    L = size(pilotMatrix,1);   % 36 symbols/block
    P = size(pilotMatrix,2);   % number of pilot blocks

    % Multi-lag L&R estimator, lags computed WITHIN each block only,
    % summed/averaged over all blocks. M=1 (old code) is the worst-case
    % variance point of this family; larger M trades unambiguous range
    % for much lower variance. M=15 keeps capture range at
    % +/-1/(2*16) = +/-0.03125 cycles/symbol (~830 kHz at typical Fsym),
    % comfortably above any residual you'll see post-coarse-correction.
    M = min(15, L-1);
    Rsum = complex(0);
    cohNum = 0;
    for m = 1:M
        prod_m = pilotMatrix(m+1:end,:) .* conj(pilotMatrix(1:end-m,:));  % (L-m) x P
        Rsum   = Rsum + sum(prod_m,'all') / ((L-m)*P);
        cohNum = cohNum + sum(abs(prod_m),'all') / ((L-m)*P);
    end

    normCFO = angle(Rsum) / (pi*(M+1));   % NOTE: pi, not 2*pi -- matches
                                           % your coarse estimator's formula;
                                           % M=1 case reduces to angle/(2*pi),
                                           % i.e. exactly your old code.

    info.ReceivedPilots = rxPilots;
    info.PilotResidual = pilotResidual;
    info.AdjacentCorrelation = Rsum;
    info.CorrelationCoherence = abs(Rsum) / (cohNum/M + eps);
    info.M = M;
end