function [plSymbols_corrected, info] = ...
    dvbs2NonPilotFineFreqPhase(plSymbols, sofRef, phyParams, Fsym)
%DVBS2NONPILOTFINEFREQPHASE Fine CFO/phase correction for pilot-less PLFRAMEs.
%
%   [plSymbols_corrected, info] = dvbs2NonPilotFineFreqPhase(plSymbols, sofRef, phyParams, Fsym)
%   corrects the residual carrier frequency offset and phase of one
%   coarse-corrected PLFRAME when no pilot symbols are available
%   (phyParams.HasPilots == false), using a three-stage approach:
%     1) Coarse SOF-based phase correction (correlate the received PL
%        header against the known SOF pattern).
%     2) Blind M-th power frequency estimate over the payload only
%        (raising constant-modulus symbols -- BPSK/QPSK/8PSK -- to the
%        M-th power strips the data modulation, leaving a pure
%        frequency ramp that lrEstimate can track).
%     3) A decision-directed PLL (comm.CarrierSynchronizer) for final
%        fine tracking of the payload.
%
%   LIMITATION -- 16APSK/32APSK, non-pilot frames only: APSK
%   constellations are not constant-modulus (they have multiple
%   amplitude rings), so the M-th power trick in stage 2 does not apply
%   cleanly. For these modulation orders this function skips the blind
%   CFO stage (Mpow=0, normCFO=0) and falls back to running the stage-3
%   PLL configured as QPSK, which is only an approximate phase-tracking
%   substitute -- it is not a correct decision-directed detector for a
%   multi-ring constellation and its tracking accuracy is not
%   guaranteed. A warning is raised whenever this fallback is used
%   (dvbs2NonPilotFineFreqPhase:UnsupportedMod). If your ACM plan uses
%   16/32APSK regularly, prefer configuring those MODCODs with pilots
%   enabled so dvbs2FineFreqEst is used instead.
%
% Inputs:
%   plSymbols - Column vector with one full, coarse-corrected PLFRAME
%               (header + payload), at 1 sample/symbol.
%   sofRef    - Reference SOF symbols from dvbs2SOFReference().
%   phyParams - Struct with the decoded PLHEADER metadata (must include
%               ModulationOrder), as returned by dvbs2PLHeaderRecover.
%   Fsym      - (Optional) Symbol rate in Hz, used only to additionally
%               report the blind CFO estimate in Hz via info.BlindCFOHz.
%
% Outputs:
%   plSymbols_corrected - Frequency- and phase-corrected PLFRAME
%                         (header + payload), same length as plSymbols.
%   info                - Diagnostic struct: SOF phase estimate, blind
%                         normalized/Hz CFO estimate, the modulation
%                         type and M-power order used, and the PLL's
%                         per-symbol phase error trace (PLLPhaseError).

    plSymbols = plSymbols(:);
    N = length(plSymbols);
    Lsof = length(sofRef);
    headerLen = 90;

    if N < headerLen
        error('plSymbols shorter than PL header length');
    end

    modOrder = phyParams.ModulationOrder;
    switch modOrder
        case 2
            modType = 'BPSK'; Mpow = 2;
        case 4
            modType = 'QPSK'; Mpow = 4;
        case 8
            modType = '8PSK'; Mpow = 8;
        otherwise
            % Non-constant-modulus APSK: M-th power doesn't apply
            % cleanly. Fall back to phase-only + PLL, same as before.
            warning('dvbs2NonPilotFineFreqPhase:UnsupportedMod', ...
                'M-th power blind CFO estimate not supported for ModulationOrder=%d; falling back to phase-only pre-correction.', modOrder);
            modType = 'QPSK'; Mpow = 0;
    end

    %% 1) Coarse SOF phase correction (as before)
    rxSof = plSymbols(1:Lsof);
    phEst = angle(sum(rxSof .* conj(sofRef)));
    plSymbolsPh = plSymbols .* exp(-1j*phEst);

    %% 2) Blind M-th power frequency estimate over the payload only
    payload = plSymbolsPh(headerLen+1:end);

    if Mpow > 0
        powered = payload.^Mpow;
        numLags = min(10, length(powered)-1);
        normCFO_powered = lrEstimate(powered, numLags);
        normCFO = normCFO_powered / Mpow;   % undo the M-th power scaling
        % NOTE unambiguous range here is +/-1/(2*Mpow*(numLags+1))
        % cycles/symbol post-M-th-power-scaling -- still comfortably
        % wide (hundreds of kHz to MHz range) relative to any residual
        % left after coarse correction. If you push numLags much higher
        % for lower variance, re-check this range against your worst
        % case coarse-stage residual.
    else
        normCFO = 0;
    end

    n = (0:N-1).';
    plSymbolsFreqCorr = plSymbolsPh .* exp(-1j*2*pi*normCFO*n);

    %% 3) Decision-directed PLL for final fine tracking (payload only)
    persistent cs lastModType
    loopBW = 1e-3;
    damping = 0.707;
    if isempty(cs) || ~isequal(modType, lastModType)
        cs = comm.CarrierSynchronizer( ...
            Modulation = modType, ...
            DampingFactor = damping, ...
            NormalizedLoopBandwidth = loopBW, ...
            SamplesPerSymbol = 1, ...
            ModulationPhaseOffset = 'Auto');
        lastModType = modType;
    end

    payloadFreqCorr = plSymbolsFreqCorr(headerLen+1:end);
    [payloadCorrected, phErrPayload] = cs(payloadFreqCorr);

    % Header: skip the decision-directed slicer (wrong constellation).
    % Apply the blind frequency correction plus the PLL's own phase
    % estimate at the start of the payload as a static approximation.
    headerFreqCorr = plSymbolsFreqCorr(1:headerLen);
    headerPhaseApprox = phErrPayload(1);
    headerCorrected = headerFreqCorr .* exp(-1j*headerPhaseApprox);

    plSymbols_corrected = [headerCorrected; payloadCorrected];

    info.SOFPhaseEst = phEst;
    info.BlindNormCFO = normCFO;
    if nargin >= 4 && ~isempty(Fsym)
        info.BlindCFOHz = normCFO*Fsym;
    end
    info.ModulationType = modType;
    info.MthPowerOrder = Mpow;
    info.PLLPhaseError = phErrPayload;
end
