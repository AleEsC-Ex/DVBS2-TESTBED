function [dataBits, isFrameLost, pktCRC] = AEC_dvbs2BitRecover(rxDataFrame, phyParams, nVar)
% AEC_dvbs2BitRecover: Fault-tolerant DVB-S2 bit recovery.
%
%   [dataBits, isFrameLost, pktCRC] = AEC_dvbs2BitRecover(rxDataFrame, phyParams, nVar)
%   runs the DVB-S2 physical-layer and link-layer recovery chain
%   (descrambling, LDPC/BCH decoding, and BBHEADER/MPEG-TS packet
%   extraction) on one already frequency/phase-corrected PLFRAME
%   payload, and reports whether the frame was usable.
%
%   The function is intentionally tolerant of decode failures: if the
%   internal recovery engines throw (e.g. because the frame is too
%   noisy to converge), the error is caught and the frame is simply
%   reported as lost rather than stopping the whole receive loop.
%
% Inputs:
%   rxDataFrame - Column vector with the corrected PLFRAME PAYLOAD only
%                 (i.e. WITHOUT the 90-symbol PL header).
%   phyParams   - Struct with the decoded PLHEADER metadata (MODCOD,
%                 FEC code rate, FECFrameLength, HasPilots, etc.), as
%                 returned by dvbs2PLHeaderRecover.
%   nVar        - Estimated noise variance, used by the LDPC decoder to
%                 compute soft-decision LLRs (log-likelihood ratios).
%
% Outputs:
%   dataBits    - Recovered payload bits (e.g. MPEG-TS packets) as a
%                 double column vector. Empty if the frame was lost.
%   isFrameLost - Logical flag: true if the BBHEADER failed its CRC
%                 check (or the recovery engine threw an exception),
%                 false if the frame decoded to a valid BBHEADER.
%   pktCRC      - Logical/double array with the individual CRC-8 result
%                 of each extracted MPEG-TS packet (1 = OK, 0 = failed).
%                 Empty if the frame was lost.

    % 1. Default outputs: assume the worst case (frame lost) until
    %    proven otherwise below.
    dataBits = [];
    isFrameLost = true;
    pktCRC = [];

    M = phyParams.ModulationOrder;
    cwLen = phyParams.FECFrameLength;

    % Convert the LDPC code rate identifier to a numeric value
    % (e.g. the string '1/2' -> 0.5).
    if ischar(phyParams.LDPCCodeIdentifier) || isstring(phyParams.LDPCCodeIdentifier)
        R = eval(phyParams.LDPCCodeIdentifier);
    else
        R = double(phyParams.LDPCCodeIdentifier);
    end

    % 2. Protected decode block (try/catch).
    try
        % Physical layer: descrambling, LLR computation, LDPC and BCH
        % decoding. Delegates to MathWorks' internal C-MEX engines for
        % speed.
        decFrame = satcom.internal.dvbs.s2BBFrameRecover(...
            rxDataFrame, M, R, cwLen, phyParams.HasPilots, nVar, false);

        % Link layer: BBHEADER CRC validation and MPEG-TS packet
        % extraction.
        [dataBits_raw, hasCRCFailed, crcOut, ~] = satcom.internal.dvbs.streamRecover(decFrame);

        % 3. Format the outputs if there was no catastrophic failure.
        isFrameLost = hasCRCFailed;

        if ~hasCRCFailed
            dataBits = double(dataBits_raw); % cast to double to simplify biterr() calls
            pktCRC = double(crcOut);
        end

    catch ME
        % Noise can be severe enough that the internal engines fail
        % mathematically (rather than just returning a bad CRC); in
        % that case the frame is reported as lost instead of
        % propagating the exception. A warning is raised (silence it
        % with warning('off','AEC_dvbs2BitRecover:EngineFailure')) so a
        % genuine noise-related loss can still be told apart from a
        % real bug.
        warning('AEC_dvbs2BitRecover:EngineFailure', ...
            'Frame dropped due to an internal exception: %s', ME.message);
    end
end
