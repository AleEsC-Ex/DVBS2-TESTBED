function phyParams = dvbs2PLHeaderRecover(rxPLHeader)
%DVBS2PLHEADERRECOVER PLHEADER (PLSC) recovery, specialized for plain DVB-S2 only.
%
%   phyParams = dvbs2PLHeaderRecover(rxPLHeader) decodes the 90-symbol
%   pi/2-BPSK PLHEADER (26-symbol SOF + 64-symbol PLSC) of one PLFRAME,
%   returning the MODCOD/FECFrame/pilot parameters this testbed's
%   receive chain needs.
%
%   WHY THIS EXISTS INSTEAD OF MATHWORKS' dvbsPLHeaderRecover: that
%   function's "DVB-S2/S2X regular" mode doesn't assume you know in
%   advance whether an incoming header is legacy DVB-S2 or DVB-S2X --
%   it runs the ML Reed-Muller decode TWICE (once against each
%   standard's candidate codeword table) and then GUESSES which result
%   to trust, based on which decode's minimum distance is the bigger
%   outlier relative to its own table's mean distance:
%
%       s2xCheck = mean(distMetS2X) - min(distMetS2X);
%       s2Check  = mean(distMetS2)  - min(distMetS2);
%       isS2XMODCOD = s2xCheck > s2Check;
%
%   This testbed only ever transmits plain legacy DVB-S2 -- the correct
%   answer is always false -- but that guess can go wrong under modest
%   impairment: the DVB-S2X candidate table is larger (it has to
%   represent more MODCODs), so its codewords are packed more densely,
%   giving the SAME received signal a higher chance of an ML
%   nearest-neighbor error against that table than against the smaller
%   legacy DVB-S2 table -- independent of whether the actual channel
%   conditions are fine for a legacy DVB-S2 decode. This was diagnosed
%   as the likely cause of a specific, repeatable PLHEADER misdecode
%   observed throughout this testbed's development (MODCOD decoded
%   correctly, but the FECFRAME normal/short bit flipping under CFO,
%   consistently landing on the exact same wrong PLSDecimalCode --
%   the signature of a systematic selection error, not random noise).
%
%   Since this testbed never transmits DVB-S2X, wideband, or VL-SNR
%   frames, this function skips that ambiguity entirely and calls the
%   same underlying decoder MathWorks' wrapper uses, but ONLY in its
%   legacy-DVB-S2 mode
%   (satcom.internal.dvbs.plHeaderRecover(rxPLSCode, false, true)) --
%   removing the fragile guess rather than trying to make it more
%   reliable. (satcom.internal.dvbs.* functions are already called
%   directly elsewhere in this codebase -- see AEC_dvbs2BitRecover.m
%   and dvbs2PilotStructure.m -- so this follows an established
%   pattern, not a new one.)
%
% Input:
%   rxPLHeader - 90x1 complex column vector: the frequency/phase
%                corrected PLHEADER symbols (26-symbol SOF + 64-symbol
%                PLSC), e.g. the output of a coarse CFO correction stage
%                (see dvbs2CoarseFreqEst.m).
%
% Output:
%   phyParams - Struct with the fields this testbed's receive chain
%               actually uses (see AEC_dvbs2BitRecover.m,
%               dvbs2FrameLength.m, dvbs2PilotStructure.m):
%                 .ModulationOrder    - constellation size (e.g. 4 for QPSK)
%                 .FECFrameLength     - LDPC codeword length in bits
%                 .LDPCCodeIdentifier - code rate as a string, e.g. "1/4"
%                 .HasPilots          - true/false
%                 .PLSDecimalCode     - decoded 7-bit PLSC value (0-127)
%                 .IsDummyFrame       - true if PLSDecimalCode < 4 (a
%                                       plain-DVB-S2 concept per the
%                                       spec's dummy-PLFRAME reservation
%                                       -- unlike MathWorks'
%                                       TimeSlicingNumber and
%                                       CanonicalMODCODName, which are
%                                       DVB-S2X/wideband-only and have
%                                       no meaning here, so this
%                                       function omits them entirely)
%               Plus a decode-confidence diagnostic, useful for judging
%               how marginal a given header lock was:
%                 .DecodeMinDistance  - Euclidean distance of the
%                                       best-matching candidate codeword
%                 .DecodeMeanDistance - mean distance across every
%                                       candidate codeword in the
%                                       legacy DVB-S2 table (a large gap
%                                       between these two means a
%                                       confident, unambiguous decode;
%                                       a small gap flags a marginal one)

    validateattributes(rxPLHeader, {'double','single'}, ...
        {'nonnan', 'finite', 'nonempty', 'column'}, mfilename, 'rxPLHeader');

    if length(rxPLHeader) ~= 90
        error('dvbs2PLHeaderRecover:InvalidHeaderLength', ...
            'rxPLHeader must be 90 symbols long for DVB-S2 (got %d).', length(rxPLHeader));
    end

    rxPLSCode = rxPLHeader(27:end);   % 64-symbol PLSC field, after the 26-symbol SOF

    % Same underlying decoder MathWorks' dvbsPLHeaderRecover calls,
    % invoked ONLY in its legacy-DVB-S2 mode (isS2X = false) -- see the
    % WHY note above for the rationale.
    [modOrder, ~, fecFrameLength, hasPilots, ldpcCodeIdentifier, decodedPLSDecimalCode, distMet] = ...
        satcom.internal.dvbs.plHeaderRecover(rxPLSCode, false, true);

    phyParams.ModulationOrder = modOrder;
    phyParams.FECFrameLength = fecFrameLength;
    phyParams.LDPCCodeIdentifier = string(ldpcCodeIdentifier);
    phyParams.HasPilots = logical(hasPilots);
    phyParams.PLSDecimalCode = cast(decodedPLSDecimalCode, class(rxPLHeader));
    phyParams.IsDummyFrame = decodedPLSDecimalCode < 4;

    phyParams.DecodeMinDistance = min(distMet);
    phyParams.DecodeMeanDistance = mean(distMet);
end
