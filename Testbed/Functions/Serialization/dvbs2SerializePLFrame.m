function payloadBytes = dvbs2SerializePLFrame(frameSeqNum, plSymbols_corrected, phyParams, frameLength, noiseVarEstimate)
%DVBS2SERIALIZEPLFRAME Pack one corrected PLFRAME + its PLHEADER metadata for the receiver-to-processing-unit hand-off.
%
%   payloadBytes = dvbs2SerializePLFrame(frameSeqNum, plSymbols_corrected, phyParams, frameLength, noiseVarEstimate)
%   encodes everything the receiving side needs to run
%   AEC_dvbs2BitRecover.m on one PLFRAME, for transport over
%   dvbs2TCPFrameWrite.m, using this fixed binary layout:
%
%     [1]     uint8  version (currently 1)
%     [4]     uint32 frameSeqNum -- lets the receiving side detect/skip
%                    dropped frames and stay aligned with its locally
%                    regenerated reference bitstream for BER comparison
%     [4]     uint32 N = numel(plSymbols_corrected)
%     [16*N]  double interleaved [real1 imag1 real2 imag2 ...] for
%                    plSymbols_corrected
%     [4]     uint32 phyParams.ModulationOrder
%     [4]     uint32 phyParams.FECFrameLength
%     [1]     uint8  phyParams.HasPilots (0/1)
%     [1]     uint8  LDPC code-rate numerator   (see dvbs2ParseLDPCRate.m)
%     [1]     uint8  LDPC code-rate denominator
%     [1]     uint8  phyParams.PLSDecimalCode (0-127)
%     [4]     uint32 frameLength (full PLFRAME length incl. header/pilots)
%     [8]     double noiseVarEstimate (see dvbs2SNREstimate.m)
%
%   A fixed layout is used instead of generic MATLAB struct
%   serialization so the wire format is documented, stable across
%   MATLAB releases, and easy to inspect while debugging.
%
%   See dvbs2DeserializePLFrame.m for the matching decode.

    plSymbols_corrected = plSymbols_corrected(:);
    N = numel(plSymbols_corrected);

    [ldpcNum, ldpcDen] = dvbs2ParseLDPCRate(phyParams.LDPCCodeIdentifier);

    version = uint8(1);
    payloadBytes = [ ...
        version, ...
        typecast(uint32(frameSeqNum), 'uint8'), ...
        typecast(uint32(N), 'uint8'), ...
        dvbs2ComplexToBytes(plSymbols_corrected), ...
        typecast(uint32(phyParams.ModulationOrder), 'uint8'), ...
        typecast(uint32(phyParams.FECFrameLength), 'uint8'), ...
        uint8(logical(phyParams.HasPilots)), ...
        uint8(ldpcNum), ...
        uint8(ldpcDen), ...
        uint8(phyParams.PLSDecimalCode), ...
        typecast(uint32(frameLength), 'uint8'), ...
        typecast(double(noiseVarEstimate), 'uint8') ...
        ];
end
