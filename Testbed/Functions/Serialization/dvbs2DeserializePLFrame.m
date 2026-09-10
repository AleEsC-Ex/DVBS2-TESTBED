function [frameSeqNum, plSymbols_corrected, phyParams, frameLength, noiseVarEstimate] = dvbs2DeserializePLFrame(payloadBytes)
%DVBS2DESERIALIZEPLFRAME Unpack one PLFRAME message (see dvbs2SerializePLFrame.m).
%
%   [frameSeqNum, plSymbols_corrected, phyParams, frameLength, noiseVarEstimate] = dvbs2DeserializePLFrame(payloadBytes)
%   reconstructs a phyParams struct with the same field names
%   AEC_dvbs2BitRecover.m expects (ModulationOrder, FECFrameLength,
%   HasPilots, LDPCCodeIdentifier), so it can be passed straight through
%   unmodified once received.

    payloadBytes = uint8(payloadBytes(:).');
    pos = 1;

    pos = pos + 1;   % version byte, reserved for future format changes

    frameSeqNum = double(typecast(payloadBytes(pos:pos+3), 'uint32')); pos = pos + 4;
    N = double(typecast(payloadBytes(pos:pos+3), 'uint32')); pos = pos + 4;

    iqBytes = payloadBytes(pos:pos + 16*N - 1); pos = pos + 16*N;
    iq = typecast(iqBytes, 'double');
    plSymbols_corrected = complex(iq(1:2:end), iq(2:2:end));
    plSymbols_corrected = plSymbols_corrected(:);

    phyParams.ModulationOrder = double(typecast(payloadBytes(pos:pos+3), 'uint32')); pos = pos + 4;
    phyParams.FECFrameLength = double(typecast(payloadBytes(pos:pos+3), 'uint32')); pos = pos + 4;
    phyParams.HasPilots = logical(payloadBytes(pos)); pos = pos + 1;
    ldpcNum = double(payloadBytes(pos)); pos = pos + 1;
    ldpcDen = double(payloadBytes(pos)); pos = pos + 1;
    phyParams.LDPCCodeIdentifier = sprintf('%d/%d', ldpcNum, ldpcDen);
    phyParams.PLSDecimalCode = double(payloadBytes(pos)); pos = pos + 1;
    frameLength = double(typecast(payloadBytes(pos:pos+3), 'uint32')); pos = pos + 4;
    noiseVarEstimate = typecast(payloadBytes(pos:pos+7), 'double');
end
