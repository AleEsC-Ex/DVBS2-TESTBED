function cltu = dvbs2DeserializeCLTU(payloadBytes)
%DVBS2DESERIALIZECLTU Unpack one acquired CLTU (see dvbs2SerializeCLTU.m).
%
%   cltu = dvbs2DeserializeCLTU(payloadBytes) returns a struct with the
%   same fields ccsdsUplinkAcquire.m produces (.codeSyms, .amplitude,
%   .noiseVar, .esNodB, .tailMetric, .offsetHz, .residualHz), ready to hand
%   straight to ccsdsUplinkDecodeCodeblock.m.

    payloadBytes = uint8(payloadBytes(:).');
    pos = 1;

    pos = pos + 1;   % version byte, reserved for future format changes

    cltu.amplitude  = typecast(payloadBytes(pos:pos+7), 'double'); pos = pos + 8;
    cltu.noiseVar   = typecast(payloadBytes(pos:pos+7), 'double'); pos = pos + 8;
    cltu.esNodB     = typecast(payloadBytes(pos:pos+7), 'double'); pos = pos + 8;
    cltu.tailMetric = typecast(payloadBytes(pos:pos+7), 'double'); pos = pos + 8;
    cltu.offsetHz   = typecast(payloadBytes(pos:pos+7), 'double'); pos = pos + 8;
    cltu.residualHz = typecast(payloadBytes(pos:pos+7), 'double'); pos = pos + 8;
    N = double(typecast(payloadBytes(pos:pos+3), 'uint32')); pos = pos + 4;

    cltu.codeSyms = dvbs2BytesToComplex(payloadBytes(pos : pos + 16*N - 1));
end
