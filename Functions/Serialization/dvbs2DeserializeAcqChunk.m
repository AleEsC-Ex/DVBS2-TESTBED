function [rssiDB, cfoEstHz, samples] = dvbs2DeserializeAcqChunk(payloadBytes)
%DVBS2DESERIALIZEACQCHUNK Unpack one acquisition chunk (see dvbs2SerializeAcqChunk.m).
%
%   [rssiDB, cfoEstHz, samples] = dvbs2DeserializeAcqChunk(payloadBytes)
%   returns the RSSI measured by S2a_RFAcquisition.m for this block (in
%   dB, taken before its AGC stage), the block-level raw CFO estimate it
%   already removed from these samples (in Hz, for logging only), and
%   the block's complex samples as a column vector.

    payloadBytes = uint8(payloadBytes(:).');
    pos = 1;

    pos = pos + 1;   % version byte, reserved for future format changes

    rssiDB = typecast(payloadBytes(pos:pos+7), 'double'); pos = pos + 8;
    cfoEstHz = typecast(payloadBytes(pos:pos+7), 'double'); pos = pos + 8;
    N = double(typecast(payloadBytes(pos:pos+3), 'uint32')); pos = pos + 4;

    samples = dvbs2BytesToComplex(payloadBytes(pos : pos + 16*N - 1));
end
