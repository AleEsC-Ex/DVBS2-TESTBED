function samples = dvbs2BytesToComplex(bytesIn)
%DVBS2BYTESTOCOMPLEX Unpack interleaved real/imag double bytes into a complex column vector.
%
%   samples = dvbs2BytesToComplex(bytesIn) is the inverse of
%   dvbs2ComplexToBytes.m: bytesIn must be a uint8 vector whose length
%   is a multiple of 16 bytes (2 doubles per complex sample).

    iq = typecast(uint8(bytesIn(:).'), 'double');
    samples = complex(iq(1:2:end), iq(2:2:end));
    samples = samples(:);
end
