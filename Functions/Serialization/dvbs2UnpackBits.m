function bits = dvbs2UnpackBits(bytesIn)
%DVBS2UNPACKBITS Expand a uint8 vector into a bit column, MSB first.
%
%   Inverse of dvbs2PackBits.m. Returns 8 bits per input byte; the caller
%   reads only as many as its message type defines and ignores the rest,
%   which is the zero padding added to reach a byte boundary.

    b = uint8(bytesIn(:));
    bits = double(reshape(bitget(repmat(b,1,8), repmat(8:-1:1,numel(b),1)).', [], 1));
end
