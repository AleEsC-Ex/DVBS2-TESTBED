function bytesOut = dvbs2PackBits(bits)
%DVBS2PACKBITS Pack a bit vector into a uint8 row, MSB first, zero-padded.
%
%   The compact control-message formats are defined in BITS rather than
%   bytes, because over the return link every bit costs link margin: frame
%   error probability compounds across the CRC-protected span, so a shorter
%   message survives a worse channel. Byte alignment is imposed only at the
%   very end, where the transport (TCP or the DBPSK modem) requires it.
%
%   Any padding needed to reach a byte boundary is zeros, and the reader
%   knows how many bits are meaningful from the message type, so the
%   padding is never ambiguous.

    bits = double(bits(:));
    nPad = mod(-numel(bits), 8);
    bits = [bits; zeros(nPad,1)];
    bytesOut = uint8(sum(reshape(bits, 8, []).' .* 2.^(7:-1:0), 2)).';
end
