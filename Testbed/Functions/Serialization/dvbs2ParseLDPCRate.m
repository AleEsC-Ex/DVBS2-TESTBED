function [num, den] = dvbs2ParseLDPCRate(ldpcCodeIdentifier)
%DVBS2PARSELDPCRATE Parse a DVB-S2 LDPC code-rate identifier like "1/4" into integers.
%
%   [num, den] = dvbs2ParseLDPCRate(ldpcCodeIdentifier) splits a
%   phyParams.LDPCCodeIdentifier string/char (e.g. "1/4", as returned by
%   dvbs2PLHeaderRecover and consumed via eval() in
%   AEC_dvbs2BitRecover.m) into its numerator and denominator, so the
%   rate can be carried over the wire as two uint8 values
%   (dvbs2SerializePLFrame.m) instead of a variable-length string, and
%   reassembled with sprintf('%d/%d', num, den) on the receiving side
%   (dvbs2DeserializePLFrame.m) into a string AEC_dvbs2BitRecover.m can
%   consume unmodified.

    parsed = sscanf(char(ldpcCodeIdentifier), '%d/%d');
    num = parsed(1);
    den = parsed(2);
end
