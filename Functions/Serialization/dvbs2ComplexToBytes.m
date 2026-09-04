function bytesOut = dvbs2ComplexToBytes(samples)
%DVBS2COMPLEXTOBYTES Pack a complex sample vector as interleaved real/imag double bytes.
%
%   bytesOut = dvbs2ComplexToBytes(samples) encodes SAMPLES (a complex
%   column/row vector) as a uint8 byte stream of interleaved
%   [real1 imag1 real2 imag2 ...] doubles (16 bytes per sample). Used
%   both by dvbs2SerializePLFrame.m and by the sim-mode
%   transmitter-to-receiver raw-sample TCP link, which streams
%   continuous samples rather than discrete framed messages.
%
%   See dvbs2BytesToComplex.m for the matching decode.

    samples = samples(:);
    iq = zeros(2*numel(samples), 1);
    iq(1:2:end) = real(samples);
    iq(2:2:end) = imag(samples);
    % Force row orientation: typecast's output shape follows its input's,
    % and this function's output is meant to concatenate with other
    % row-vector byte pieces (see dvbs2SerializePLFrame.m).
    bytesOut = typecast(double(iq), 'uint8').';
end
