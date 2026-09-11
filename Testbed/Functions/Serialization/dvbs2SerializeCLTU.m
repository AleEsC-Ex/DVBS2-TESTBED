function payloadBytes = dvbs2SerializeCLTU(cltu)
%DVBS2SERIALIZECLTU Pack one acquired CLTU (codeword symbols + scalars) for the S1a-to-S1b link.
%
%   payloadBytes = dvbs2SerializeCLTU(cltu) encodes one CLTU struct as
%   returned by ccsdsUplinkAcquire.m (fields .codeSyms, .amplitude,
%   .noiseVar, .esNodB, .tailMetric, .offsetHz, .residualHz) for transport
%   over dvbs2TCPFrameWrite.m, using this fixed binary layout:
%
%     [1]     uint8  version (currently 1)
%     [8]     double amplitude
%     [8]     double noiseVar
%     [8]     double esNodB
%     [8]     double tailMetric
%     [8]     double offsetHz
%     [8]     double residualHz
%     [4]     uint32 N = numel(codeSyms)
%     [16*N]  double interleaved [real1 imag1 real2 imag2 ...]
%
%   amplitude and noiseVar are REQUIRED downstream -- ccsdsUplinkDecodeCodeblock.m
%   cannot scale its LLRs without them. The rest (esNodB, tailMetric,
%   offsetHz, residualHz) are diagnostics carried along only so the process
%   that finishes the decode can still report the same link-quality figures
%   S1b's profile always has, even though acquisition itself now happens in
%   a different process (S1a_Transmitter.m).
%
%   One message per DETECTED CLTU, not one per radio read -- this is the
%   whole point of moving acquisition (ccsdsUplinkAcquire.m) up to S1a: the
%   continuous, mostly-idle uplink stream is filtered down to just the
%   bursts that actually cleared the ASM correlation, before it ever
%   crosses the process boundary.
%
%   See dvbs2DeserializeCLTU.m for the matching decode.

    codeSyms = cltu.codeSyms(:);
    N = numel(codeSyms);

    version = uint8(1);
    payloadBytes = [ ...
        version, ...
        typecast(double(cltu.amplitude), 'uint8'), ...
        typecast(double(cltu.noiseVar), 'uint8'), ...
        typecast(double(cltu.esNodB), 'uint8'), ...
        typecast(double(cltu.tailMetric), 'uint8'), ...
        typecast(double(cltu.offsetHz), 'uint8'), ...
        typecast(double(cltu.residualHz), 'uint8'), ...
        typecast(uint32(N), 'uint8'), ...
        dvbs2ComplexToBytes(codeSyms) ...
        ];
end
