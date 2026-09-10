function payloadBytes = dvbs2SerializeAcqChunk(rssiDB, cfoEstHz, samples)
%DVBS2SERIALIZEACQCHUNK Pack one acquisition chunk (samples + its scalars) for the S2a-to-S2b link.
%
%   payloadBytes = dvbs2SerializeAcqChunk(rssiDB, cfoEstHz, samples)
%   encodes one block of front-end-processed samples together with the
%   two per-block scalars measured for it, for transport over
%   dvbs2TCPFrameWrite.m, using this fixed binary layout:
%
%     [1]     uint8  version (currently 2)
%     [8]     double rssiDB
%     [8]     double cfoEstHz -- the block-level raw CFO estimate that
%                    dvbs2RawCFOCompensate.m already REMOVED from these
%                    samples; carried for logging/diagnostics only, so
%                    the receiving side can still correlate it against
%                    per-frame PLHEADER decode results downstream.
%     [4]     uint32 N = numel(samples)
%     [16*N]  double interleaved [real1 imag1 real2 imag2 ...]
%
%   WHY THIS LINK IS FRAMED: it used to be a raw, unframed byte stream,
%   because it only ever carried a continuous run of samples that the
%   receiving side could read in arbitrary fixed-size pieces. Now that
%   S2a_RFAcquisition.m also measures RSSI (it has to -- RSSI must be
%   taken BEFORE the AGC that now lives there, since AGC deliberately
%   erases absolute power information) and estimates the block's CFO,
%   each block of samples has scalars bound to it, so they have to
%   travel together as one delimited message rather than as an
%   anonymous byte run.
%
%   A framed link also removes the previous requirement that the reading
%   side know the block size in advance: S2a's blocks can vary in length
%   (comm.SDRuReceiver returns a variable valid-sample count, and the
%   simulated channel's SCO stage can resample to a different length),
%   which an unframed stream read in fixed-size pieces could not express.
%
%   See dvbs2DeserializeAcqChunk.m for the matching decode.

    samples = samples(:);
    N = numel(samples);

    version = uint8(2);
    payloadBytes = [ ...
        version, ...
        typecast(double(rssiDB), 'uint8'), ...
        typecast(double(cfoEstHz), 'uint8'), ...
        typecast(uint32(N), 'uint8'), ...
        dvbs2ComplexToBytes(samples) ...
        ];
end
