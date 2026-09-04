function dvbs2TCPFrameWrite(conn, payloadBytes)
%DVBS2TCPFRAMEWRITE Write one length-prefixed message to a TCP connection.
%
%   dvbs2TCPFrameWrite(conn, payloadBytes) writes payloadBytes (a uint8
%   vector) to CONN -- a tcpserver or tcpclient object, MATLAB's
%   stream-oriented TCP objects -- preceded by a 4-byte little-endian
%   uint32 length prefix, so the reading side (dvbs2TCPFrameRead.m) can
%   reconstruct message boundaries from what TCP otherwise delivers as
%   one continuous, unframed byte stream.
%
%   payloadBytes must already be a uint8 vector; see
%   dvbs2SerializeFeedback.m / dvbs2SerializePLFrame.m for how the
%   higher-level message payloads used in this testbed are built.

    payloadBytes = uint8(payloadBytes(:).');
    lenPrefix = typecast(uint32(numel(payloadBytes)), 'uint8');
    write(conn, [lenPrefix, payloadBytes], 'uint8');
end
