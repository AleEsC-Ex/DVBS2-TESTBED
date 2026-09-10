function payloadBytes = dvbs2TCPFrameRead(conn)
%DVBS2TCPFRAMEREAD Blocking read of one length-prefixed message from a TCP connection.
%
%   payloadBytes = dvbs2TCPFrameRead(conn) blocks (polling
%   conn.NumBytesAvailable) until one complete length-prefixed message
%   written by dvbs2TCPFrameWrite.m has fully arrived on CONN, then
%   returns just the payload as a uint8 row vector, with the 4-byte
%   length prefix consumed and discarded. TCP is a byte stream, not a
%   message-framed protocol -- a single read() call is never guaranteed
%   to return exactly one message's worth of data, so this function
%   polls in two stages: first until the 4-byte length prefix itself
%   has arrived, then again until the full payload it describes has.
%
%   If the remote side disconnects before a full message arrives, this
%   throws dvbs2TCPFrameRead:ConnectionClosed instead of polling
%   forever -- callers running a receive loop against a finite/bounded
%   sender should catch this specific identifier to exit cleanly once
%   the sender stops.

    while conn.NumBytesAvailable < 4
        if ~dvbs2TCPIsConnected(conn)
            error('dvbs2TCPFrameRead:ConnectionClosed', ...
                'Connection closed by the remote side before a message arrived.');
        end
        pause(0.001);
    end
    lenPrefix = read(conn, 4, 'uint8');
    msgLen = double(typecast(uint8(lenPrefix), 'uint32'));

    while conn.NumBytesAvailable < msgLen
        if ~dvbs2TCPIsConnected(conn)
            error('dvbs2TCPFrameRead:ConnectionClosed', ...
                'Connection closed by the remote side mid-message.');
        end
        pause(0.001);
    end
    payloadBytes = read(conn, msgLen, 'uint8');
    payloadBytes = uint8(payloadBytes(:).');
end
