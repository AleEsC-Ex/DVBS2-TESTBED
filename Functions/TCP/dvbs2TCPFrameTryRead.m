function payloadBytes = dvbs2TCPFrameTryRead(conn)
%DVBS2TCPFRAMETRYREAD Non-blocking check for one length-prefixed TCP message.
%
%   payloadBytes = dvbs2TCPFrameTryRead(conn) returns the next complete
%   message written by dvbs2TCPFrameWrite.m if one has fully arrived on
%   CONN already, or [] immediately if not (no polling/blocking) --
%   unlike dvbs2TCPFrameRead.m, which blocks until a message arrives.
%   Use this where the caller must keep doing other work while checking
%   (e.g. polling for feedback between transmit bursts without stalling
%   transmission).

    payloadBytes = [];
    if isempty(conn) || ~isvalid(conn) || conn.NumBytesAvailable < 4
        return;
    end

    lenPrefix = read(conn, 4, 'uint8');
    msgLen = double(typecast(uint8(lenPrefix), 'uint32'));

    % The length prefix has already been consumed at this point; if the
    % rest of the payload hasn't arrived yet, block briefly for it
    % rather than losing the already-read prefix (a short, bounded wait
    % here is preferable to the complexity of pushing bytes back onto
    % the socket).
    timeoutSec = 1;
    t0 = tic;
    while conn.NumBytesAvailable < msgLen
        if toc(t0) > timeoutSec
            error('dvbs2TCPFrameTryRead:IncompletePayload', ...
                'Length prefix received but payload did not fully arrive within %.1f s.', timeoutSec);
        end
        pause(0.001);
    end
    payloadBytes = read(conn, msgLen, 'uint8');
    payloadBytes = uint8(payloadBytes(:).');
end
