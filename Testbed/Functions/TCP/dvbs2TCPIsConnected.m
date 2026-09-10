function tf = dvbs2TCPIsConnected(conn)
%DVBS2TCPISCONNECTED Best-effort "is this connection still up" check, for either tcpserver or tcpclient.
%
%   tf = dvbs2TCPIsConnected(conn) returns conn.Connected for a
%   tcpserver, which exposes that property directly (true once a client
%   has connected). tcpclient does NOT expose a Connected property --
%   calling conn.Connected on one throws "Unrecognized ... 'Connected'
%   for class 'tcpclient'" -- so for a tcpclient this falls back to
%   isvalid(conn) instead, which only confirms the local object handle
%   hasn't been deleted, not that the remote side is still responsive.
%   There is no reliable way to proactively detect a tcpclient's
%   remote-side disconnect without attempting a read/write and catching
%   the resulting error (see dvbs2SendRetransmitRequest.m, which does
%   exactly that rather than relying on this function).
%
%   Written this way so code that may be handed either object type
%   (currently dvbs2TCPFrameRead.m) doesn't need its own isprop() check
%   and can't hit the tcpclient error above.

    if isempty(conn) || ~isvalid(conn)
        tf = false;
        return;
    end
    if isprop(conn, 'Connected')
        tf = conn.Connected;
    else
        tf = true;   % tcpclient: no such property -- assume up, let read/write surface a real disconnect
    end
end
