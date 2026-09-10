function ok = dvbs2SendRetransmitRequest(retransmitClient, startIdx, endIdx)
%DVBS2SENDRETRANSMITREQUEST Send one selective-repeat ARQ request to the transmitter.
%
%   dvbs2SendRetransmitRequest(retransmitClient, startIdx, endIdx) packs
%   and sends a request for global packet indices STARTIDX..ENDIDX
%   (inclusive) over RETRANSMITCLIENT (the tcpclient connection to the
%   transmitter's retransmit-request server, see
%   Functions/dvbs2TestbedConfig.m).
%
%   tcpclient objects don't expose a Connected property to check before
%   writing (unlike tcpserver -- see dvbs2TCPIsConnected.m), so instead
%   of a pre-check this just attempts the write and catches any
%   failure (e.g. the remote side not reachable, or the connection
%   dropped since it was opened), logging a warning rather than letting
%   the caller's processing loop crash over a failed retransmit request
%   -- a missed request just means that gap gets picked up again by the
%   periodic recheck on the receiving side.
%
%   Used from three call sites -- gap detection, a packet's own CRC-8
%   failure, and the periodic re-check of still-outstanding indices --
%   so the pack+guard+write logic lives in one place instead of being
%   duplicated three times.

%   OK is returned so the caller can COUNT failures instead of relying on
%   the warning below being noticed. A run prints over a thousand frame
%   lines, and a warning buried among them is effectively invisible --
%   which is precisely the situation this is being used to diagnose. S3
%   reported 82 requests "sent" while S2a read 0, and until now "sent"
%   meant only that the call was made, not that the write succeeded.

    ok = true;
    try
        dvbs2TCPFrameWrite(retransmitClient, dvbs2SerializeRetransmitRequest(startIdx, endIdx));
    catch ME
        ok = false;
        warning('dvbs2SendRetransmitRequest:WriteFailed', ...
            'Could not send retransmit request [%d,%d]: %s', startIdx, endIdx, ME.message);
    end
end
