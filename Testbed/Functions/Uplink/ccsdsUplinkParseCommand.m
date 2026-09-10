function cmd = ccsdsUplinkParseCommand(payloadBytes)
%CCSDSUPLINKPARSECOMMAND Decode one uplink command payload.
%
%   cmd = ccsdsUplinkParseCommand(payloadBytes) returns a struct whose
%   .Type field is "report", "request" or "unknown".
%
%     report    .Count .MeanSNRdB .SigmaSNRdB .RSSIdB
%     request   .StartIdx .EndIdx
%     unknown   .Reason -- why it could not be read
%
%   NEVER THROWS. A decoded CLTU is untrusted input: LDPC and BCH both
%   miscorrect occasionally, so a frame can arrive intact-looking with a
%   nonsense type field or a truncated body. This is called from the
%   transmitter's main loop, where an exception would take down the link,
%   so every failure comes back as "unknown" instead.
%
%   A CLTU decodes to a whole codeword's information block -- 64 bits for
%   LDPC(128,64) -- so a 5-byte command arrives with 3 bytes of fill after
%   it. CCSDS carries no length field (the tail sequence marks the end
%   instead), so the reader is expected to know its own message sizes. The
%   deserializers below read their fixed prefix and ignore the rest.

    cmd = struct('Type', "unknown", 'Reason', "");

    if isempty(payloadBytes)
        cmd.Reason = "empty payload";
        return;
    end

    try
        msgType = dvbs2MessageType(payloadBytes);
    catch err
        cmd.Reason = string(err.message);
        return;
    end

    switch msgType
        case 0
            try
                fb = dvbs2DeserializeFeedback(payloadBytes);
            catch err
                cmd.Reason = "malformed report: " + string(err.message);
                return;
            end
            cmd = struct('Type', "report", ...
                'Count', fb.Count, ...
                'MeanSNRdB', fb.MeanSNRdB, ...
                'SigmaSNRdB', fb.SigmaSNRdB, ...
                'RSSIdB', fb.RSSIdB);

        case 1
            try
                rq = dvbs2DeserializeRetransmitRequest(payloadBytes);
            catch err
                cmd.Reason = "malformed request: " + string(err.message);
                return;
            end
            cmd = struct('Type', "request", ...
                'StartIdx', rq.StartIdx, ...
                'EndIdx', rq.EndIdx);

        otherwise
            cmd.Reason = sprintf("unknown message type %d", msgType);
    end
end
