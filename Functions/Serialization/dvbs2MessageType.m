function msgType = dvbs2MessageType(payloadBytes)
%DVBS2MESSAGETYPE Read the 3-bit type field at the head of a control message.
%
%   msgType = dvbs2MessageType(payloadBytes) returns 0 for an ACM feedback
%   report (dvbs2SerializeFeedback.m) and 1 for an ARQ retransmit request
%   (dvbs2SerializeRetransmitRequest.m).
%
%   On the TCP links each message type had its own socket, so the receiver
%   always knew what it was reading. The return link carries both types over
%   ONE channel, so the type has to be read before the message can be handed
%   to the right deserializer -- which is exactly why both layouts start
%   with this field.
%
%   Both serializers put it in the top 3 bits of the first byte, so this
%   never needs to unpack the whole message.

    if isempty(payloadBytes)
        error('dvbs2MessageType:Empty', 'Message is empty.');
    end
    msgType = double(bitshift(uint8(payloadBytes(1)), -5));
end
