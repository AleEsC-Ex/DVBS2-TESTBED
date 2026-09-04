function payloadBytes = ccsdsUplinkCommand(varargin)
%CCSDSUPLINKCOMMAND Build one uplink command payload.
%
%   The uplink carries exactly two commands, both 5 bytes:
%
%   REPORT -- link quality measured by the receiving side
%     bytes = ccsdsUplinkCommand("report", count, meanSNRdB, sigmaSNRdB, rssiDB)
%
%       count       frames the report summarises, 0..255. ZERO IS MEANINGFUL
%                   and says "alive, but decoded nothing" -- a distinct
%                   condition from silence and a more alarming one.
%       meanSNRdB   +-32 dB in steps of 0.0625
%       sigmaSNRdB  0..16 dB in steps of 0.0625
%       rssiDB      -96..0 dB in steps of 0.125
%
%     Statistics rather than samples, so the message stays 5 bytes whether
%     it summarises 5 frames or 75. Mean and standard deviation recombine
%     exactly across batches, so the transmitter's policy loses nothing by
%     receiving summaries -- see dvbs2SerializeFeedback.m.
%
%   REQUEST -- retransmission of a frame or a range of frames
%     bytes = ccsdsUplinkCommand("request", startIdx)
%     bytes = ccsdsUplinkCommand("request", startIdx, endIdx)
%
%       startIdx    first global packet index wanted, 0..16777215
%       endIdx      last one, inclusive. Omit for a single frame.
%
%     The span travels as a 13-bit COUNT rather than a second absolute
%     index, which makes an oversized request unrepresentable rather than
%     merely rejected on arrival -- a corrupted index cannot ask the
%     transmitter to build a two-billion-element range.
%
%   Both fit in one LDPC(128,64) codeword, which carries 64 information
%   bits, so either command costs exactly the same airtime.
%
%   See ccsdsUplinkParseCommand.m for the matching decode.

    if nargin < 1
        error('ccsdsUplinkCommand:NoCommand', 'Specify "report" or "request".');
    end
    cmd = string(varargin{1});

    switch lower(cmd)
        case "report"
            if nargin ~= 5
                error('ccsdsUplinkCommand:ReportArgs', ...
                    'report takes count, meanSNRdB, sigmaSNRdB, rssiDB.');
            end
            feedback.Count = varargin{2};
            feedback.MeanSNRdB = varargin{3};
            feedback.SigmaSNRdB = varargin{4};
            feedback.RSSIdB = varargin{5};
            payloadBytes = dvbs2SerializeFeedback(feedback);

        case "request"
            if nargin == 2
                startIdx = varargin{2};
                endIdx = startIdx;
            elseif nargin == 3
                startIdx = varargin{2};
                endIdx = varargin{3};
            else
                error('ccsdsUplinkCommand:RequestArgs', ...
                    'request takes startIdx, and optionally endIdx.');
            end
            payloadBytes = dvbs2SerializeRetransmitRequest(startIdx, endIdx);

        otherwise
            error('ccsdsUplinkCommand:UnknownCommand', ...
                'Unknown command "%s"; expected "report" or "request".', cmd);
    end
end
