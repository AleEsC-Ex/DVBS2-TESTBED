function [commands, info] = ccsdsUplinkReceive(newSamples, config)
%CCSDSUPLINKRECEIVE The whole CCSDS Telecommand uplink receiver, one call.
%
%   [commands, info] = ccsdsUplinkReceive(newSamples, config) accepts one
%   read from the uplink radio, adds it to an internal buffer, and returns
%   every command recovered from it as a cell array of structs from
%   ccsdsUplinkParseCommand.m ("report", "request" or "unknown").
%
%   Call `clear ccsdsUplinkReceive ccsdsUplinkAcquire ccsdsUplinkDCSuppress`
%   to reset between runs; all three hold state across calls.
%
%   A THIN WRAPPER, NOT WHERE THE CHAIN LIVES ANY MORE. This function used
%   to own all eight stages directly. It now just calls
%   ccsdsUplinkAcquire.m for stages 1-5 (acquisition: DC suppression,
%   coarse CFO, matched filter, optional Gardner timing, ASM detection,
%   Costas tracking) and runs stages 6-8 (derandomize + LDPC decode via
%   ccsdsUplinkDecodeCodeblock.m, then command recovery via
%   ccsdsUplinkParseCommand.m) on whatever CLTUs it returns. See
%   ccsdsUplinkAcquire.m's own header for the full chain and the reasoning
%   behind the windowing.
%
%   WHY THIS STILL EXISTS, GIVEN THE SPLIT. The testbed's live processes
%   (S1a_Transmitter.m / S1b_ACMControl.m) call ccsdsUplinkAcquire.m and
%   ccsdsUplinkDecodeCodeblock.m directly, across the process boundary,
%   because acquisition runs close to the radio and decode runs wherever
%   the rest of the control logic lives. The standalone sdr_test/UplinkRx.m
%   and UplinkRxTest.m bring-up scripts have no process boundary to split
%   across, so this one-call version is what they use instead.

    [cltus, aInfo] = ccsdsUplinkAcquire(newSamples, config);

    commands = {};
    info = struct('windows', aInfo.windows, 'detections', aInfo.detections, ...
        'asmLocks', aInfo.asmLocks, 'decodes', 0, 'parityFails', 0, ...
        'duplicates', aInfo.duplicates, 'bestCarrierDB', aInfo.bestCarrierDB, ...
        'bestASMMetric', aInfo.bestASMMetric, 'offsetHz', NaN, ...
        'esNodB', NaN, 'residualHz', NaN, 'tailMetric', NaN, ...
        'dcLevel', aInfo.dcLevel, 'buffered', aInfo.buffered, 'payloads', {{}});

    for k = 1:numel(cltus)
        c = cltus{k};

        %% Stages 6 and 7 -- derandomize, LDPC decode
        [payload, ok, ~] = ccsdsUplinkDecodeCodeblock( ...
            c.codeSyms, c.amplitude, c.noiseVar, config);

        info.offsetHz = c.offsetHz;
        info.esNodB = c.esNodB;
        info.residualHz = c.residualHz;
        info.tailMetric = c.tailMetric;

        if ok
            info.decodes = info.decodes + 1;
            info.payloads{end+1} = payload;
            %% Stage 8 -- command recovery
            commands{end+1} = ccsdsUplinkParseCommand(payload); %#ok<AGROW>
        else
            info.parityFails = info.parityFails + 1;
        end
    end
end
