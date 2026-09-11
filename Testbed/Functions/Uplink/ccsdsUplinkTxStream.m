function [samples, info] = ccsdsUplinkTxStream(nSamples, newPayloads, config)
%CCSDSUPLINKTXSTREAM The PLOP-2 transmit state machine.
%
%   [samples, info] = ccsdsUplinkTxStream(nSamples, newPayloads, config)
%   returns the next NSAMPLES of the continuous uplink stream, after queueing
%   any payloads in NEWPAYLOADS (a cell array of uint8 vectors; pass {} to
%   queue nothing).
%
%   info fields:
%     .element      what was generated during this call, last first:
%                   "acquisition", "cltu" or "idle"
%     .cltusSent    CLTUs emitted since the session began
%     .queued       payloads still waiting
%     .idleSymbols  idle symbols emitted since the last CLTU
%     .buffered     shaped samples held over for the next call
%     .peak         largest magnitude in this block -- watch for clipping
%
%   Call `clear ccsdsUplinkTxStream ccsdsUplinkPulseShape` to start a new
%   session; both hold state.
%
%   THE PROCEDURE IT IMPLEMENTS. CCSDS 231.0-B calls this PLOP-2:
%
%     carrier on -> acquisition sequence -> CLTU -> idle -> CLTU -> idle ...
%
%   The acquisition sequence is sent ONCE, at the start of the session. After
%   that the transmitter alternates between CLTUs and idle, and critically it
%   NEVER STOPS TRANSMITTING. When no command is queued it emits idle
%   indefinitely.
%
%   WHY THAT IS THE ENTIRE POINT. The old transmitter sent a CLTU and then
%   7500 samples of zeros -- the carrier dropped between commands. A receiver
%   facing that has nothing to hold its loops on, so it must re-acquire
%   carrier and symbol timing from scratch for every single message, from a
%   standing start, in the 40 ms the burst lasts. That is a hard problem and
%   it is a self-inflicted one. Continuous transmission means the loops lock
%   ONCE, on the acquisition sequence, and then only have to track -- which
%   is what loops are good at.
%
%   HOW A COMMAND GETS OUT. Queue it, and it goes on the air as soon as the
%   minimum idle since the previous CLTU has been met. That floor exists so
%   two CLTUs can never run together with no separation; three octets is
%   enough to be unambiguous without costing airtime that idle would have
%   used anyway.
%
%   WHY IDLE IS GENERATED IN TWO DIFFERENT SIZES. With nothing queued it is
%   produced in chunks of idleChunkSymbols, purely so the waveform generator
%   is called a few times a second rather than hundreds. With a command
%   waiting, only the idle still OWED by minIdleSymbols is produced, so the
%   command goes out at the earliest legal moment instead of waiting for a
%   whole chunk to finish. The two cases together give low overhead when the
%   link is quiet and low latency when it is not.
%
%   THE OUTPUT IS SAMPLE-BUFFERED, not symbol-buffered, so NSAMPLES need not
%   be a whole number of symbols. Whatever the shaper produces beyond this
%   call's request is held for the next one.

    persistent sampBuf queue started idleSinceCLTU cltusSent lastElement

    % INITIALISE ON `started`, NOT ON `sampBuf`. The obvious sentinel --
    % isempty(sampBuf) -- is wrong, and wrong in a way that only shows up
    % occasionally: the buffer legitimately becomes empty whenever a block
    % consumes it exactly, and treating that as a first call wipes the
    % command queue, resets the CLTU count and re-sends the acquisition
    % sequence in the middle of a session. Measured before this fix: 3
    % commands queued, 1 transmitted. `started` is false-but-not-empty after
    % the first call, so it distinguishes the two states properly.
    if isempty(started)
        sampBuf = complex(zeros(0,1));
        queue = {};
        started = false;
        % Set so the first CLTU may follow the acquisition sequence
        % immediately -- PLOP-2 puts no idle between them.
        idleSinceCLTU = Inf;
        cltusSent = 0;
        lastElement = "";
    end

    if nargin >= 2 && ~isempty(newPayloads)
        if ~iscell(newPayloads)
            newPayloads = {newPayloads};
        end
        queue = [queue, newPayloads(:).'];
    end

    p = config.uplink.plop;

    while numel(sampBuf) < nSamples
        if ~started
            syms = ccsdsUplinkSymbols("acquisition", p.acquisitionSymbols, config);
            started = true;
            lastElement = "acquisition";

        elseif ~isempty(queue) && idleSinceCLTU >= p.minIdleSymbols
            syms = ccsdsUplinkSymbols("cltu", queue{1}, config);
            queue(1) = [];
            idleSinceCLTU = 0;
            cltusSent = cltusSent + 1;
            lastElement = "cltu";

        else
            if isempty(queue)
                nIdle = p.idleChunkSymbols;
            else
                % Only what is still owed, so the queued command goes out at
                % the earliest legal moment.
                nIdle = max(1, p.minIdleSymbols - idleSinceCLTU);
            end
            syms = ccsdsUplinkSymbols("idle", nIdle, config);
            idleSinceCLTU = idleSinceCLTU + nIdle;
            lastElement = "idle";
        end

        sampBuf = [sampBuf; ccsdsUplinkPulseShape(syms, config)]; %#ok<AGROW>
    end

    samples = sampBuf(1:nSamples);
    sampBuf = sampBuf(nSamples+1:end);

    % lastElement persists, so a call served entirely from the buffer still
    % reports what the stream is currently emitting rather than "".
    info = struct( ...
        'element', lastElement, ...
        'cltusSent', cltusSent, ...
        'queued', numel(queue), ...
        'idleSymbols', idleSinceCLTU, ...
        'buffered', numel(sampBuf), ...
        'peak', max(abs(samples)));
end
