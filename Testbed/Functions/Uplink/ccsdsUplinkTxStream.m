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
%     .secPerCltu   measured cost to generate one CLTU, running average
%     .cltuBudgetCapped  true if this call had queued CLTUs it could not
%                   afford under config.uplink.txBudgetFraction -- a
%                   growing queue length (.queued) alongside this being
%                   true often means the budget is too tight for the
%                   current retransmit/feedback traffic, not that
%                   anything is broken
%
%   Call `clear ccsdsUplinkTxStream ccsdsUplinkPulseShape` to start a new
%   session; both hold state.
%
%   REAL CLTUs ARE BUDGET-CAPPED PER CALL (config.uplink.txBudgetFraction),
%   THE REST FILLED WITH IDLE -- the same pattern S1b's downlink waveform
%   generator already uses (config.tx.genBudgetFraction) and for the same
%   reason: LDPC-encoding a real CLTU costs meaningfully more than
%   generating idle filler of the same length, and that cost scales with
%   retransmit/feedback traffic -- which is exactly what climbs when the
%   link is already struggling. Left uncapped, a growing backlog makes
%   every call more expensive right when the caller (S2a, sharing this
%   loop with its OWN downlink receive radio) can least afford it. See the
%   config field's own comment for the hardware numbers that motivated
%   this. Excess queued CLTUs simply wait for a later call -- nothing is
%   dropped, only delayed.
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

    persistent sampBuf queue started idleSinceCLTU cltusSent lastElement secPerCltu

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
        % Measured cost of encoding+shaping one CLTU, running average, used
        % to turn config.uplink.txBudgetFraction into a per-call CLTU count
        % cap below. Starts at 0 (not a guessed default): with no
        % measurement yet, the cap comes out effectively unlimited for the
        % first call or two, which is harmless -- it self-corrects the
        % moment a real CLTU is actually generated and timed.
        secPerCltu = 0;
    end

    if nargin >= 2 && ~isempty(newPayloads)
        if ~iscell(newPayloads)
            newPayloads = {newPayloads};
        end
        queue = [queue, newPayloads(:).'];
    end

    p = config.uplink.plop;

    % How many real CLTUs THIS CALL may afford to generate -- see the
    % header comment and config.uplink.txBudgetFraction's own comment for
    % why this exists. max(1, ...), NOT max(0, ...) -- deliberately always
    % lets at least one through, for a reason that has nothing to do with
    % throughput: secPerCltu is a MEASURED running average, seeded at 0,
    % and the first real measurement is inflated by one-time JIT-compile
    % cost (measured directly: 79 ms against a real per-call budget of
    % 17 ms). With max(0, ...) that single bad sample makes the cap
    % collapse to zero -- and then STAYS zero forever, because a cap of
    % zero means no CLTU ever runs again, so secPerCltu can never be
    % re-measured to correct itself. A permanent starvation trap, not
    % graceful throttling. Guaranteeing one CLTU per call keeps sampling
    % secPerCltu even in the worst case, so the JIT-inflated first sample
    % gets diluted by faster, realistic ones within a few calls (the EMA
    % below), and the cap converges to something real instead of getting
    % stuck. The cost is small: one real CLTU per call is still far below
    % what the old, uncapped code could do (as many as fit in a whole
    % block), so the protection this exists for is still overwhelmingly
    % intact.
    cltuBudgetSec = (nSamples / config.uplink.sampleRate) * config.uplink.txBudgetFraction;
    maxCltusThisCall = max(1, floor(cltuBudgetSec / max(secPerCltu, eps)));
    cltusThisCall = 0;

    while numel(sampBuf) < nSamples
        if ~started
            syms = ccsdsUplinkSymbols("acquisition", p.acquisitionSymbols, config);
            started = true;
            lastElement = "acquisition";

        elseif ~isempty(queue) && idleSinceCLTU >= p.minIdleSymbols && cltusThisCall < maxCltusThisCall
            tCltu = tic;
            syms = ccsdsUplinkSymbols("cltu", queue{1}, config);
            queue(1) = [];
            idleSinceCLTU = 0;
            cltusSent = cltusSent + 1;
            cltusThisCall = cltusThisCall + 1;
            lastElement = "cltu";
            % Running average of the LDPC-encode-dominated cost that the
            % budget above is actually protecting against. Timed around
            % symbol generation alone, not the pulse-shaping call below --
            % that part costs about the same regardless of content, so the
            % differential cost a CLTU adds over idle is well captured here.
            secPerCltu = 0.8*secPerCltu + 0.2*toc(tCltu);

        else
            if isempty(queue) || cltusThisCall >= maxCltusThisCall
                % Nothing queued, OR this call's real-CLTU budget is spent
                % -- either way there is no reason to rush the next CLTU
                % out, so fill efficiently with a whole idle chunk instead
                % of just the minimum still owed.
                nIdle = p.idleChunkSymbols;
            else
                % Budget still has room; only what is still owed, so the
                % queued command goes out at the earliest legal moment.
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
        'peak', max(abs(samples)), ...
        'secPerCltu', secPerCltu, ...
        'cltuBudgetCapped', cltusThisCall >= maxCltusThisCall && ~isempty(queue));
end
