%UPLINKRX Standalone CCSDS Telecommand uplink receiver.
%
%   Receives on 500 MHz and runs the full eight-stage chain -- DC
%   suppression, coarse carrier acquisition by squaring, RRC matched filter,
%   start-sequence correlation, Costas loop, derandomization, LDPC decoding,
%   command recovery -- printing each command UplinkTx.m sends together with
%   the numbers that say how healthy the link is.
%
%   WHAT TO READ FIRST, IN ORDER. There are two independent detection
%   metrics and they fail differently, which is what makes the link
%   diagnosable without a spectrum analyser. All figures below are measured,
%   in simulation, at the configured rate and coding.
%
%   SQUARED-SPECTRUM PEAK (stage 2) -- "is there a BPSK signal at all"
%       13 - 15 dB      nothing there. This is the noise baseline, not a
%                       weak signal: it is max/median of a Rayleigh
%                       magnitude over the search band. Check gain, centre
%                       frequency, cabling, and that UplinkTx.m is running.
%       16 dB           a burst at about 0 dB Es/No -- right at the limit
%       20 dB           about 2 dB Es/No
%       26 dB           about 6 dB Es/No
%       40 dB           about 16 dB Es/No, plenty of margin
%
%   START CORRELATION (stage 4) -- "is that signal actually a CLTU"
%       up to 0.39      noise. The largest value pure noise produced over
%                       many windows.
%       0.50            the threshold
%       0.73            a burst at 0 dB Es/No
%       0.90            6 dB
%       0.99            16 dB
%
%   READING THEM TOGETHER is the point:
%
%       both low                nothing is arriving
%       peak high, corr low     something is on the air but it is not this
%                               waveform, or the offset is outside
%                               maxCarrierOffsetHz so stage 2 locked onto
%                               the wrong thing
%       both high, no decodes   the burst is being found and framed but the
%                               codeblock will not close. Check Es/No -- and
%                               note the link needs only about 1 dB
%       both high, decodes      working
%
%   That progression is the reason this script exists. In the integrated
%   testbed a broken uplink and a broken downlink produce the same symptom,
%   "0 decoded", and nothing distinguishes them. Here the receiver reports
%   how close the link is even when nothing decodes at all.
%
%   Stop with Ctrl+C.

clear; clc;

addpath(genpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Functions')));

config = dvbs2TestbedConfig();
u = config.uplink;

% Both of these hold state across calls -- a sample buffer, a filter state,
% a duplicate-suppression history. Cleared so a fresh run cannot inherit any
% of it from a previous one.
clear ccsdsUplinkReceive ccsdsUplinkDCSuppress;

fr = ccsdsUplinkFraming(config);
fprintf('UplinkRx: CCSDS TC, coherent BPSK, %s(%d,%d), randomizer on\n', ...
    u.channelCoding, fr.codewordLength, fr.infoLength);
fprintf('UplinkRx: %g sym/s, %g samples/symbol, %.1f ksps, RRC rolloff %.2f\n', ...
    u.symbolRate, u.samplesPerSymbol, u.sampleRate/1e3, u.rolloffFactor);
fprintf('UplinkRx: CLTU %d symbols, search window %d step %d, offset search +-%.1f kHz\n', ...
    fr.cltuSymbols, u.searchWindowSamples, u.searchStrideSamples, u.maxCarrierOffsetHz/1e3);
fprintf('UplinkRx: gates -- stage 2 %g dB, stage 4 %.2f\n\n', ...
    u.detectThresholdDB, u.asmThreshold);

radioRx = comm.SDRuReceiver( ...
    'Platform', config.usrp.platform, ...
    'IPAddress', u.rxIPAddress, ...
    'MasterClockRate', config.usrp.masterClockRate, ...
    'DecimationFactor', u.inter_decimateFactor, ...
    'CenterFrequency', u.centerFrequency, ...
    'Gain', u.rxGain, ...
    'OutputDataType', 'double', ...
    'SamplesPerFrame', u.rxFrameLength);
cleanupRx = onCleanup(@() release(radioRx));
fprintf('UplinkRx: receiving on %s at %.3f MHz, gain %g dB\n\n', ...
    u.rxIPAddress, u.centerFrequency/1e6, u.rxGain);

nReports = 0;
nRequests = 0;
nUnknown = 0;
nParityFail = 0;
nASMLocks = 0;
overruns = 0;
lastCount = NaN;
missed = 0;
bestPeak = -Inf;
bestCorr = 0;
lastEsNo = NaN;
lastOffset = NaN;
lastResidual = NaN;
firstDecode = false;
runTic = tic;
lastReport = tic;

% Same split S1 uses, so the two are directly comparable. The number that
% matters is MS PER READ: in the integrated system S1 measured 32.6 ms per
% 8192-sample read while S2a's much larger reads cost 2.9 ms. If reads are
% cheap HERE, where nothing else competes for the radio, then the cost in S1
% comes from sharing one USRP with a transmit stream. If they are expensive
% here too, it is inherent to the read and the uplink receiver cannot live in
% a process that also has a real-time deadline.
prof = struct('radioRead', 0, 'dsp', 0, 'reads', 0, 'emptyReads', 0);

while true

    if toc(runTic) >= config.runDurationSec
        wall = toc(runTic);
        fprintf('\n=== UplinkRx PROFILE === %.1f s wall\n', wall);
        fprintf('  radio reads   %7.2f s  %5.1f%%   %d reads (%d empty), %.1f ms each\n', ...
            prof.radioRead, 100*prof.radioRead/wall, prof.reads, prof.emptyReads, ...
            1e3*prof.radioRead/max(prof.reads,1));
        fprintf('  uplink DSP    %7.2f s  %5.1f%%\n', prof.dsp, 100*prof.dsp/wall);
        fprintf('  everything else %5.2f s  %5.1f%%\n', ...
            wall - prof.radioRead - prof.dsp, ...
            100*(wall - prof.radioRead - prof.dsp)/wall);
        fprintf('  --\n  %d reports, %d requests, %d overruns\n', ...
            nReports, nRequests, overruns);
        fprintf('  one frame of %d samples at %g sps is %.1f ms of airtime\n\n', ...
            u.rxFrameLength, u.sampleRate, 1e3*u.rxFrameLength/u.sampleRate);
        break;
    end
    % Drain the radio. Anything left in its buffer between reads is lost to
    % an overrun, and a lost burst is a lost message -- there is no
    % retransmission at this layer.
    for k = 1:u.maxDrainReads
        tRead = tic;
        [rxSamples, validLen, overrun] = radioRx();
        prof.radioRead = prof.radioRead + toc(tRead);
        prof.reads = prof.reads + 1;
        if overrun
            overruns = overruns + 1;
        end
        if validLen == 0
            prof.emptyReads = prof.emptyReads + 1;
            break;
        end

        tDSP = tic;
        [cmds, info] = ccsdsUplinkReceive(rxSamples(1:validLen), config);
        prof.dsp = prof.dsp + toc(tDSP);

        nASMLocks = nASMLocks + info.asmLocks;
        nParityFail = nParityFail + info.parityFails;
        bestPeak = max(bestPeak, info.bestCarrierDB);
        bestCorr = max(bestCorr, info.bestASMMetric);
        if isfinite(info.esNodB),     lastEsNo = info.esNodB;         end
        if isfinite(info.offsetHz),   lastOffset = info.offsetHz;     end
        if isfinite(info.residualHz), lastResidual = info.residualHz; end

        for m = 1:numel(cmds)
            c = cmds{m};
            if ~firstDecode
                firstDecode = true;
                fprintf('UplinkRx: *** FIRST COMMAND DECODED *** peak %.1f dB, corr %.2f, Es/No %.1f dB, offset %+.0f Hz\n', ...
                    info.bestCarrierDB, info.bestASMMetric, lastEsNo, lastOffset);
            end
            switch c.Type
                case "report"
                    nReports = nReports + 1;
                    % UplinkTx steps the report count by 2 each time it
                    % sends one. A larger jump is a burst that did not make
                    % it -- which is the only way to see a LOST message,
                    % since a message that never arrives leaves no other
                    % trace.
                    if ~isnan(lastCount)
                        gap = mod(c.Count - lastCount, 200);
                        if gap > 2
                            missed = missed + (gap/2 - 1);
                            fprintf(2, 'UplinkRx: missed %d report(s) between count %d and %d\n', ...
                                gap/2 - 1, lastCount, c.Count);
                        end
                    end
                    lastCount = c.Count;
                    fprintf('UplinkRx: report  count=%3d mean=%+6.2f sigma=%4.2f rssi=%+6.2f dB\n', ...
                        c.Count, c.MeanSNRdB, c.SigmaSNRdB, c.RSSIdB);
                case "request"
                    nRequests = nRequests + 1;
                    fprintf('UplinkRx: request [%d..%d]  (%d frame(s))\n', ...
                        c.StartIdx, c.EndIdx, c.EndIdx - c.StartIdx + 1);
                otherwise
                    % Parity passed but the body made no sense. Should be
                    % vanishingly rare -- 64 parity equations all satisfied
                    % by chance is a 2^-64 event -- so if this climbs, the
                    % two ends disagree about the message format rather than
                    % the channel being bad.
                    nUnknown = nUnknown + 1;
                    fprintf(2, 'UplinkRx: decoded but unrecognised: %s\n', c.Reason);
            end
        end
    end

    if toc(lastReport) >= 2.0
        lastReport = tic;
        if ~firstDecode
            % Nothing decoded yet. Report both detectors instead -- together
            % they say how close the link is, and which stage is short.
            fprintf('UplinkRx: %5.1f s | NO DECODES | peak %5.1f dB | corr %.2f | ASM locks %d | parity fails %d | overruns %d\n', ...
                toc(runTic), bestPeak, bestCorr, nASMLocks, nParityFail, overruns);
        else
            fprintf('UplinkRx: %5.1f s | %d reports %d requests | missed %d | Es/No %4.1f dB | offset %+7.0f Hz | resid %+5.1f Hz | parity fails %d | overruns %d\n', ...
                toc(runTic), nReports, nRequests, missed, lastEsNo, ...
                lastOffset, lastResidual, nParityFail, overruns);
            if nUnknown > 0
                fprintf('          (%d frame(s) decoded but were not a known command)\n', nUnknown);
            end
        end
        bestPeak = -Inf;
        bestCorr = 0;
    end
end
