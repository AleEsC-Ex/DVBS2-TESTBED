%UPLINKTXTEST Verify the PLOP-2 uplink transmitter without radios.
%
%   Four things, each of which can be silently wrong:
%     1. the stream really is continuous, and in the right ORDER --
%        acquisition first, then CLTUs and idle, with no gaps in the carrier
%     2. block boundaries are seamless, i.e. the shaping filter's state is
%        genuinely being carried across calls
%     3. both commands survive encode -> CLTU -> decode -> parse
%     4. the level and spectrum are what the radio and the channel expect
%
%   The receiver chain is NOT exercised here: this deliberately recovers the
%   symbols the way a PERFECT receiver would, so that a failure here means
%   the transmitter is wrong, with no possibility of the receiver being
%   blamed for it. UplinkRxTest.m does the opposite.

clear; clc;
clear ccsdsUplinkTxStream ccsdsUplinkPulseShape;
addpath(genpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Functions')));

config = dvbs2TestbedConfig();
u = config.uplink;
fr = ccsdsUplinkFraming(config);
sps = u.samplesPerSymbol;

fprintf('=== configuration ===\n');
fprintf('  %s(%d,%d), tail %d, randomizer %d\n', u.channelCoding, ...
    fr.codewordLength, fr.infoLength, u.hasTailSequence, u.hasRandomizer);
fprintf('  %g sym/s, %g samples/symbol, %.1f ksps, RRC rolloff %.2f\n', ...
    u.symbolRate, sps, u.sampleRate/1e3, u.rolloffFactor);
fprintf('  acquisition %d sym (%.0f ms) | CLTU %d sym (%.0f ms) | min idle %d sym\n', ...
    u.plop.acquisitionSymbols, 1e3*u.plop.acquisitionSymbols/u.symbolRate, ...
    fr.cltuSymbols, 1e3*fr.cltuSymbols/u.symbolRate, u.plop.minIdleSymbols);
fprintf('  occupied BW %.2f kHz, target RMS %.2f\n\n', ...
    u.symbolRate*(1+u.rolloffFactor)/1e3, u.txRMS);

%% 1. Element order and continuity
fprintf('=== 1. stream structure ===\n');
% Queue two commands up front so both a CLTU and idle appear early.
payloads = { ccsdsUplinkCommand("report", 42, 12.5, 1.25, -40), ...
             ccsdsUplinkCommand("request", 1234, 1240) };

blockSamples = u.txBlockSamples;
nBlocks = 12;
stream = complex(zeros(nBlocks*blockSamples, 1));
elements = strings(nBlocks, 1);
for b = 1:nBlocks
    if b == 1
        [blk, inf1] = ccsdsUplinkTxStream(blockSamples, payloads, config);
    elseif b == 6
        [blk, inf1] = ccsdsUplinkTxStream(blockSamples, ...
            {ccsdsUplinkCommand("report", 7, -3, 0.5, -70)}, config);
    else
        [blk, inf1] = ccsdsUplinkTxStream(blockSamples, {}, config);
    end
    stream((b-1)*blockSamples + (1:blockSamples)) = blk;
    elements(b) = inf1.element;
end

fprintf('  %d blocks x %d samples = %.2f s of airtime\n', ...
    nBlocks, blockSamples, numel(stream)/u.sampleRate);
fprintf('  CLTUs sent: %d (3 queued)\n', inf1.cltusSent);

% Continuity: a PLOP-2 stream must never go quiet. Check the envelope after
% the filter has filled, in symbol-length windows.
env = movmean(abs(stream).^2, 4*sps);
env = env(2*numel(rcosdesign(u.rolloffFactor, u.filterSpanSymbols, sps, 'sqrt')) : end);
quietFrac = mean(env < 0.02*mean(env));
fprintf('  fraction of stream that is effectively silent: %.4f  (must be ~0)\n', quietFrac);
fprintf('  min/mean/max envelope power: %.4f / %.4f / %.4f\n', ...
    min(env), mean(env), max(env));

%% 2. Block boundaries seamless
fprintf('\n=== 2. filter state across block boundaries ===\n');
% Regenerate the same stream in ONE call and compare. If the shaping filter
% were restarting per block, these would differ at every boundary.
clear ccsdsUplinkTxStream ccsdsUplinkPulseShape;
[oneShot, ~] = ccsdsUplinkTxStream(numel(stream), payloads, config);
% Re-queue the mid-stream command at the same point is not possible in one
% call, so compare only up to where that command was added.
cmp = min(numel(oneShot), 5*blockSamples);
maxDiff = max(abs(oneShot(1:cmp) - stream(1:cmp)));
fprintf('  first %d samples, blocked vs single call: max diff %.3e  (must be ~0)\n', ...
    cmp, maxDiff);

%% 3. Command round trip
fprintf('\n=== 3. command round trip through the real encoder/decoder ===\n');
cases = { ...
    {"report",  0,   0.00,  0.00, -96}, ...
    {"report", 255, 31.9375, 15.9375, 0}, ...
    {"request", 0}, ...
    {"request", 16777215}, ...
    {"request", 500, 500+8191} };

tcCfg = ccsdsUplinkTCConfig(config, "CLTU");
nPass = 0;
for i = 1:numel(cases)
    c = cases{i};
    payload = ccsdsUplinkCommand(c{:});
    syms = ccsdsUplinkSymbols("cltu", payload, config);

    frames = ccsdsTCIdealReceiver(complex(syms), tcCfg);
    ok = false;
    for k = 1:numel(frames)
        got = ccsdsUplinkParseCommand(dvbs2PackBits(double(frames{k}(:))));
        if got.Type == "report" && string(c{1}) == "report"
            ok = got.Count == c{2} && abs(got.MeanSNRdB - c{3}) < 0.07 && ...
                 abs(got.SigmaSNRdB - c{4}) < 0.07 && abs(got.RSSIdB - c{5}) < 0.13;
        elseif got.Type == "request" && string(c{1}) == "request"
            wantEnd = c{end};
            if numel(c) == 2, wantEnd = c{2}; end
            ok = got.StartIdx == c{2} && got.EndIdx == wantEnd;
        end
        if ok, break; end
    end
    nPass = nPass + ok;
    fprintf('  %-8s %-30s %s\n', c{1}, mat2str([c{2:end}]), string(ok));
end
fprintf('  ---- %d/%d passed ----\n', nPass, numel(cases));

%% 4. Level and spectrum
fprintf('\n=== 4. level and spectrum ===\n');
rms = sqrt(mean(abs(stream).^2));
pk = max(abs(stream));
fprintf('  RMS %.3f (target %.3f) | peak %.3f | PAPR %.2f dB\n', ...
    rms, u.txRMS, pk, 20*log10(pk/rms));
fprintf('  headroom to full scale: %.2f dB %s\n', 20*log10(1/pk), ...
    string(pk < 1));

N = 16384;
seg = stream(end-N+1:end);
W = abs(fft(seg, N)).^2;  W = W/sum(W);
f = (0:N-1)/N*u.sampleRate;
f(f >= u.sampleRate/2) = f(f >= u.sampleRate/2) - u.sampleRate;
inBand = sum(W(abs(f) <= u.symbolRate*(1+u.rolloffFactor)/2));
dcFrac = sum(W(abs(f) < 200));
fprintf('  energy inside the RRC band   : %.4f\n', inBand);
fprintf('  energy within +-200 Hz of DC : %.4f  (suppressed carrier: expect ~0)\n', dcFrac);
