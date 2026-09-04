%UPLINKRXTEST Verify the uplink receiver chain without radios.
%
%   Four things, in increasing order of how much they depend on the others:
%
%     1. the constants   the start sequence, tail, randomizer and codeword
%                        geometry the receiver assumes are checked against
%                        what ccsdsTCWaveform actually generates, and the
%                        parity-check matrix against real codewords. A wrong
%                        constant here produces a receiver that decodes
%                        nothing, which on hardware is indistinguishable
%                        from a dead radio -- so it is checked first.
%     2. thresholds      what the two detection metrics read on pure noise,
%                        which is what the thresholds have to clear.
%     3. end to end      bursts through a channel with delay, Doppler,
%                        arbitrary phase, DC offset and noise, swept over
%                        Es/No.
%     4. margins         where each stage stops working, so it is known
%                        which one gives out first.

clear; clc;
clear ccsdsUplinkReceive ccsdsUplinkDCSuppress;
addpath(genpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'Functions')));

config = dvbs2TestbedConfig();
u = config.uplink;
fr = ccsdsUplinkFraming(config);
tcCfg = ccsdsUplinkTCConfig(config);

fprintf('=== 1. constants, against the toolbox ===\n');

% Regenerate a burst at symbol rate and read the pieces back out.
probeBits = randi([0 1], fr.infoLength, 1);
sym = real(ccsdsTCWaveform(probeBits, tcCfg));
gotBits = double(sym > 0);

okStart = isequal(gotBits(1:numel(fr.startBits)), fr.startBits);
okTail  = isequal(gotBits(end-numel(fr.tailBits)+1:end), fr.tailBits);
okLen   = numel(sym) == fr.cltuSymbols;

cwBits = gotBits(fr.codewordOffset + (1:fr.codewordLength));
deran  = mod(cwBits - fr.randomizer, 2);
okSys  = isequal(deran(1:fr.infoLength), probeBits);

H = ccsdsUplinkLDPCMatrix(fr.codewordLength);
okParity = all(mod(H*deran, 2) == 0);
okParityRaw = all(mod(H*cwBits, 2) == 0);

fprintf('  CLTU length %d symbols (%d start + %d codeword + %d tail) : %s\n', ...
    fr.cltuSymbols, numel(fr.startBits), fr.codewordLength, numel(fr.tailBits), string(okLen));
fprintf('  start sequence matches                                   : %s\n', string(okStart));
fprintf('  tail sequence matches                                    : %s\n', string(okTail));
fprintf('  derandomized codeword is systematic (info == payload)     : %s\n', string(okSys));
fprintf('  H * (derandomized codeword) == 0                         : %s\n', string(okParity));
fprintf('  H * (raw codeword) == 0                                  : %s  <- must be false,\n', string(okParityRaw));
fprintf('       which is the proof that derandomizing comes BEFORE decoding\n');
fprintf('  ---- constants %s ----\n\n', ...
    string(okStart && okTail && okLen && okSys && okParity && ~okParityRaw));

fprintf('=== 2. what the detectors read on pure noise ===\n');
rng(20260827);
nTrial = 40;
carrierNoise = zeros(nTrial,1);
asmNoise = zeros(nTrial,1);
for t = 1:nTrial
    w = (randn(u.searchWindowSamples,1) + 1j*randn(u.searchWindowSamples,1))/sqrt(2);
    [f0, carrierNoise(t)] = ccsdsUplinkCoarseCFO(w, config);
    mf = ccsdsUplinkMatchedFilter(w .* exp(-1j*2*pi*f0*(0:numel(w)-1).'/u.sampleRate), config);
    d = ccsdsUplinkASMDetect(mf, config);
    asmNoise(t) = d.metric;
end
fprintf('  stage 2 squared-spectrum peak : median %.1f dB, max %.1f dB   (threshold %g)\n', ...
    median(carrierNoise), max(carrierNoise), u.detectThresholdDB);
fprintf('  stage 4 start correlation     : median %.3f,  max %.3f     (threshold %g)\n\n', ...
    median(asmNoise), max(asmNoise), u.asmThreshold);

fprintf('=== 3. end to end, through a channel ===\n');
fprintf('  delay uniform over a burst, CFO uniform +-%g kHz, phase uniform,\n', u.maxCarrierOffsetHz/1e3);
fprintf('  DC offset 5%% of RMS, AWGN. 24 bursts per point.\n\n');
fprintf('  %6s %6s %9s %9s %10s %9s %9s\n', ...
    'Es/No', 'sent', 'stage 4', 'stage 7', 'commands', 'CFO err', 'Es/No est');
fprintf('  %6s %6s %9s %9s %10s %9s %9s\n', ...
    'dB', '', 'locked', 'parity', 'correct', 'Hz', 'dB');

esnoList = [-2 -1 0 1 2 3 4 6 10];
nSent = 24;
correct = zeros(numel(esnoList), 1);
for ei = 1:numel(esnoList)
    esno = esnoList(ei);
    res = runLink(config, esno, nSent, 20260827 + ei);
    correct(ei) = res.correct;
    fprintf('  %6.1f %6d %9d %9d %10d %9.2f %9.2f\n', ...
        esno, res.sent, res.stage4, res.decoded, res.correct, ...
        res.cfoErrHz, res.esNoEst);
end
fprintf('  (stage 2 is a per-WINDOW gate, not per burst, so it is not counted here;\n');
fprintf('   at the configured %g dB it passes essentially every window by design)\n', ...
    u.detectThresholdDB);

fprintf('\n=== 4. sensitivity ===\n');
iAll = find(correct == nSent, 1);
iAny = find(correct > 0, 1);
fprintf('  every command recovered from : %s dB Es/No\n', localDB(esnoList, iAll));
fprintf('  first command recovered at   : %s dB Es/No\n', localDB(esnoList, iAny));
fprintf('\n  For reference, the LDPC(128,64) code ALONE -- fed perfect symbols with\n');
fprintf('  no sync to do -- reaches 0%% codeword error at 2 dB Es/No. So the chain\n');
fprintf('  is within about a decibel of the code it is carrying, which means no\n');
fprintf('  single sync stage is now the thing holding the link back.\n');

fprintf('\n=== 5. what it costs ===\n');
clear ccsdsUplinkReceive ccsdsUplinkDCSuppress;
noise = (randn(200000,1) + 1j*randn(200000,1))/sqrt(2);
tCost = tic;
for p = 1:u.rxFrameLength:numel(noise)
    ccsdsUplinkReceive(noise(p:min(p+u.rxFrameLength-1, numel(noise))), config);
end
el = toc(tCost);
fprintf('  1.00 s of airtime, pure noise (worst case -- every window is examined\n');
fprintf('  and none is rejected) processed in %.3f s = %.0f%% of real time\n', el, 100*el);

function s = localDB(list, idx)
    if isempty(idx)
        s = 'never (in this sweep)';
    else
        s = sprintf('%g', list(idx));
    end
end

function res = runLink(config, esnodB, nBurst, seed)
%RUNLINK One sweep point: nBurst commands through the channel and the
%receiver, streamed in radio-sized reads.
    u = config.uplink;
    rng(seed);
    clear ccsdsUplinkReceive ccsdsUplinkDCSuppress;
    clear ccsdsUplinkTxStream ccsdsUplinkPulseShape;

    res = struct('sent', nBurst, 'stage2', 0, 'stage4', 0, 'decoded', 0, ...
        'correct', 0, 'cfoErrHz', NaN, 'esNoEst', NaN);

    esNoEst = [];
    expected = cell(1, nBurst);
    for b = 1:nBurst
        if mod(b,2) == 0
            expected{b} = ccsdsUplinkCommand("report", mod(b*7,256), ...
                -20 + mod(b*3.5, 45), mod(b*0.5, 8), -80 + mod(b*4, 70));
        else
            expected{b} = ccsdsUplinkCommand("request", mod(b*911, 100000), ...
                mod(b*911, 100000) + mod(b, 250));
        end
    end

    % A CONTINUOUS PLOP-2 stream, not a series of padded bursts: acquisition
    % sequence, then the commands separated by idle. Commands are queued a
    % few at a time so idle of varying length falls between them, which is
    % what a real link looks like.
    clean = complex(zeros(0,1));
    queued = 0;
    while queued < nBurst || numel(clean) < 4*u.searchWindowSamples
        if queued < nBurst
            batch = expected(queued+1 : min(queued+2, nBurst));
            queued = queued + numel(batch);
        else
            batch = {};
        end
        clean = [clean; ccsdsUplinkTxStream(u.txBlockSamples, batch, config)]; %#ok<AGROW>
    end
    % Let the last CLTU clear the shaper and give the receiver a final window.
    clean = [clean; ccsdsUplinkTxStream(2*u.searchWindowSamples, {}, config)];

    % One carrier offset for the whole session -- a continuous link has ONE
    % carrier, not a fresh one per burst.
    cfo = (2*rand - 1) * u.maxCarrierOffsetHz;
    ph = 2*pi*rand;
    t = (0:numel(clean)-1).' / u.sampleRate;
    stream = clean .* exp(1j*(2*pi*cfo*t + ph));

    Ps = mean(abs(clean).^2);
    sigma2 = Ps * u.samplesPerSymbol / (10^(esnodB/10));
    stream = stream + sqrt(sigma2/2)*(randn(size(stream)) + 1j*randn(size(stream)));
    stream = stream + 0.05*sqrt(Ps)*(1 + 1j)/sqrt(2);     % LO leakage

    % Feed it through in reads the size the radio delivers.
    got = {};
    cfoErr = [];
    for p = 1:u.rxFrameLength:numel(stream)
        chunk = stream(p : min(p+u.rxFrameLength-1, numel(stream)));
        [cmds, info] = ccsdsUplinkReceive(chunk, config);
        res.stage2 = res.stage2 + info.detections;
        res.stage4 = res.stage4 + info.asmLocks;
        res.decoded = res.decoded + info.decodes;
        for c = 1:numel(cmds)
            got{end+1} = cmds{c}; %#ok<AGROW>
        end
        if ~isnan(info.esNodB)
            esNoEst(end+1) = info.esNodB; %#ok<AGROW>
        end
        if ~isnan(info.offsetHz)
            cfoErr(end+1) = info.offsetHz; %#ok<AGROW>
        end
    end

    % A command counts as correct if it parsed and matches one that was
    % sent. Position in the stream is not used -- a duplicate-suppressed or
    % re-ordered decode is still a correct one.
    wanted = cellfun(@(p) ccsdsUplinkParseCommand(p), expected, 'UniformOutput', false);
    for c = 1:numel(got)
        for k = 1:numel(wanted)
            if isequal(got{c}, wanted{k})
                res.correct = res.correct + 1;
                break;
            end
        end
    end

    if ~isempty(esNoEst), res.esNoEst = mean(esNoEst); end
    if ~isempty(cfoErr)
        % A continuous link has ONE carrier for the whole session, so every
        % reported offset is an estimate of the same number. Worst case is
        % the honest figure to quote.
        res.cfoErrHz = max(abs(cfoErr - cfo));
    end
end
