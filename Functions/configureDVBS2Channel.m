function rxIn = configureDVBS2Channel(txOut, cfgDVBS2, simParams)
%CONFIGUREDVBS2CHANNEL Apply CFO, SCO, phase noise and AWGN to a DVB-S2 waveform.
%
%   rxIn = configureDVBS2Channel(txOut, cfgDVBS2, simParams) takes an
%   already-generated transmitted DVB-S2 waveform TXOUT and returns the
%   impaired received waveform RXIN, applying impairments in the order
%   CFO -> SCO -> phase noise -> AWGN (matching ETSI-style link budgets).
%
%   The CFO phase ramp continues seamlessly across successive calls
%   (persistent elapsed-time state) instead of restarting at zero each
%   time -- a real carrier offset has no notion of this testbed's burst
%   boundaries between calls, which exist purely as a TX-side data
%   generation/ACM-check convenience, not an actual gap in the
%   transmitted signal. Call `clear configureDVBS2Channel` to reset this
%   state at the start of a fresh run.
%
%   cfgDVBS2 must contain: SamplesPerSymbol, RolloffFactor
%   simParams must contain: chanBW, hasCFO, cfo, hasSCO, sco, hasPN,
%                            phNoiseLevel, EsNodB

persistent tStart phaseAccum
if isempty(tStart)
    tStart = 0;
    phaseAccum = 0;
end

sps    = cfgDVBS2.SamplesPerSymbol;
Rsymb  = simParams.chanBW / (1 + cfgDVBS2.RolloffFactor);
Fs     = Rsymb * sps;

rxSig = txOut(:);
N = length(rxSig);

%% 0. Time-varying pass profile (optional)
% Es/No and CFO for THIS block, either the static configured values or a
% point on a satellite-pass profile. The pass clock is tStart, which
% counts airtime rather than wall-clock, so the profile is reproducible
% regardless of how fast the host happens to run.
%
% The shape is the real geometry rather than an arbitrary sinusoid. For an
% overhead pass the slant range is R(t) = sqrt(h^2 + (v*t)^2), so with u
% running -1 -> 0 -> +1 across the pass:
%
%     R(u)/h  = sqrt(1 + (k*u)^2)          k set by the wanted edge loss
%     Es/No   = peak - 20*log10(R/h)       free-space loss goes as R^2
%     Doppler = -peak * (k*u/sqrt(1+(k*u)^2)) / (k/sqrt(1+k^2))
%
% giving maximum Es/No and zero Doppler at closest approach, and minimum
% Es/No with maximum Doppler (opposite signs) at the two horizons.
blockEsNodB = simParams.EsNodB;
blockCfoHz  = simParams.cfo;

if isfield(simParams, 'hasTimeVarying') && simParams.hasTimeVarying
    % Position within the pass, using the block's midpoint. Time past the
    % end of one pass wraps into the next.
    tMid = tStart + (N/2)/Fs;
    u = 2*mod(tMid, simParams.passDurationSec)/simParams.passDurationSec - 1;

    edgeLossDB = simParams.EsNodBPeak - simParams.EsNodBEdge;
    k = sqrt(10^(edgeLossDB/10) - 1);        % R_edge/h = 10^(edgeLoss/20)

    rangeRatio = sqrt(1 + (k*u)^2);
    blockEsNodB = simParams.EsNodBPeak - 20*log10(rangeRatio);

    if simParams.cfoPeakHz ~= 0
        blockCfoHz = -simParams.cfoPeakHz * (k*u/rangeRatio) / (k/sqrt(1+k^2));
    end
end

%% 1. Carrier frequency offset (CFO)
% Phase is ACCUMULATED rather than recomputed from absolute time. With a
% time-varying CFO the two differ: exp(1j*2*pi*f_k*t) with a large
% absolute t would jump in phase every time f_k changed between blocks,
% whereas accumulating keeps the carrier continuous through the sweep.
if simParams.hasCFO
    dphi = 2*pi*blockCfoHz/Fs;
    phase = phaseAccum + dphi*(1:N).';
    rxSig = rxSig .* exp(1j*phase);
    phaseAccum = mod(phase(end), 2*pi);      % wrapped, so long runs stay exact
end
tStart = tStart + N/Fs;   % advances regardless of which impairments are on

%% 2. Sampling clock offset (SCO)
% A positive SCO means the receiver ADC clock runs fast relative to the
% transmitter, equivalent to resampling at a slightly different rate.
%
% NOTE: resample()/upfirdn() work on INTEGER upsample/downsample ratios,
% so they are the wrong tool for ppm-level offsets -- the rational
% approximation of e.g. 1.000005 needs a denominator around 2e5, and
% p*q blows past resample's 2^31 limit immediately. For a fractional
% clock drift this small, interpolate onto a time-warped grid instead.
if simParams.hasSCO
    scoFactor = 1 + simParams.sco*1e-6;      % ppm -> fractional rate error
    N = length(rxSig);
    tOrig = (0:N-1).';                       % original sample instants (in samples)
    tNew  = (0:N-1).' * scoFactor;           % receiver ADC samples at a drifted rate
    tNew  = tNew(tNew <= tOrig(end));        % drop samples that run past the input's end
    rxSig = interp1(tOrig, rxSig, tNew, 'spline');
end


%% 3. Phase noise
% comm.PhaseNoise needs a mask (Level in dBc/Hz vs FrequencyOffset in Hz).
if simParams.hasPN
    switch simParams.phNoiseLevel
        case "Low"
            level      = [-73 -83 -93 -112 -128]; % dBc/Hz (VERIFY against ETSI mask)
            freqOffset = [1e2 1e3 1e4 1e5 1e6 ]; % Hz
        case "Medium"
            level      = [-59 -77 -88 -94 -104];
            freqOffset = [1e2 1e3 1e4 1e5 1e6 ]; % Hz
        case "High"
            level      = [-25 -50 -73 -85 -103];
            freqOffset = [1e2 1e3 1e4 1e5 1e6 ]; % Hz
        otherwise
            error("phNoiseLevel must be ""Low"", ""Medium"" or ""High"".");
    end
    pnoise = comm.PhaseNoise(Level=level, FrequencyOffset=freqOffset, SampleRate=Fs);
    rxSig = pnoise(rxSig);
end

%% 4. AWGN scaled from Es/No
% Es/No is defined per symbol; the signal here is oversampled at sps
% samples/symbol, so the equivalent SNR per sample must be reduced by
% 10*log10(sps) before calling awgn().
snrdB = blockEsNodB - 10*log10(sps);
rxIn = awgn(rxSig, snrdB, 'measured');

end