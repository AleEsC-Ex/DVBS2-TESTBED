function [fEst, metricDB, info] = ccsdsUplinkCoarseCFO(x, config)
%CCSDSUPLINKCOARSECFO Stage 2a -- acquire the carrier offset by squaring.
%
%   [fEst, metricDB, info] = ccsdsUplinkCoarseCFO(x, config) estimates the
%   carrier frequency offset of a window that may contain a BPSK burst, and
%   returns how strongly that window looks like a burst at all.
%
%   metricDB is the squared-spectrum peak relative to the median of the same
%   band, in dB. It is BOTH the detection statistic and the confidence in
%   fEst -- there is no separate energy detector.
%
%   info fields: .peakHz (the tone found, at twice the offset), .binHz,
%   .fftLength, .medianDB.
%
%   WHY SQUARING. A BPSK signal is s(t)*exp(j*2*pi*df*t+jphi) with s = +-1,
%   so its spectrum is the data's spectrum shifted -- there is no carrier
%   line to find. Squaring gives s^2*exp(j*4*pi*df*t+j2phi), and s^2 is
%   always positive, so the modulation collapses into a DC term and a tone
%   appears at TWICE the offset. The old receiver did not need this: under
%   PCM/PSK/PM about half the transmitted power sat in an unmodulated
%   residual carrier and one FFT peak found it directly. Suppressing that
%   carrier is what bought this link its 3 dB and its bandwidth back, and
%   this function is the price.
%
%   THE PRICE, SPECIFICALLY, AND IT IS THE WHOLE DIFFICULTY OF THIS STAGE.
%   Squaring multiplies the noise by itself as well as by the signal, so the
%   tone's strength goes as the SQUARE of per-sample SNR while the noise
%   floor barely moves. Per-sample SNR here is Es/No minus 10*log10(25), so
%   at 2 dB Es/No it is -12 dB and the squaring loss is severe. Measured,
%   with neither of the two measures below: this stage failed outright below
%   4 dB Es/No -- returning frequency errors of KILOHERTZ, not hertz -- while
%   every stage after it worked at 2 dB. Acquisition, not coding, set the
%   sensitivity of the whole link.
%
%   TWO THINGS FIX THAT, both of them about not squaring more noise than
%   necessary:
%
%   1. BAND-LIMIT FIRST. The signal can only be within +-(maxCarrierOffset +
%      half the occupied bandwidth) of DC, about +-30 kHz of a 200 kHz
%      stream. The other 70% of the band is pure noise being squared for no
%      reason. A lowpass there costs 64 taps and is worth about 5 dB.
%
%      Note it is a FILTER, not a decimation. Decimating would be cheaper
%      still, but the tone lands at TWICE the offset, so the squared signal
%      needs +-50 kHz of room -- and at 100 kHz of Nyquist there is none to
%      spare. Filtering gets the full benefit anyway, because it is
%      per-sample SNR, not sample rate, that drives the squaring loss.
%
%   2. DO NOT DILUTE THE BURST. The detection statistic goes as A^2/N, with
%      A the samples that actually contain signal and N the window length,
%      so a window twice as long as it needs to be costs 3 dB. The search
%      window is sized in dvbs2TestbedConfig.m to just over one CLTU for
%      exactly this reason; it used to be four times that.
%
%   What is left is paid back by the FFT's own processing gain, 10*log10(N),
%   about 42 dB at these window sizes.
%
%   WHY NOT MATCHED-FILTER FIRST, which would raise per-sample SNR by 12.7
%   dB and make the squaring far cheaper? Because the matched filter is only
%   matched once the offset is removed. At the +-25 kHz this link must
%   tolerate, the signal sits entirely OUTSIDE the RRC filter's +-5.4 kHz
%   passband and the filter would delete it. Acquisition has to happen
%   first, on the raw stream; that is exactly why the carrier recovery is
%   split in two, with the Costas loop downstream of the RRC in stage 5.
%
%   WHY THE MEDIAN. It is the peak's own level relative to the rest of the
%   band, and unlike the mean the median is not dragged upward by the very
%   peak being measured, nor by a strong interferer elsewhere in the band.
%
%   ACCURACY, AND WHY IT IS NOT ENOUGH ON ITS OWN. A bare bin is Fs/N, about
%   6 Hz here, which sounds fine until you notice that 6 Hz over a 40 ms
%   CLTU is a quarter of a turn of phase. Parabolic interpolation across the
%   peak takes it to well under a hertz, and halving to recover df halves
%   the error again. What is left is the Costas loop's job.

    % Cached across calls: this stage runs on EVERY window, so the pieces
    % that depend only on the window length -- the Hann taper, the frequency
    % axis, the in-band mask -- are built once. They were about a third of
    % its cost when rebuilt each time.
    persistent lpf lpfKey win winLen fAxis bandMask axisKey

    Fs = config.uplink.sampleRate;
    maxOff = config.uplink.maxCarrierOffsetHz;
    occBW = config.uplink.symbolRate * (1 + config.uplink.rolloffFactor);

    x = complex(x(:));
    fEst = 0;
    metricDB = -Inf;
    info = struct('peakHz', NaN, 'binHz', NaN, 'fftLength', 0, 'medianDB', NaN);

    if numel(x) < 64
        return;
    end

    % Band-limit to where the signal could possibly be, before squaring.
    cutoff = (maxOff + occBW/2) / (Fs/2);
    if cutoff < 0.95
        thisKey = [cutoff, 64];
        if isempty(lpf) || ~isequal(thisKey, lpfKey)
            lpf = fir1(64, cutoff).';
            lpfKey = thisKey;
        end
        % filter() rather than conv(): its group delay is irrelevant to a
        % frequency estimate, and the phase this stage reports is not used
        % -- stage 4 measures phase for itself.
        xf = filter(lpf, 1, x);
    else
        xf = x;
    end

    % Square, then taper. The taper is applied AFTER squaring because it is
    % the squared signal's tone we are resolving; a Hann window trades a
    % little resolution for far lower sidelobes, which is what keeps a
    % strong residual DC term from leaking across the band and swamping a
    % nearby true peak.
    if isempty(win) || winLen ~= numel(xf)
        win = hann(numel(xf));
        winLen = numel(xf);
    end
    x2 = (xf.^2) .* win;

    N = 2^nextpow2(numel(x2));
    X = abs(fft(x2, N));

    thisAxis = [N, Fs, maxOff];
    if isempty(fAxis) || ~isequal(thisAxis, axisKey)
        f = (0:N-1).'/N * Fs;
        f(f >= Fs/2) = f(f >= Fs/2) - Fs;
        fAxis = f;
        % The tone sits at 2*df, so the search band is twice the offset range.
        bandMask = abs(f) <= 2*maxOff;
        axisKey = thisAxis;
    end
    f = fAxis;
    inBand = bandMask;

    if ~any(inBand)
        return;
    end
    Xin = X(inBand);
    [pk, kIn] = max(Xin);
    idxIn = find(inBand);
    k = idxIn(kIn);
    med = median(Xin);

    metricDB = 20*log10((pk + eps)/(med + eps));

    % Parabolic interpolation across the peak, in the same units the peak
    % was found in. Guarded so a peak sitting on the first or last bin, or a
    % flat top, cannot produce a wild correction.
    peakHz = f(k);
    if k > 1 && k < N
        a = X(k-1); b = X(k); c = X(k+1);
        denom = a - 2*b + c;
        if denom ~= 0
            delta = 0.5*(a - c)/denom;
            if abs(delta) < 1
                peakHz = peakHz + delta*Fs/N;
            end
        end
    end

    fEst = peakHz/2;

    info.peakHz = peakHz;
    info.binHz = Fs/N;
    info.fftLength = N;
    info.medianDB = 20*log10(med + eps);
end
