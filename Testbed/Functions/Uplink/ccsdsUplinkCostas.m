function [y, info] = ccsdsUplinkCostas(x, config)
%CCSDSUPLINKCOSTAS Stage 5 -- track the residual carrier across the burst.
%
%   [y, info] = ccsdsUplinkCostas(x, config) runs a second-order Costas loop
%   over one burst's worth of symbols (one sample per symbol, already
%   coarsely corrected and phase-aligned by stages 2 and 4) and returns them
%   with the residual carrier removed.
%
%   info fields: .residualHz (frequency the loop converged on),
%   .finalPhaseRad, .phaseTrace, .freqTrace.
%
%   WHAT IS LEFT FOR IT TO DO, in numbers. Stage 2's estimate is good to
%   well under a hertz and stage 4 nails the phase at the start sequence, so
%   at first sight the loop has nothing to correct. It earns its place on
%   the three things those two cannot do:
%
%     - stage 4 measures the phase ONCE, at the front of the burst. Any
%       residual frequency error rotates the constellation steadily from
%       there, and the codeblock is 128 symbols further along. Half a hertz
%       over 40 ms is 7 degrees, harmless; 5 Hz is 72 degrees and takes the
%       tail of the codeblock past the decision boundary.
%     - oscillator phase noise is not a frequency offset and no open-loop
%       estimate removes it.
%     - it degrades gracefully. If stage 2's peak was interpolated badly,
%       the loop absorbs the difference instead of the burst being lost.
%
%   WHY IT IS HERE AND NOT WHERE A BLOCK DIAGRAM WOULD PUT IT. Two
%   constraints, pulling the same way:
%
%     AFTER THE MATCHED FILTER, because the phase detector's own SNR is what
%     sets how much the loop jitters, and the filter is worth 12.7 dB of it.
%
%     AFTER THE START-SEQUENCE CORRELATION, because a Costas loop pulls in
%     over roughly 1/Bn. At the 40 Hz that keeps its jitter acceptable, that
%     is 25 ms against a 40 ms burst -- it would still be acquiring while
%     the codeblock went past. Starting it from the phase stage 4 already
%     measured means it begins locked and only has to hold, which is a
%     completely different and much easier problem. This is the usual shape
%     of a burst modem: the preamble acquires, the loop tracks.
%
%   THE DETECTOR is e = sign(Re y) * Im y, the standard BPSK Costas error:
%   the sign term strips the modulation, leaving a quantity proportional to
%   the phase error for small errors. Normalising by |y| makes the loop gain
%   independent of received level, so the same bandwidth setting behaves the
%   same way whatever the link budget is on the day.
%
%   Note it inherits the pi ambiguity -- sign(Re y) cannot tell a symbol
%   from its negation, and the loop is equally happy 180 degrees out. That
%   is not a problem here only because stage 4 has already resolved it and
%   the loop starts from that resolution.

    Rs = config.uplink.symbolRate;
    Bn = config.uplink.costasLoopBandwidthHz;
    zeta = config.uplink.costasDampingFactor;

    x = complex(x(:));
    y = complex(zeros(size(x)));
    info = struct('residualHz', 0, 'finalPhaseRad', 0, ...
        'phaseTrace', zeros(0,1), 'freqTrace', zeros(0,1));

    if isempty(x)
        return;
    end

    % Proportional-plus-integral gains for a given loop noise bandwidth and
    % damping, with detector and NCO gains normalised to 1 (which the
    % amplitude normalisation below makes true). Standard second-order
    % digital PLL design; theta is the normalised loop bandwidth.
    T = 1/Rs;
    theta = Bn*T/(zeta + 0.25/zeta);
    d = 1 + 2*zeta*theta + theta^2;
    Kp = 4*zeta*theta/d;
    Ki = 4*theta^2/d;

    phase = 0;
    freq = 0;
    phaseTrace = zeros(numel(x), 1);
    freqTrace = zeros(numel(x), 1);

    for n = 1:numel(x)
        yn = x(n) * exp(-1j*phase);
        y(n) = yn;

        mag = abs(yn);
        if mag > eps
            e = sign(real(yn)) * imag(yn) / mag;
        else
            e = 0;
        end

        freq = freq + Ki*e;
        phase = phase + freq + Kp*e;

        phaseTrace(n) = phase;
        freqTrace(n) = freq;
    end

    info.residualHz = freq/(2*pi) * Rs;
    info.finalPhaseRad = phase;
    info.phaseTrace = phaseTrace;
    info.freqTrace = freqTrace;
end
