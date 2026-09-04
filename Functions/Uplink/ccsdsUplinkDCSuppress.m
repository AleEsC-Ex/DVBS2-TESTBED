function [y, dcLevel] = ccsdsUplinkDCSuppress(x, config)
%CCSDSUPLINKDCSUPPRESS Stage 1 -- remove LO leakage from the uplink stream.
%
%   [y, dcLevel] = ccsdsUplinkDCSuppress(x, config) applies a first-order
%   DC-blocking highpass to a chunk of raw complex baseband samples, keeping
%   filter state across calls so chunk boundaries are seamless. dcLevel is
%   the current estimate of the removed offset, useful as a health reading.
%
%   Call `clear ccsdsUplinkDCSuppress` to reset between runs.
%
%   WHY THIS MATTERS MORE THAN IT USED TO. Under PCM/PSK/PM the data sat on
%   a subcarrier, well away from 0 Hz, so a direct-conversion front end's LO
%   leakage landed in a part of the spectrum nothing was using and could
%   simply be ignored. Coherent BPSK puts the data AT baseband, so the
%   leakage now lands in the middle of the signal. It also survives the
%   squaring in stage 2 as a strong tone at 0 Hz, competing directly with
%   the tone the frequency estimator is looking for.
%
%   THE CORNER FREQUENCY IS THE WHOLE DESIGN, AND IT IS EASY TO GET WRONG.
%   y[n] = x[n] - x[n-1] + a*y[n-1] has a zero at DC and a pole at a, giving
%   a -3 dB corner near (1-a)*Fs/(2*pi) and a time constant of 1/(1-a)
%   samples. Two things have to hold at once:
%
%     - the corner must sit far below the signal band. At 20 Hz against a
%       10.8 kHz occupied bandwidth the filter removes about 0.4% of the
%       spectrum, which is nothing.
%     - the time constant must be long compared with anything in the
%       waveform the filter could mistake for DC. This is where an earlier
%       version of this testbed came unstuck: a = 0.99 gives a 100-sample
%       memory, which was SHORTER than the 160-sample constant-phase holds
%       of the DBPSK return link it was applied to, so the filter tracked
%       and subtracted the signal itself and produced 100% frame errors.
%       Here 20 Hz at 200 ksps is a 1592-sample memory, 64 symbols, and the
%       randomizer guarantees the data has no DC content over that span for
%       it to chase. The two facts are related: it is the randomizer that
%       makes an aggressive DC blocker safe.
%
%   The filter is primed from the first chunk's mean rather than starting
%   from zero state, so a large standing offset does not produce a
%   1592-sample transient at the top of every run -- which, arriving before
%   the first burst, would otherwise be the loudest thing in the window the
%   detector looks at.

    persistent zi initialised

    fc = config.uplink.dcCornerHz;
    Fs = config.uplink.sampleRate;

    a = 1 - 2*pi*fc/Fs;
    if a <= 0 || a >= 1
        error('ccsdsUplinkDCSuppress:BadCorner', ...
            ['config.uplink.dcCornerHz (%g) is not sensible against a ' ...
             '%g Hz sample rate.'], fc, Fs);
    end

    x = complex(x(:));

    if isempty(initialised)
        % Steady state of y[n] = x[n] - x[n-1] + a*y[n-1] for a constant
        % input m is y = 0, reached with state zi = -m. Setting it directly
        % skips the transient.
        if isempty(x)
            zi = 0;
        else
            zi = -mean(x);
        end
        initialised = true;
    end

    [y, zi] = filter([1, -1], [1, -a], x, zi);

    % zi holds -(the offset the filter is currently cancelling), so it is
    % the DC estimate, reported for logging.
    dcLevel = -zi;
end
