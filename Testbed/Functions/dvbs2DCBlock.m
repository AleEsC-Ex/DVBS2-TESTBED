function samplesOut = dvbs2DCBlock(samplesIn)
%DVBS2DCBLOCK Removes LO-leakage/DC-offset from raw receive samples.
%
%   samplesOut = dvbs2DCBlock(samplesIn) applies a first-order DC-blocking
%   IIR filter to a column vector of raw complex baseband samples,
%   maintaining filter state across calls so chunk boundaries don't
%   introduce discontinuities. LO leakage in a direct-conversion RF
%   front end produces a large, spurious spike at exactly 0 Hz that
%   otherwise dominates RSSI and every downstream stage (AGC, raw CFO
%   compensation), masking the actual received signal underneath it.
%
% Inputs:
%   samplesIn  - Column vector of raw complex baseband samples, straight
%                out of radioRx() (before RSSI measurement, AGC, or any
%                other processing).
%
% Output:
%   samplesOut - Same size as samplesIn, with the DC/LO-leakage component
%                removed.

persistent zi

alpha = 0.99;
b = [1, -1];
a = [1, -alpha];

if isempty(zi)
    zi = 0;
end

[samplesOut, zi] = filter(b, a, samplesIn(:), zi);

end
