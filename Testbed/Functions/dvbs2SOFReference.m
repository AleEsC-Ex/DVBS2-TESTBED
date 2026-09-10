
function sofRef = dvbs2SOFReference()
%DVBS2SOFREFERENCE Reference SOF (Start Of Frame) symbols for DVB-S2 frame sync.
%
%   sofRef = dvbs2SOFReference() returns the 26 complex symbols
%   corresponding to the fixed SOF bit pattern 0x18D2E82 (ETSI EN 302
%   307-1), modulated per the PLHEADER's pi/2-BPSK mapping: successive
%   bits alternate onto the real and imaginary axes. This pattern is
%   identical for EVERY PL frame regardless of MODCOD, which is what
%   makes it usable for modulation-independent frame synchronization.
%
%   IMPORTANT -- VERIFY AGAINST YOUR OWN TRANSMITTER:
%   Bit-to-symbol polarity (which bit value maps to +1 vs -1) and the
%   even/odd axis assignment are exactly the kind of detail that's easy
%   to get backwards from a spec reading alone. Before relying on this
%   in your frame synchronizer, generate one clean PLHEADER with
%   dvbs2WaveformGenerator at high EsNo / no impairments, and correlate
%   it against this reference (see dvbs2FrameSync). If the correlation
%   peak is weak or missing, try:
%     - sofRef = conj(sofRef)          (conjugate)
%     - sofRef = -sofRef               (180-degree flip)
%     - swapping which parity (even/odd) maps to real vs imaginary
%   until you get a clean, strong peak against your own known-good
%   PLHEADER. This is a one-time calibration step, not something you
%   need to re-derive per run.

sofHex = uint32(hex2dec('18D2E82'));
sofBits = double(bitget(sofHex, 26:-1:1));   % MSB first, 26 bits

sofRef = zeros(26,1);
for k = 0:25
    b = sofBits(k+1);
    sym = 1 - 2*b;                  % bit 0 -> +1, bit 1 -> -1
    if mod(k,2) == 0
        sofRef(k+1) = sym;          % even index -> real axis
    else
        sofRef(k+1) = 1j*sym;       % odd index -> imaginary axis
    end
end

end