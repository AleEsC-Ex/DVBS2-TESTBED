function refHeader = dvbs2PLHeaderReference(phyParams)
%DVBS2PLHEADERREFERENCE Ideal 90-symbol PLHEADER for a decoded PLS code.
%
%   refHeader = dvbs2PLHeaderReference(phyParams)
%
%   phyParams - decoded PLHEADER struct from dvbs2PLHeaderRecover.m
%   refHeader - 90x1 complex column: the PLHEADER that a transmitter
%               WOULD have sent for these parameters -- 26 SOF symbols
%               followed by the 64-symbol PLSC, already Reed-Muller
%               coded, scrambled and pi/2-BPSK modulated.
%
%   WHY THIS EXISTS. dvbs2SNREstimate needs a known-symbol reference to
%   measure a residual against. The pilots give it one, but pilot
%   positions are derived from the PLSC and the pilots are spread across
%   the whole 99.8 ms frame -- so a fault in either the pilot layout or
%   the frequency correction corrupts that reference silently. The
%   PLHEADER is an independent second reference: 90 symbols in a 270 us
%   window, present in EVERY PLFRAME, and always pi/2-BPSK whatever the
%   frame's own modulation.
%
%   WHY IT IS ONLY A REFEREE, NOT THE PRIMARY ESTIMATE. 90 symbols give a
%   relative noise-power error of 1/sqrt(90) = 10.5%, i.e. +-0.44 dB per
%   frame, against +-0.15 dB from the 792 pilot symbols. The header is
%   the more RELIABLE reference and the pilots are the more PRECISE one,
%   which is exactly why they are worth comparing rather than choosing
%   between.
%
%   THE PLS CODE PACKS AS  MODCOD*4 + short*2 + pilots  (ETSI EN 302
%   307-1 5.5.2.2), so the MODCOD index recovered here is floor(PLS/4).
%   Verified against the hardware logs: PLS 5/17/29/41/45 correspond to
%   S1a's reported MODCOD 1/4/7/10/11.

    % A run uses only a handful of distinct PLS codes -- typically one per
    % ACM rung -- so building each reference once and reusing it keeps
    % this off the per-frame path entirely.
    persistent cache
    if isempty(cache)
        cache = containers.Map('KeyType', 'double', 'ValueType', 'any');
    end

    plsCode = double(phyParams.PLSDecimalCode);

    if isKey(cache, plsCode)
        refHeader = cache(plsCode);
        return;
    end

    modcod = floor(plsCode / 4);

    % satcom.internal.dvbs.plHeader is the exact encoder counterpart of
    % the satcom.internal.dvbs.plHeaderRecover decoder that
    % dvbs2PLHeaderRecover.m already calls, so the reference is generated
    % by the same code path that produced the decode -- no second,
    % hand-rolled implementation of the Reed-Muller coding and scrambling
    % to drift out of step with it.
    refHeader = satcom.internal.dvbs.plHeader('S2', modcod, ...
        logical(phyParams.HasPilots), double(phyParams.FECFrameLength));

    refHeader = refHeader(:);

    if numel(refHeader) ~= 90
        error('dvbs2PLHeaderReference:BadLength', ...
            'Expected a 90-symbol PLHEADER, got %d (time slicing is not used here).', ...
            numel(refHeader));
    end

    cache(plsCode) = refHeader;
end
