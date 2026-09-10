function [accept, reason] = dvbs2FrameAcceptable(phyParams, config)
%DVBS2FRAMEACCEPTABLE Reject PLHEADERs the transmitter could not have sent.
%
%   [accept, reason] = dvbs2FrameAcceptable(phyParams, config)
%
%   phyParams - decoded PLHEADER struct from dvbs2PLHeaderRecover.m
%   config    - testbed config; reads config.rxReject.* (see below)
%
%   accept    - true if this header is consistent with what S1a is
%               configured to transmit
%   reason    - '' when accepted; otherwise a short human-readable phrase
%               naming the single assumption that was violated
%
%   WHAT THIS IS FOR. The PLSC is 64 symbols protected by a Reed-Muller
%   code and decoded by nearest-neighbour search. Under noise it can land
%   on a valid-but-wrong codeword: the decode "succeeds", reports high
%   confidence, and hands the receive chain a MODCOD, frame length and
%   pilot layout that describe a frame nobody transmitted. Everything
%   downstream then measures the wrong thing while looking healthy --
%   observed on hardware as PLS=47, PLS=71, and a ModulationOrder=16
%   decode on a QPSK-only link.
%
%   The defence is that S2b already knows what S1a is allowed to send. A
%   header describing anything else is not a marginal frame to be decoded
%   carefully; it is proof that this particular decode is wrong.
%
%   EVERY CHECK HERE IS A STATEMENT ABOUT THIS TESTBED'S CURRENT
%   CONFIGURATION, NOT ABOUT DVB-S2. Each one is separately switchable so
%   it can be retired on its own as the transmitter grows into more of the
%   standard, and each carries the condition that has to be met first.
%   None of them describe anything the standard forbids.

    accept = true;
    reason = '';

    if ~isfield(config, 'rxReject')
        return;   % nothing configured -- accept everything, as before
    end
    r = config.rxReject;

    % --- 1. PILOTS REQUIRED ------------------------------------------
    % The whole receive chain downstream of here is pilot-based:
    % dvbs2FineFreqEst, dvbs2PhaseCompensate and dvbs2SNREstimate all
    % take fp and all three degrade to a much cruder fallback without
    % pilots. S1a transmits pilots on every frame, so HasPilots = false
    % can only mean the TYPE field's pilot bit was flipped in the decode.
    %
    % TO REMOVE: the non-pilot paths have to be trustworthy first. The
    % SNR fallback in particular currently measures against a hardcoded
    % QPSK constellation (dvbs2SNREstimate.m, the fp.pilotInd-empty
    % branch), which is wrong for every other modulation.
    if isfield(r, 'requirePilots') && r.requirePilots && ~phyParams.HasPilots
        accept = false;
        reason = 'PLSC reports no pilots';
        return;
    end

    % --- 2. NORMAL FECFRAME REQUIRED ---------------------------------
    % S1a only ever builds 64800-bit normal FECFRAMEs. A short-frame
    % decode (16200) is a flipped TYPE bit, and it is a particularly
    % damaging one: it makes dvbs2FrameLength return roughly a quarter of
    % the true length, so the receiver consumes a fraction of the frame
    % and resumes its SOF search in the middle of the payload.
    %
    % TO REMOVE: S1a has to actually transmit short frames, and the
    % LDPC/BCH tables for the short-frame code rates have to be wired up.
    if isfield(r, 'requireNormalFrame') && r.requireNormalFrame && ...
            double(phyParams.FECFrameLength) ~= 64800
        accept = false;
        reason = sprintf('PLSC reports a %d-bit (short) FECFRAME', ...
            double(phyParams.FECFrameLength));
        return;
    end

    % --- 3. MODCOD MUST BE ONE S1a CAN SELECT -------------------------
    % The PLS code packs as MODCOD*4 + TYPE, so the MODCOD index is the
    % top 5 bits. dvbs2PLHeaderRecover already uses the same convention
    % for its dummy test (PLSDecimalCode < 4 is MODCOD 0), and it matches
    % the hardware logs exactly: PLS 5/17/29/41/45 against S1a's reported
    % MODCOD 1/4/7/10/11.
    %
    % S1a's ACM policy can only ever choose from config.acm.modcodSet, so
    % anything outside it is unreachable by construction -- not unlikely,
    % impossible. This is the check that catches a QPSK link decoding to
    % 16APSK, which no amount of confidence weighting would have caught,
    % because the wrong codeword was decoded confidently.
    %
    % TO REMOVE: widen config.acm.modcodSet. This check then widens with
    % it automatically and needs no edit -- it only ever asserts that the
    % receiver and transmitter agree on the same set.
    if isfield(r, 'restrictToModcodSet') && r.restrictToModcodSet && ...
            isfield(config, 'acm') && isfield(config.acm, 'modcodSet')
        modcod = floor(double(phyParams.PLSDecimalCode) / 4);
        if ~ismember(modcod, config.acm.modcodSet)
            accept = false;
            reason = sprintf('MODCOD %d is outside acm.modcodSet (%d-%d)', ...
                modcod, min(config.acm.modcodSet), max(config.acm.modcodSet));
            return;
        end
    end
end
