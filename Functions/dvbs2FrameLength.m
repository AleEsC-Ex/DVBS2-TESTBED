function frameLength = dvbs2FrameLength(phyParams)
%DVBS2FRAMELENGTH Total PLFRAME length in symbols, including pilots.
%
%   frameLength = dvbs2FrameLength(phyParams) computes the number of
%   symbols in one complete DVB-S2 PLFRAME -- PL header + XFECFRAME
%   payload + any interleaved pilot blocks -- from the decoded PLHEADER
%   metadata. Use this to know how many symbols to pull out of the
%   receive buffer for a given frame (see dvbs2PilotStructure, which
%   consumes the same frameLength to build the pilot index map).
%
%   phyParams - Struct with the decoded PLHEADER metadata, as returned
%               by dvbs2PLHeaderRecover. Must contain:
%                 .ModulationOrder  - constellation size (e.g. 4 for QPSK)
%                 .FECFrameLength   - LDPC codeword length in BITS
%                                     (64800 for Normal, 16200 for Short)
%                 .HasPilots        - true/false, whether pilot blocks
%                                     are inserted in this frame
%
%   frameLength - Total PLFRAME length in symbols:
%                   90 (PL header) + XFECFRAME symbols + pilot symbols.
%
%   Per ETSI EN 302 307-1, a 36-symbol pilot block is inserted after
%   every 16 data slots (90 symbols/slot), except when that position
%   falls exactly at the end of the frame.

bitsPerSymbol = log2(phyParams.ModulationOrder);

% XFECFRAME length in symbols: the LDPC codeword mapped onto the
% constellation (bitsPerSymbol bits per symbol).
xfecLen = phyParams.FECFrameLength / bitsPerSymbol;

if phyParams.HasPilots == 1
    numSlots = xfecLen / 90;
    % One pilot block every 16 slots, except a trailing one exactly at
    % the frame end (per the standard, no pilot block is inserted
    % after the very last slot).
    numPilotBlocks = floor((numSlots - 1)/16);
else
    numPilotBlocks = 0;
end

% PL header (90 symbols) + XFECFRAME payload + 36 symbols per pilot block.
frameLength = 90 + xfecLen + 36*numPilotBlocks;
end
