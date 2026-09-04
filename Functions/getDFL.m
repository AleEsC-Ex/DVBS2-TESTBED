function dfl = getDFL(modcod, fecFrame)
% getDFL Compute the DVB-S2 Data Field Length (DFL) for a given MODCOD.
%
%   dfl = getDFL(modcod, fecFrame) looks up the standard DVB-S2 Kbch
%   (BCH-coded block length) for the requested MODCOD/FECFrame
%   combination and returns the Data Field Length, i.e. the number of
%   payload bits available after the 80-bit mandatory BBHEADER:
%
%       DFL = Kbch - 80
%
%   This is the value normally assigned to cfgDVBS2.DFL before calling
%   dvbs2WaveformGenerator.
%
% Syntax:
%   dfl = getDFL(modcod, fecFrame)
%
% Inputs:
%   modcod   - Integer from 1 to 28, the standard DVB-S2 MODCOD index
%              (1 = QPSK 1/4 ... 28 = 32APSK 9/10, per ETSI EN 302 307-1).
%   fecFrame - Char array or string, either 'normal' or 'short'
%              (case-insensitive), selecting the LDPC FECFRAME size
%              (64,800 bits for 'normal', 16,200 bits for 'short').
%
% Output:
%   dfl      - Data Field Length in bits (Kbch - 80).
%
% Notes:
%   Not every MODCOD is defined for short FECFRAMEs: MODCOD 11, 17, 23
%   and 28 have no short-frame variant in the DVB-S2 standard. Calling
%   getDFL with 'short' for one of these MODCODs raises an error rather
%   than returning a bogus negative DFL.

% Validate and normalize the FECFrame argument.
fecFrame = lower(char(fecFrame));
if ~strcmp(fecFrame, 'normal') && ~strcmp(fecFrame, 'short')
    error('getDFL:InvalidFECFrame', ...
        'fecFrame must be ''normal'' or ''short''.');
end

% Validate the MODCOD range defined by the DVB-S2 standard.
if modcod < 1 || modcod > 28
    error('getDFL:InvalidMODCOD', ...
        'modcod must be an integer between 1 and 28.');
end

% --- DVB-S2 Kbch LOOKUP TABLE ---
% Each table position corresponds to the indexed MODCOD [1..28].
% A value of 0 marks a MODCOD/FECFrame combination that does not exist
% in the standard (short FECFRAME variants of MODCOD 11/17/23/28).

if strcmp(fecFrame, 'normal')
    % Kbch values for Normal Frames (64,800 total LDPC bits).
    % Indexed from MODCOD 1 (QPSK 1/4) through MODCOD 28 (32APSK 9/10).
    kbch_table = [...
        16008 21408 25728 32208 38688 43040 48408 51648 53840 57472 ...
        58192 38688 43040 48408 53840 57472 58192 43040 48408 51648 ...
        53840 57472 58192 48408 51648 53840 57472 58192 ];
else
    % Kbch values for Short Frames (16,200 total LDPC bits).
    % Indexed from MODCOD 1 (QPSK 1/4) through MODCOD 28 (32APSK 9/10).
    kbch_table = [...
        3072 5232 6312 7032 9552 10632 11712 12432 13152 14232 0 ...
        9552 10632 11712 13152 14232 0 10632 11712 12432 13152 14232 0 11712 ...
        12432 13152 14232 0 ];
end

% Look up the Kbch value for the requested MODCOD.
Kbch = kbch_table(modcod);

if Kbch == 0
    error('getDFL:ReservedCombination', ...
        'MODCOD %d is not defined for FECFrame ''%s'' (reserved combination in DVB-S2).', ...
        modcod, fecFrame);
end

% The maximum allowed DFL is Kbch minus the 80 mandatory BBHEADER bits.
dfl = Kbch - 80;
end
