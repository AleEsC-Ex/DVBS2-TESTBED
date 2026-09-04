function fp = dvbs2PilotStructure(phyParams, frameLength)
%DVBS2PILOTSTRUCTURE Generate pilot positions and reference symbols.
%
% The pilot indices are relative to a complete PLFRAME:
%   index 1 = first SOF symbol
%
% This implementation assumes PL scrambling sequence index 0,
% which matches dvbs2WaveformGenerator.

    bitsPerSymbol = log2(double(phyParams.ModulationOrder));
    xfecLen = double(phyParams.FECFrameLength) / bitsPerSymbol;

    numSlots = xfecLen / 90;

    if abs(numSlots-round(numSlots)) > 1e-12
        error("XFECFRAME is not an integer number of slots.");
    end

    numSlots = round(numSlots);

    fp.headerInd = (1:90).';
    fp.plFrameSize = frameLength;
    fp.hasPilots = logical(phyParams.HasPilots);

    %% Calculate full-frame pilot indices

    pos = 90;
    pilotInd = [];

    for slot = 1:numSlots

        % Current 90-symbol data slot
        pos = pos + 90;

        % A 36-symbol pilot block follows every 16 data slots,
        % except when that position is the end of the frame.
        if fp.hasPilots && mod(slot,16) == 0 && slot < numSlots

            pilotInd = [
                pilotInd;
                (pos+1:pos+36).'                 %#ok<AGROW>
            ];

            pos = pos + 36;
        end
    end

    fp.pilotInd = pilotInd;
    fp.numPilotBlocks = numel(pilotInd)/36;

    if pos ~= frameLength
        error( ...
            "Pilot parser calculated %d symbols, but frameLength is %d.", ...
            pos,frameLength);
    end

    %% Generate known PL-scrambled pilot sequence

    if isempty(pilotInd)
        fp.refPilots = complex(zeros(0,1));
        return;
    end

    % Pilot indices relative to the first symbol after the PLHEADER.
    payloadPilotInd = pilotInd - 90;

    persistent scramblingSequence
    if isempty(scramblingSequence)
        % This is an internal MathWorks function used in their official
        % DVB-S2 receiver example.
        plScrambIntSeq = ...
            satcom.internal.dvbs.plScramblingIntegerSequence(0);

        % Map integer sequence:
        %   0 -> 1
        %   1 -> j
        %   2 -> -1
        %   3 -> -j
        complexMap = [1; 1j; -1; -1j];

        scramblingSequence = ...
            complexMap(plScrambIntSeq + 1);
    end

    if payloadPilotInd(end) > length(scramblingSequence)
        error("PL scrambling sequence is shorter than the PLFRAME.");
    end

    basePilot = (1+1j)/sqrt(2);

    fp.refPilots = ...
        basePilot .* scramblingSequence(payloadPilotInd);

end