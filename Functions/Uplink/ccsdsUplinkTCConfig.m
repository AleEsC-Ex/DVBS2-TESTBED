function tcCfg = ccsdsUplinkTCConfig(config, dataFormat)
%CCSDSUPLINKTCCONFIG Build the CCSDS TC format object from the testbed config.
%
%   tcCfg = ccsdsUplinkTCConfig(config) returns the ccsdsTCConfig for a
%   CLTU, which is what carries data.
%
%   tcCfg = ccsdsUplinkTCConfig(config, dataFormat) returns it for one of
%   the three things PLOP-2 puts on the air:
%
%     "CLTU"                  start sequence + codeblock + tail
%     "acquisition sequence"  alternating symbols, sent once at session
%                             start so the receiver can lock its loops
%     "idle sequence"         alternating symbols, sent whenever no command
%                             is ready, so the carrier never drops
%
%   It exists so that the transmitter and receiver cannot drift apart. Every
%   field here has to match exactly at both ends -- a different coding
%   scheme, codeword length or randomizer setting produces a waveform the
%   other side simply cannot decode, with no error message beyond silence.
%   Deriving both from one function makes that impossible.
%
%   WHY THE DATAFORMAT BRANCH IS NOT COSMETIC. The object's property set
%   CHANGES with DataFormat. A CLTU config exposes the coding properties; an
%   acquisition or idle config exposes only DataFormat and Modulation, and
%   assigning ChannelCoding to one of those is an error, not a no-op. So the
%   coding block below has to be skipped rather than merely ignored.
%
%   NOTE ON BPSK: with Modulation = "BPSK" the object exposes only
%   DataFormat, ChannelCoding, LDPCCodewordLength, HasTailSequence and
%   Modulation. SymbolRate, SamplesPerSymbol, ModulationIndex,
%   SubcarrierFrequency and PCMFormat are not properties of a BPSK
%   configuration at all -- setting them errors. ccsdsTCWaveform then
%   returns one sample per symbol, and the symbol rate is whatever the pulse
%   shaper and the radio make it. See ccsdsUplinkPulseShape.m.
%
%   NOTE ON HASRANDOMIZER: under LDPC the randomizer is mandatory and MATLAB
%   applies it regardless, so the property is inactive and setting it has no
%   effect. It is still set here because it IS active under BCH.

    if nargin < 2 || isempty(dataFormat)
        dataFormat = "CLTU";
    end
    dataFormat = string(dataFormat);

    tcCfg = ccsdsTCConfig;
    tcCfg.DataFormat = dataFormat;
    tcCfg.Modulation = "BPSK";

    if dataFormat ~= "CLTU"
        % Acquisition and idle sequences carry no data, so they have no
        % coding, no randomizer and no tail. The object does not even define
        % those properties in this mode.
        return;
    end

    tcCfg.ChannelCoding = config.uplink.channelCoding;
    tcCfg.HasRandomizer = config.uplink.hasRandomizer;
    if config.uplink.channelCoding == "LDPC"
        tcCfg.LDPCCodewordLength = config.uplink.ldpcCodewordLength;
        tcCfg.HasTailSequence = config.uplink.hasTailSequence;
    end
end
