# DVB-S2 Satellite Ground-Segment Testbed

A software-defined DVB-S2 forward link and CCSDS Telecommand return link,
built on two NI USRPs, for an Erasmus exchange project at Aarhus
University. The system implements a full transmit/receive chain with
adaptive coding & modulation (ACM), selective-repeat ARQ, and independent
BER/PER measurement — not a simulation of one, an actual RF link running
on real radios.

**Platform:** MATLAB R2026a, Windows 11
**Hardware:** NI USRP-2920 (`192.168.10.2` — downlink TX @ 2 GHz + uplink
RX @ 500 MHz, shared IP) · NI USRP-2922 (`192.168.10.3` — uplink TX)

For the full architecture, function-by-function reference, and the ACM
control-loop deep dive, see **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**.
This README is the short version.

---

## What it does

- **Forward link (2 GHz):** DVB-S2 (ETSI EN 302 307-1) — waveform
  generation, adaptive MODCOD selection driven by measured channel
  conditions, and the full physical-layer receive chain (frame sync,
  timing/CFO/phase recovery, LDPC/BCH decode).
- **Return link (500 MHz):** CCSDS Telecommand (CCSDS 231.0-B, PLOP-2) —
  carries ACM feedback reports and selective-repeat ARQ retransmit
  requests back to the transmitter.
- **Independent measurement:** per-packet CRC-8 and self-synchronizing
  BER/PER against a deterministic reference payload, decoupled from
  whether a frame decoded successfully — the system's own accuracy check,
  not just a pass/fail log.
- **Two operating modes:** real RF over the USRPs (`config.useSDR = true`),
  or a fully simulated channel model over TCP loopback with no hardware
  attached at all (`config.useSDR = false`) — useful for developing and
  testing the DSP/control logic without radio access.

## Architecture at a glance

Five MATLAB processes, each its own script, talking over TCP on loopback,
startable in any order:

| Process | Role |
|---|---|
| `S1a_Transmitter.m` | Owns both radios (shared IP constraint). Radio I/O only — generates nothing of its own. |
| `S1b_ACMControl.m` | Waveform generation, ACM policy, transmit FIFO, CCSDS uplink receive chain. |
| `S2a_RFAcquisition.m` | Downlink front end (DC block, RSSI, AGC) and, in RF mode, the return-link gateway. |
| `S2b_Reciever.m` | The DVB-S2 physical-layer DSP chain: frame sync, timing/CFO/phase recovery, per-frame SNR estimation. |
| `S3_ProcessingUnit.m` | LDPC/BCH decode, BER/PER measurement, and selective-repeat ARQ. |

```
S1b --[TX blocks]--> S1a --[2 GHz RF]--> S2a --[TCP]--> S2b --[TCP]--> S3
S1a <--[raw uplink samples]-- ... <--[500 MHz RF, relayed]-- S2a <--[ARQ / ACM feedback]-- S2b, S3
```

Full port map, per-function breakdown, and the reasoning behind every
design decision (why the radios had to split this way, how ACM's
hysteresis works, why generation runs one block ahead of what it sends)
are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Getting started

```powershell
# Launches all five processes, staggered, each in its own MATLAB window
.\Launch-DVBS2Testbed.ps1

# Stops a run early / cleans up a previous run's leftover windows
.\Stop-DVBS2Testbed.ps1
```

Runs default to `config.useSDR = false` (simulated channel, no hardware
needed) — flip it in `Functions/Testbed/dvbs2TestbedConfig.m` to run on
real USRPs. Per-run console logs land in `logs/` (gitignored).

## Current status

Measured on hardware, all runs reaching well into the 32APSK end of the
MODCOD ladder:

| Configuration | Frame loss |
|---|---|
| Original single-process design | 1.3% – 10.8% (run-to-run RF variance) |
| Split design, no generation pipeline | 11.8% – 25.9% |
| Split design, with generation pipeline | **8.0% – 11.1%** |

**Known open issues:**
- A missing MATLAB event-queue yield in `S2a_RFAcquisition.m` was breaking
  the retransmit-request relay entirely (0 requests relayed for a whole
  run). Fixed in code; not yet re-verified on hardware.
- Every hardware run this session showed RSSI collapsing permanently to
  the noise floor partway through, traced to a UHD driver crash
  (`WSAENOBUFS`, Windows socket-buffer exhaustion). Possibly connected to
  the same root cause as the retransmit bug above; not yet confirmed.

See `dvbs2TestbedConfig.m`'s inline comments (particularly around
`config.tx.genBudgetFraction`) for the full measurement history behind
these numbers.

## Repository layout

```
S1a_Transmitter.m, S1b_ACMControl.m,          The five live processes
S2a_RFAcquisition.m, S2b_Reciever.m,
S3_ProcessingUnit.m

Functions/                                     Core DSP/PHY functions
Functions/Uplink/                              CCSDS Telecommand chain
Functions/TCP/                                 Inter-process transport
Functions/Serialization/                       Wire-format pack/unpack
Functions/Testbed/                             Shared config + ACM policy

sdr_test/                                       Standalone hardware bring-up
                                                 and commissioning scripts

docs/ARCHITECTURE.md                            Full architecture reference
Launch-DVBS2Testbed.ps1, Stop-DVBS2Testbed.ps1  Multi-process launcher
```

---

*Erasmus exchange project, Aarhus University.*
