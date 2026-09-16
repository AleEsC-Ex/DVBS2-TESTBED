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

## Mission scenario

This testbed emulates the RF link between a CubeSat and its ground
station, split the way a real one would be: a low-rate, easy-to-close UHF
link handles commanding, while a separate, higher-frequency link carries
the payload downlink at a rate UHF could never sustain.

- **Payload downlink, S-band (2 GHz here):** the CubeSat transmits its
  payload data down to the ground station over DVB-S2 — an increasingly
  common choice for smallsat downlinks that need more throughput than
  legacy formats, backed by mature, off-the-shelf ground equipment.
- **Command uplink, UHF (500 MHz here):** the CubeSat's receive antenna is
  UHF, so commanding runs over CCSDS Telecommand — the standard real
  missions use for spacecraft commanding — instead of riding on the
  S-band carrier.

Mapped onto the five processes: **S1a/S1b are the CubeSat** — it
transmits its DVB-S2 downlink at 2 GHz and receives commands on its UHF
receiver at 500 MHz, both on the same physical radio (`192.168.10.2`).
**S2a/S2b/S3 are the ground station** — it receives the S-band downlink
and transmits back over UHF whatever the link needs to send up: ACM
feedback (so the CubeSat knows what MODCOD the channel currently
supports) and selective-repeat ARQ retransmit requests for any payload
data the ground station didn't receive cleanly.

That's why "forward link" in this repo means CubeSat-to-ground and
"return link" (or "uplink") means ground-to-CubeSat — the naming follows
the mission, not just which USRP happens to transmit first.

## What it does

- **Forward link, CubeSat → ground (2 GHz, S-band):** DVB-S2 (ETSI EN 302
  307-1) — waveform generation, adaptive MODCOD selection driven by
  measured channel conditions, and the full physical-layer receive chain
  (frame sync, timing/CFO/phase recovery, LDPC/BCH decode).
- **Return link, ground → CubeSat (500 MHz, UHF):** CCSDS Telecommand
  (CCSDS 231.0-B, PLOP-2) — carries ACM feedback reports and
  selective-repeat ARQ retransmit requests back to the CubeSat.
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
needed) — flip it in `Testbed/Functions/Testbed/dvbs2TestbedConfig.m` to
run on real USRPs. Per-run console logs land in `logs/` (gitignored).

## Current status

Measured on hardware, all runs reaching well into the 32APSK end of the
MODCOD ladder:

| Configuration | Frame loss |
|---|---|
| Original single-process design | 1.3% – 10.8% (run-to-run RF variance) |
| Split design, no generation pipeline | 11.8% – 25.9% |
| Split design, with generation pipeline | **8.0% – 11.1%** |

**Known open issues:**
- The uplink TX radio underruns in several occasion (4092 underruns in 420s) which probably is producing that the reciever don't recieve the CLTUs correctly, loosing some of them in the process and failing at the recovery in others (almost 30-40% of the CLTUs are lost).
- There is no way to cut the link in case it is necessary and look for a re-establishment of the link. With this I mean that is not possible to return to the link acquisition loop in order to keep transmitting if the actual link is lost or not well established.


See `dvbs2TestbedConfig.m`'s inline comments (particularly around
`config.tx.genBudgetFraction`) for the full measurement history behind
these numbers.

## Repository layout

All MATLAB source lives under `Testbed/`, kept separate from the README,
docs, and the PowerShell entry points at the root:

```
README.md, docs/ARCHITECTURE.md                Documentation (you are here)
Launch-DVBS2Testbed.ps1, Stop-DVBS2Testbed.ps1  Multi-process launcher

Testbed/
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
```

---

*Erasmus exchange project, Aarhus University.*
