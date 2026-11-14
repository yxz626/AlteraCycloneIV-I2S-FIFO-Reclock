# Cy4_I2Sq — I2S / DoP / native-DSD FIFO Reclock

An open FPGA reclocking design for the **EP4CE6F17C8** core board (Cyclone IV E)
with a **W9825G6JH SDRAM** and **EPCS4** configuration device. It accepts an I2S
stream from any source (e.g. a Raspberry Pi), buffers it in SDRAM, and re-emits it
from a locally selected low-jitter audio clock.

## Features

- Automatic input format / sample-rate detection: **8 PCM rates** (44.1 kHz – 384 kHz)
  with **16 / 20 / 24 / 32-bit** word lengths.
- Dual crystal oscillators (**45.1584 MHz** and **49.152 MHz**) with glitch-free
  global clock MUX selection based on the detected clock family.
- SDRAM ring buffer with Hi-end measures: water-mark slip inside a smooth fade,
  TPDF dither + rounding on the 20-bit path, fade-in / fade-out on every switch,
  and full I/O timing sign-off.
- **Native DSD** (DSD64 / 128 / 256 / 512) pass-through with bit-exact FIFO reclock.
- **DoP (DSD over PCM) v1.1** auto-detection and decode to native DSD, reusing the
  same SDRAM / FIFO reclock path.
- **AD1862** 20-bit output mode toggled by the `K3` key; LED rate-tier display
  driven from the always-on clock domain.
- **DSDOE** (`dsd_on`, pin F2) and **F0–F3** sample-rate indicators
  (pins C2 / A2 / A3 / A4), encoded per `f0~F3.txt`.
- Robust stream handling: short dropouts are absorbed by the SDRAM buffer,
  rate/format changes require consecutive valid measurements, and PCM-locked
  state is protected against false native-DSD detection.

## Verification status (at time of release)

| Item | Result |
|---|---|
| Quartus Prime 17.1 full compile | **0 errors / 0 warnings**, Flow Successful |
| TimeQuest | worst setup **+0.709 ns**, hold **+0.132 ns**, all corners positive, 0 unconstrained paths |
| Resources | LE 3,437 / 6,272 (55 %), memory bits 51,200 / 276,480 (19 %), pins 62 / 180 |
| Icarus Verilog functional sim | **`ALL CHECKS PASSED (0 errors)`** |
| DoP decode bit-exactness | 3,849 / 3,872 / 3,848 bits, all **0 errors** |
| Pin assignment check | **62 / 62 consistent** (`python sim/check_pins.py`) |
| Programming file | `Cy4_I2Sq.jic` (EPCS4, ASx1) |

## Repository layout

| Path | Contents |
|---|---|
| `Cy4_I2S_reclock.v` | Complete Verilog RTL (top-level + all submodules) |
| `Cy4_I2Sq.qpf` / `.qsf` / `.sdc` | Quartus project, pin assignments, timing constraints |
| `Cy4_I2Sq.jic` | Release firmware for EPCS4 |
| `output_files/` | SOF plus full compile / fitter / timing reports |
| `sim/` | Icarus Verilog testbench, SDRAM / clock-MUX models, simulation log, pin checker |
| `f0~F3.txt` | DSDOE / F0–F3 encoding definition |
| `工程概述.md` | Full design document (Chinese) — architecture, pin map, all fixes |
| `验证报告.md` | Requirement-to-verification report (Chinese) |
| `工程要求说明.txt` | Original requirements (Chinese) |
| `归档备份/` | Archive manifests with per-file SHA256 |

Reference datasheets and core-board schematics / PCB drawings are **not
redistributed** in this repository. Please obtain them from Intel (Cyclone IV),
Analog Devices (AD1862), Winbond (W9825G6JH) and your board vendor.
The local archive ZIPs are also intentionally excluded; Git history serves as
the version record.

## Build and reproduce

```sh
# Full compile
quartus_sh --flow compile Cy4_I2Sq

# Generate EPCS4 programming file
quartus_cpf -c -d EPCS4 -s EP4CE6 -m ASx1 \
    output_files/Cy4_I2Sq.sof Cy4_I2Sq.jic

# Functional simulation (Icarus Verilog)
iverilog -g2005 -o sim/tb.vvp -s tb \
    sim/altclkctrl_sim.v sim/sdram_model.v \
    sim/tb_Cy4_I2S_reclock.v Cy4_I2S_reclock.v
vvp sim/tb.vvp

# Pin assignment consistency check
python sim/check_pins.py
```

Program `Cy4_I2Sq.jic` through JTAG + Serial Flash Loader into the EPCS4, then
**power-cycle** the board so the FPGA configures itself from the serial flash.
Burning the SOF over JTAG alone is volatile and lost on power-off.

## Hardware notes

- Two oscillators are required: **E15 = 45.1584 MHz** (44.1 kHz family) and
  **E16 = 49.152 MHz** (48 kHz family). E16 also clocks the always-on management
  domain.
- LEDs are active-high; `K3` is active-low.
- Physical `PIN_F2` is `DSD_ON` (DSDOE), while the **signal** named F2
  (`fsel[2]`) is on `PIN_A3` — do not confuse them when wiring.
- The exact 62-pin assignment and clock/IO constraints are documented in
  `工程概述.md`.

For the full design rationale, debugging history and measured verification data,
see [工程概述.md](./工程概述.md) and [验证报告.md](./验证报告.md).
