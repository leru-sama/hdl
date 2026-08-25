# JESD204 Soft PCS - Standalone Project

Standalone project containing two JESD204 soft-PCS designs (extracted from the
ADI HDL repository) targeting a **transparent serdes with 20-bit parallel data
per lane** and a **ready/valid** application interface:

1. **`jesd204_soft_pcs_wrapper`** — minimal PCS: 8b10b encode/decode, comma
   (K28.5) pattern alignment, elastic FIFO with ready/valid handshake.
2. **`jesd204_pcs_link_training`** — full PCS with **link training** (CGS → ILAS
   → DATA) reusing ADI's `jesd204_tx_ctrl` / `jesd204_rx_ctrl` / `rx_cgs` /
   `ilas_monitor` / `tx_lane` / `rx_lane` (elastic buffers). The upper layer only
   deals with business payload over ready/valid; the PCS handles the whole
   bring-up handshake and **multi-lane de-skew**.

## Directory Structure

```
jesd204_pcs_standalone/
├── code/                              # All Verilog source files
│   ├── jesd204_pcs_link_training.v    # Top: PCS with link training (CGS/ILAS/DATA)
│   ├── jesd204_soft_pcs_wrapper.v     # Top: minimal soft-PCS wrapper
│   ├── jesd204_soft_pcs_fifo.v        # Elastic FIFO (minimal wrapper)
│   ├── jesd204_tx_ctrl.v              # TX link-training FSM (WAIT/CGS/ILAS/DATA)
│   ├── jesd204_rx_ctrl.v              # RX link-training FSM (RESET/CGS/SYNC)
│   ├── jesd204_rx_cgs.v               # CGS (K28.5 comma) detection
│   ├── jesd204_ilas_monitor.v         # ILAS parse + per-lane buffer release
│   ├── jesd204_tx_lane.v / rx_lane.v  # Per-lane TX/RX datapath + elastic buffer
│   ├── elastic_buffer.v               # Per-lane de-skew elastic buffer
│   ├── jesd204_8b10b_encoder.v/.._decoder.v
│   ├── jesd204_lmfc.v, jesd204_frame_mark.v, jesd204_scrambler.v, ...
│   └── ...                            # Supporting ADI modules
├── script/
│   ├── filelist.f                     # Source file list for tools
│   └── run_sim.sh                     # Simulation script (SWEEP=1 for skew sweep)
├── sim/
│   ├── jesd204_pcs_link_training_tb.v # 2-DUT link-training + multi-lane test
│   └── soft_pcs_wrapper_tb.v          # minimal-wrapper loopback test
├── Makefile
└── README.md
```

## The 2-DUT link-training testbench

`sim/jesd204_pcs_link_training_tb.v` instantiates **two independent PCS DUTs**
(no loopback of a single core):

* `tx_device` drives the line; `rx_device` recovers it.
* They are connected **only** through a 4-lane serdes model that injects a
  **random per-lane skew** (0..16 parallel-clock cycles).
* The RX parallel clock is **frequency-locked to the TX** (as a real
  CDR-recovered clock is) but **phase-shifted** — the elastic buffers absorb the
  phase offset and the inter-lane skew.

Bring-up is fully autonomous: `rx_device`'s `sync_n` drives `tx_device`'s
`sync_request_n`, so the TX only advances CGS→ILAS→DATA once the RX has locked.

**Data-integrity check (latency-tolerant scoreboard):** the TX sends a marker
preamble followed by a **per-lane distinct incrementing payload**; the RX
capture is anchored on the preamble and every payload beat is compared on all
lanes and octets. This proves both data integrity and correct lane de-skew.

### Results

Single run (default seed) and a 20-seed random-skew sweep all pass, including
worst-case skews such as `[16,16,16,7]`:

```
seed=5  skews=[16,16,16,7] : SUCCESS: 2-DUT link training + data transfer verified across 4 lanes
...
==== PASS=20 FAIL=0 ====
```

## Quick Start

Prerequisites: Icarus Verilog (`iverilog`), optionally GTKWave.

```bash
# 2-DUT link-training test (single random-skew pattern)
make simulate_link_training

# Sweep 20 random per-lane skew patterns through the 2-DUT test
make test_link_training_sweep

# Minimal soft-PCS wrapper sweep (serdes bit-slip 0..9)
make test_simple

# Everything
make test

# Or via the script (SWEEP=1 adds the skew sweep)
SWEEP=1 ./script/run_sim.sh
```

`script/filelist.f` lists all sources for synthesis/sim tools (paths relative to
`code/`); use `-f script/filelist.f`.

## `jesd204_pcs_link_training` interface

Clock/reset: `clk`, `reset`.

Configuration: `cfg_octets_per_multiframe`, `cfg_octets_per_frame`,
`cfg_lanes_disable`, `cfg_links_disable`, `cfg_disable_scrambler`,
`cfg_disable_char_replacement`, `cfg_mframes_per_ilas`, `cfg_skip_ilas`,
`cfg_continuous_cgs`, `cfg_continuous_ilas`.

TX application (ready/valid): `tx_valid`, `tx_ready`,
`tx_data[DATA_PATH_WIDTH*8*NUM_LANES-1:0]`, `tx_charisk`.

RX application (ready/valid): `rx_valid`, `rx_ready`, `rx_data`, `rx_charisk`,
`rx_notintable`, `rx_disperr`.

Serdes: `serdes_tx_data`, `serdes_rx_data` (`DATA_PATH_WIDTH*10` bits/lane = 20b
for `DATA_PATH_WIDTH=2`).

Link control/status: `sync_request_n` (from remote RX), `sync_n` (to remote TX),
`status_ctrl_state` (00=RESET, 01=CGS, 10=ILAS, 11=DATA),
`status_lane_cgs_state`, `status_lane_ifs_ready`, `ilas_config_*`,
`status_err_statistics_cnt`.

## Parameters

- `NUM_LANES` — serdes lanes (test uses 4).
- `NUM_LINKS` — links (1).
- `DATA_PATH_WIDTH` — 10b symbols per clock; **2 → 20-bit serdes lanes**.
- `ENABLE_FRAME_ALIGN_CHECK`, `ENABLE_CHAR_REPLACE`, `ELASTIC_BUFFER_SIZE`.

## License

BSD-1-Clause (same as the original ADI HDL repository).
