# JESD204 PCS with Link Training — Module Documentation

## 1. Overview

`jesd204_pcs_link_training` is a complete JESD204 PCS (Physical Coding Sublayer) with link training support. It implements the full TX/RX datapath including:

- **TX**: scrambler, character replacement (/A/ /F/), 8b10b encoder
- **RX**: 8b10b decoder, descrambler, frame-align monitor, per-lane elastic buffer (lane de-skew), CGS detect
- **Link training FSM**: CGS → ILAS → DATA
- **Frame-align error detection**: auto resync via `ENABLE_FRAME_ALIGN_ERR_RESET`

The design operates in a **single clock domain** (`clk`). In simulation, the serdes model injects per-lane skew to mimic real-world channel mismatches.

---

## 2. Module Hierarchy

```
jesd204_pcs_link_training  (top-level)
│
├── jesd204_lmfc  (1)
│   └── Generates local multi-frame clock (LMFC) edges
│
├── jesd204_frame_mark  (1)
│   └── Generates frame/multiframe boundary markers (sof, eof, somf, eomf)
│
├── jesd204_tx_ctrl  (1)
│   ├── sync_bits  (1)  — CDC for sync_request_n
│   └── TX state machine: WAIT → CGS → ILAS → DATA
│
├── jesd204_tx_lane  (NUM_LANES)
│   ├── jesd204_scrambler  (1, TX mode)
│   ├── util_pipeline_stage  (1)
│   └── jesd204_frame_align_replace  (1, TX mode)
│
├── jesd204_8b10b_encoder  (NUM_LANES × DATA_PATH_WIDTH)
│
├── jesd204_8b10b_decoder  (NUM_LANES × DATA_PATH_WIDTH)
│
├── jesd204_rx_ctrl  (1)
│   └── RX state machine: RESET → WAIT_FOR_PHY → CGS → SYNCHRONIZED
│
└── jesd204_rx_lane  (NUM_LANES)
    ├── util_pipeline_stage  (2)
    ├── align_mux  (1)
    ├── jesd204_rx_frame_align  (1, if ENABLE_FRAME_ALIGN_CHECK)
    │   ├── jesd204_frame_mark  (1)
    │   └── jesd204_frame_align_replace  (1, RX mode)
    ├── jesd204_scrambler  (1, descrambler mode)
    ├── elastic_buffer  (1)
    ├── jesd204_ilas_monitor  (1)
    └── jesd204_rx_cgs  (1)
```

---

## 3. Top-Level Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `NUM_LANES` | 1 | Number of serdes lanes |
| `NUM_LINKS` | 1 | Number of JESD204 links |
| `DATA_PATH_WIDTH` | 8 | 8b symbols per clock per lane (2, 4, or 8). DPW=8 → 80b/lane serdes; DPW=4 → 40b/lane; DPW=2 → 20b/lane |
| `ENABLE_FRAME_ALIGN_CHECK` | 1 | Instantiate `rx_frame_align` to monitor /A/ /F/ positioning |
| `ENABLE_CHAR_REPLACE` | 1 | TX inserts /A/ at EOMF, /F/ at EOF via `jesd204_frame_align_replace` |
| `ENABLE_FRAME_ALIGN_ERR_RESET` | 1 | rx_ctrl resets to CGS when frame-align error threshold is crossed |
| `FRAME_ALIGN_ERR_THRESHOLD` | 16 | Error count before asserting resync |
| `ELASTIC_BUFFER_SIZE` | 256 | Per-lane elastic buffer depth in bits |

---

## 4. Top-Level Port Definition

### 4.1 Clock / Reset

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `clk` | in | 1 | Single clock domain for TX/RX datapaths |
| `reset` | in | 1 | Synchronous active-high reset |

### 4.2 Configuration

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `cfg_octets_per_multiframe` | in | [9:0] | Number of octets in one multiframe (K). Must be multiple of DATA_PATH_WIDTH. Typical: 32 |
| `cfg_octets_per_frame` | in | [7:0] | Number of octets in one frame (F) |
| `cfg_lanes_disable` | in | [NUM_LANES-1:0] | Per-lane disable (1 = disabled) |
| `cfg_links_disable` | in | [NUM_LINKS-1:0] | Per-link disable (1 = disabled) |
| `cfg_disable_scrambler` | in | 1 | 1 = bypass scrambler/descrambler |
| `cfg_disable_char_replacement` | in | 1 | 1 = do not insert /A/ /F/ alignment chars |
| `cfg_mframes_per_ilas` | in | [7:0] | Number of multiframes in one ILAS sequence (M) |
| `cfg_skip_ilas` | in | 1 | 1 = skip ILAS, go straight from CGS to DATA |
| `cfg_continuous_cgs` | in | 1 | 1 = stay in CGS (initial bring-up) |
| `cfg_continuous_ilas` | in | 1 | 1 = repeat ILAS continuously |

### 4.3 TX Application Interface (ready/valid handshake)

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `tx_valid` | in | 1 | Application data valid strobe |
| `tx_ready` | out | 1 | PCS ready to accept tx_data (during DATA phase) |
| `tx_data` | in | [DATA_PATH_WIDTH×8×NUM_LANES-1:0] | Application payload, packed per-lane: `[lane0_char0, lane0_char1, ...]` |
| `tx_charisk` | in | [DATA_PATH_WIDTH×NUM_LANES-1:0] | Per-character control flag (1 = control character) |

### 4.4 RX Application Interface (ready/valid handshake)

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `rx_valid` | out | 1 | rx_data / rx_charisk are valid this cycle |
| `rx_ready` | in | 1 | Application ready to consume RX data |
| `rx_data` | out | [DATA_PATH_WIDTH×8×NUM_LANES-1:0] | Decoded application payload |
| `rx_charisk` | out | [DATA_PATH_WIDTH×NUM_LANES-1:0] | Per-character control flag |
| `rx_notintable` | out | [DATA_PATH_WIDTH×NUM_LANES-1:0] | 8b10b not-in-table error per char |
| `rx_disperr` | out | [DATA_PATH_WIDTH×NUM_LANES-1:0] | 8b10b disparity error per char |

### 4.5 Serdes Interface

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `serdes_tx_data` | out | [DATA_PATH_WIDTH×10×NUM_LANES-1:0] | Parallel 8b10b-encoded symbols to serdes TX. Packed per lane: `[lane0_sym0, lane0_sym1, ...]` |
| `serdes_rx_data` | in | [DATA_PATH_WIDTH×10×NUM_LANES-1:0] | Parallel 8b10b symbols from serdes RX |

### 4.6 Link Control / Status

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `sync_request_n` | in | [NUM_LINKS-1:0] | From remote RX. ACTIVE-LOW. Triggers TX state machine to leave WAIT and enter CGS→ILAS→DATA |
| `status_ctrl_state` | out | [1:0] | Current TX state: 2'b00=RESET, 2'b01=CGS, 2'b10=ILAS, 2'b11=DATA |
| `status_lane_cgs_state` | out | [2×NUM_LANES-1:0] | Per-lane CGS detector state (2 bits each) |
| `status_lane_ifs_ready` | out | [NUM_LANES-1:0] | Per-lane ILAS monitor IFS ready |
| `sync_n` | out | [NUM_LINKS-1:0] | To remote TX. ACTIVE-LOW. Low = "I am in CGS, please keep sending K28.5" |

### 4.7 ILAS Configuration (RX → Application)

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `ilas_config_valid` | out | [NUM_LANES-1:0] | One-hot: lane i has valid ILAS config |
| `ilas_config_addr` | out | [NUM_LANES×2-1:0] | 2-bit config address per lane (0=R/ADJCNT, 1=Q/ADJDIR, 2=F/MF, 3=K/CS) |
| `ilas_config_data` | out | [NUM_LANES×DATA_PATH_WIDTH×8-1:0] | Config data per lane |

### 4.8 Error Status

| Port | Dir | Width | Description |
|------|-----|-------|-------------|
| `status_err_statistics_cnt` | out | [32×NUM_LANES-1:0] | 32-bit error counter per lane. Increments on 8b10b not-in-table / disparity errors. Reset on `event_data_phase` |
| `status_frame_align_err_cnt_0` | out | [NUM_LANES-1:0] | Frame-align error counter for lane 0 (8 bits each) |
| `event_frame_alignment_error` | out | 1 | Sticky OR of per-lane frame-align error threshold. Pulses high when any enabled lane has ≥ threshold errors and `ENABLE_FRAME_ALIGN_ERR_RESET=1` |

---

## 5. Functional Description

### 5.1 TX Datapath

```
Application Data
       │
       ▼
  [Scrambler]
       │
       ▼
  [Frame Align Replace]  ← inserts /A/ at EOMF, /F/ at EOF
       │
       ▼
  [8b10b Encoder]  ← per character, per lane
       │
       ▼
  serdes_tx_data
```

**TX State Machine (`jesd204_tx_ctrl`)**:

| State | Description | tx_ready | serdes output |
|-------|-------------|----------|---------------|
| WAIT | Waiting for sync_request_n deassertion | 0 | Idle |
| CGS | Code Group Synchronization | 0 | `/K28.5/` (0xBC) on all lanes |
| ILAS | Initial Lane Alignment Sequence | 0 | `/R/`, config data, `/Q/`, `/A/` |
| DATA | Normal data transfer | 1 | Scrambled application data with /A/ /F/ |

### 5.2 RX Datapath

```
serdes_rx_data
       │
       ▼
  [8b10b Decoder]
       │
       ▼
  [CGS Detection]  ← detects K28.5 presence
       │
       ▼
  [Align Mux]  ← rotates data to align first non-K char
       │
       ▼
  [Frame Align Check]  ← verifies /A/ /F/ positioning
       │
       ▼
  [Descrambler]
       │
       ▼
  [Elastic Buffer]  ← lane de-skew
       │
       ▼
  [ILAS Monitor]  ← extracts config data
       │
       ▼
  rx_data / rx_charisk
```

**RX State Machine (`jesd204_rx_ctrl`)**:

| State | Description | sync_n | buffer_release_n |
|-------|-------------|--------|------------------|
| RESET | Reset state | high | high |
| WAIT_FOR_PHY | Waiting for physical layer ready | high | high |
| CGS | Code Group Synchronization | low | high |
| SYNCHRONIZED | Link established, data flowing | high | controlled release |

### 5.3 Link Training Flow

1. **Sync Request**: RX asserts `sync_request_n` low (or `cfg_continuous_cgs=1` keeps TX in CGS)
2. **CGS Phase**: TX sends continuous `/K28.5/` (0xBC). RX CGS detector verifies sustained K28.5 without errors
3. **ILAS Phase**: TX sends configuration data. RX ILAS monitor captures lane config (ADJCNT, ADJDIR, F/MF, K/CS)
4. **DATA Phase**: TX sends scrambled application data. RX elastic buffers are released simultaneously for lane de-skew
5. **Error Recovery**: If frame-align errors exceed threshold, `event_frame_alignment_error` pulses and RX drops back to RESET/CGS

---

## 6. Sub-Module Descriptions

### 6.1 `jesd204_lmfc`

Generates the Local Multi-Frame Clock (LMFC). Produces `lmfc_edge` (once per multiframe), `lmfc_clk`, `lmfc_counter`, and SYSREF-related signals.

### 6.2 `jesd204_frame_mark`

Generates frame/multiframe boundary markers (`sof`, `eof`, `somf`, `eomf`) as per-lane bit vectors. Supports `DATA_PATH_WIDTH` = 4, 6, 8.

### 6.3 `jesd204_tx_ctrl`

TX control state machine. Manages WAIT → CGS → ILAS → DATA transitions. Generates `lane_cgs_enable`, `eof_reset`, `tx_ready`, and ILAS data (`ilas_data`, `ilas_charisk`). Uses `sync_bits` CDC for `sync_request_n`.

### 6.4 `jesd204_tx_lane`

Per-lane TX datapath. Multiplexes between CGS (K28.5), ILAS, and user data. Applies scrambling and character replacement.

### 6.5 `jesd204_8b10b_encoder`

Single-character 8b10b encoder with running disparity. Supports K28.x control characters.

### 6.6 `jesd204_8b10b_decoder`

Single-character 8b10b decoder with running disparity. Detects K28.X, reports not-in-table and disparity errors.

### 6.7 `jesd204_rx_ctrl`

RX control state machine. Drives `cgs_reset`, `ifs_reset`, `sync_n`, `phy_en_char_align`. Transitions: RESET → WAIT_FOR_PHY → CGS → SYNCHRONIZED.

### 6.8 `jesd204_rx_lane`

Per-lane RX datapath. Performs character decoding, IFS detection, frame alignment, descrambling, elastic buffering, ILAS monitoring, and CGS detection.

### 6.9 `jesd204_rx_cgs`

Per-lane CGS detector. Detects sustained K28.5 presence without errors. States: INIT → CHECK → DATA.

### 6.10 `jesd204_ilas_monitor`

Extracts ILAS configuration words from the received data stream. Captures R/ADJCNT, Q/ADJDIR, F/MF, K/CS.

### 6.11 `jesd204_scrambler`

15-bit LFSR-style scrambler/descrambler. Supports scramble and descramble modes.

### 6.12 `jesd204_frame_align_replace`

Inserts or verifies alignment characters (/A/ = 0x7C, /F/ = 0xFC) at EOMF and EOF boundaries.

### 6.13 `align_mux`

Rotates incoming data bus by programmable offset to align first non-K28.5 character to position 0.

### 6.14 `elastic_buffer`

Per-lane elastic buffer for lane de-skew. Supports asymmetric input/output widths.

### 6.15 `util_pipeline_stage`

Configurable pipeline register stage with `(* shreg_extract = "no" *)` attribute.

### 6.16 `sync_bits`

CDC synchronizer for multi-bit signals (2-FF synchronizer chain).

### 6.17 `sync_event`

Event synchronization across clock domains using toggle-based handshake.

### 6.18 `jesd204_pattern_align`

10-bit pattern aligner for raw serdes data. Searches for known 10-bit pattern and rotates data stream.

### 6.19 `jesd204_rx_frame_align`

RX frame alignment monitor. Verifies /A/ and /F/ positioning and counts alignment errors.

---

## 7. Signal Reference

### 7.1 Clock / Reset
- `clk` : Single clock domain for both TX and RX datapaths. In silicon this would typically be the device clock; in simulation both sides share this clock and the serdes model injects per-lane skew.

### 7.2 Configuration
- `cfg_octets_per_multiframe` : Number of octets in one multiframe (K). Must be a multiple of DATA_PATH_WIDTH. Typical: 32.
- `cfg_octets_per_frame` : Number of octets in one frame (F).
- `cfg_lanes_disable` : Per-lane disable (1 = lane disabled).
- `cfg_links_disable` : Per-link disable (1 = link disabled).
- `cfg_disable_scrambler` : 1 = bypass scrambler/descrambler.
- `cfg_disable_char_replacement` : 1 = do not insert /A/ /F/ alignment chars.
- `cfg_mframes_per_ilas` : Number of multiframes in one ILAS sequence (M).
- `cfg_skip_ilas` : 1 = skip ILAS, go straight from CGS to DATA.
- `cfg_continuous_cgs` : 1 = stay in CGS (used for initial bring-up).
- `cfg_continuous_ilas` : 1 = repeat ILAS continuously.

### 7.3 TX Application Interface
- `tx_valid` : Application data valid strobe.
- `tx_ready` : PCS ready to accept tx_data (during DATA phase).
- `tx_data` : Application payload, packed as `[lane0_char0, lane0_char1, ..., lane3_char7]`.
- `tx_charisk` : Per-character control flag. 1 = this 8b character is a control character (Kx.y).

### 7.4 RX Application Interface
- `rx_valid` : rx_data / rx_charisk are valid this cycle.
- `rx_ready` : Application is ready to consume RX data.
- `rx_data` : Decoded application payload.
- `rx_charisk` : Per-character control flag.
- `rx_notintable` : 8b10b not-in-table error per char.
- `rx_disperr` : 8b10b disparity error per char.

### 7.5 Serdes Interface
- `serdes_tx_data` : Parallel 8b10b-encoded symbols to the serdes TX. Packed per lane: `[lane0_sym0, lane0_sym1, ..., lane3_sym7]`. Each symbol is 10 bits wide.
- `serdes_rx_data` : Parallel 8b10b symbols from the serdes RX. Same packing as serdes_tx_data.

### 7.6 Link Control / Status
- `sync_request_n` : From the remote RX device. ACTIVE-LOW. Driven by the RX's rx_ctrl: low = "please send CGS". This is the input that triggers the TX state machine to leave WAIT and enter CGS -> ILAS -> DATA.
- `status_ctrl_state` : Current TX state machine state. 2'b00 = RESET, 2'b01 = CGS, 2'b10 = ILAS, 2'b11 = DATA.
- `status_lane_cgs_state` : Per-lane CGS detector state (2 bits each).
- `status_lane_ifs_ready` : Per-lane ILAS monitor IFS ready.
- `sync_n` : To the remote TX device. ACTIVE-LOW. Driven by the local RX's rx_ctrl: low = "I am in CGS, please keep sending K28.5". Registered for CDC.

### 7.7 ILAS Configuration
- `ilas_config_valid` : One-hot: lane i has valid ILAS config.
- `ilas_config_addr` : 2-bit config address per lane (0=R/ADJCNT, 1=Q/ADJDIR, 2=F/MF, 3=K/CS).
- `ilas_config_data` : Config data per lane.

### 7.8 Error Status
- `status_err_statistics_cnt` : 32-bit error counter per lane. Increments on 8b10b not-in-table / disparity errors. Reset on each event_data_phase (start of DATA).
- `status_frame_align_err_cnt_0` : Frame-align error counter for lane 0 (8 bits). Mirror of the internal per-lane frame_align_err_cnt[i] for the first lane only.
- `event_frame_alignment_error` : Sticky OR of per-lane frame_align_err_thresh_met. Pulses high when any enabled lane has accumulated >= FRAME_ALIGN_ERR_THRESHOLD frame-align errors and ENABLE_FRAME_ALIGN_ERR_RESET=1. This triggers rx_ctrl to drop back to RESET/CGS.

---

## 8. Simulation Verification

The design is verified with a 2-DUT point-to-point testbench (`jesd204_pcs_link_training_tb`):

- **Configuration**: 4 lanes, DATA_PATH_WIDTH=8, max skew 16 cycles
- **Test**: TX transmits 4 marker beats + 256 payload beats with per-lane distinct incrementing data
- **Verification**: RX captures all valid application beats, locates the preamble, and verifies every payload byte across all lanes and octets
- **Result**: 256/256 matches, 0 mismatches, TX reaches DATA state, RX de-skew complete

```
2-DUT Test Results (DPW=8):
  Lane skews:  7, 8, 7, 8
  TX beats:    260
  RX beats:    900
  Anchor idx:  133
  Matches:     256
  Mismatches:  0
  Frame-align err cnt (lane 0): 3
  TX state:    11
========================================
SUCCESS: 2-DUT link training + data transfer verified across 4 lanes (DPW=8)
```

---

## 9. Design Notes

1. **Single Clock Domain**: The top-level design operates in a single clock domain. In silicon, `clk` would be the device clock; in simulation, the serdes model injects per-lane skew.

2. **Active-Low Sync Protocol**: `sync_request_n` (input to TX) and `sync_n` (output from RX) are active-low. The RX asserts `sync_n` low to indicate it is in CGS and request the TX keep sending K28.5.

3. **Frame-Align Error Recovery**: When `ENABLE_FRAME_ALIGN_ERR_RESET=1`, if any lane's frame-align error counter exceeds `FRAME_ALIGN_ERR_THRESHOLD` (default 16), `event_frame_alignment_error` pulses and `rx_ctrl` drops back to RESET/CGS.

4. **Elastic Buffer Release Coordination**: `buffer_release_n` is gated by a coordinated delay across all lanes. The top-level counts LMFC edges and only releases all buffers simultaneously when `all_buffer_ready_n` is asserted.

5. **DATA_PATH_WIDTH Constraints**: The design primarily supports `DATA_PATH_WIDTH = 4` and `8`. Special handling exists for `DPW=8` with `F*K % 8 == 4`, where multiframe boundaries fall mid-beat.

6. **Character Replacement Interaction with Scrambling**: Character replacement is disabled when scrambling is disabled, because the TX and RX `jesd204_frame_align_replace` logic relies on being able to recognize scrambler output patterns.
