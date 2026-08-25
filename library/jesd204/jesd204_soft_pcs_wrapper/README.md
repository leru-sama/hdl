# jesd204_soft_pcs_wrapper

A top-level wrapper around the ADI JESD204 **soft PCS** (`jesd204_soft_pcs_tx` /
`jesd204_soft_pcs_rx`) for use with a *transparent* (data pass-through) serdes
that presents the raw parallel bits of each lane every PCS clock.

It adapts the raw, free-running serdes datapath to a standard **ready/valid**
handshake so an upper-layer application can source/sink 8b10b characters.

## Picking the parameters for your serdes

The serdes-facing bus is `DATA_PATH_WIDTH*10` bits per lane (with `IFC_TYPE=0`),
because the soft PCS processes `DATA_PATH_WIDTH` 8b10b symbols (10 bits each) per
lane per clock.

| serdes parallel width per lane | `DATA_PATH_WIDTH` |
| ------------------------------ | ----------------- |
| 10 bits                        | 1                 |
| **20 bits**                    | **2**             |
| 40 bits                        | 4                 |

For a serdes that presents **20 bits per lane** use the default
`DATA_PATH_WIDTH = 2` (2 × 10b symbols = 20 bits/lane/clock).

## Interface

Both the TX and RX application ports use a `valid`/`ready` handshake and share
the single PCS/serdes parallel clock domain `clk`.

### TX (application → serdes)
* `tx_ready` is asserted whenever the encoder can take a beat (always, out of
  reset — the line must be fed every cycle).
* When `tx_valid & tx_ready`, the supplied `tx_char`/`tx_charisk` are 8b10b
  encoded onto `tx_serdes_data`.
* When the application has nothing to send, an idle `/K28.5/` comma is inserted
  so the serdes always carries a valid, DC-balanced symbol stream.

### RX (serdes → application)
* Raw symbols on `rx_serdes_data` are pattern aligned (`rx_patternalign_en`) and
  8b10b decoded.
* Decoded beats are buffered in a small FWFT elastic FIFO and presented through
  `rx_valid`/`rx_ready` together with `rx_char`, `rx_charisk`, `rx_notintable`
  and `rx_disperr`.
* `rx_enable` gates capture (assert once alignment is achieved).
* Idle commas are filtered when `RX_FILTER_IDLE=1` so the application only sees
  payload.
* `rx_overflow` pulses if the application back-pressures long enough to overrun
  the elastic buffer. Because the serdes RX is free-running, the buffer absorbs
  only **bounded** back-pressure — the application must sustain the average line
  rate. Leave idle slack (runs of idle commas that the RX filters) if you need
  the RX side to tolerate stalls.

## Parameters

| name              | default | description                                        |
| ----------------- | ------- | -------------------------------------------------- |
| `NUM_LANES`       | 1       | number of serdes lanes                             |
| `DATA_PATH_WIDTH` | 2       | 8b10b symbols per lane per clock (2 ⇒ 20b/lane)    |
| `IFC_TYPE`        | 0       | 0: raw 10b·DPW, 1: Intel F-Tile 40b padded framing |
| `INVERT_LANES`    | 0       | invert serdes bits (lane polarity swap)            |
| `REGISTER_INPUTS` | 1       | pipeline the RX serdes inputs                      |
| `RX_FILTER_IDLE`  | 1       | drop idle `/K28.5/` beats before the RX FIFO       |
| `RX_FIFO_ADDR_W`  | 4       | RX elastic buffer depth = `2**RX_FIFO_ADDR_W`      |

## Simulation

A self-checking iverilog loopback testbench lives at
`../tb/soft_pcs_wrapper_tb.v`. It models an arbitrary serdes bit-slip, locks the
RX pattern aligner on an idle comma stream, then streams a bursty incrementing
payload through the TX ready/valid interface while back-pressuring the RX
ready/valid interface, verifying symbol integrity, 8b10b status and that the
elastic buffer never overflows.

```
./run_sim.sh          # runs a bit-slip sweep (offsets 0..9)
```
