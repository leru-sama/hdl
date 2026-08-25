# JESD204 Soft PCS Wrapper - Standalone Project

This is a standalone project containing the JESD204 Soft PCS Wrapper with ready/valid handshake interface, extracted from the ADI HDL repository.

## Directory Structure

```
jesd204_pcs_standalone/
├── code/                   # All Verilog source files
│   ├── jesd204_soft_pcs_wrapper.v   # Top-level wrapper
│   ├── jesd204_soft_pcs_fifo.v      # Elastic FIFO
│   ├── jesd204_soft_pcs_tx.v        # PCS TX module
│   ├── jesd204_8b10b_encoder.v      # 8b10b encoder
│   ├── jesd204_soft_pcs_rx.v        # PCS RX module
│   ├── jesd204_pattern_align.v      # Pattern alignment
│   └── jesd204_8b10b_decoder.v      # 8b10b decoder
├── script/                 # Build scripts and file lists
│   ├── filelist.f          # Source file list for synthesis/tools
│   └── run_sim.sh          # Simulation script
├── sim/                    # Testbench files
│   └── soft_pcs_wrapper_tb.v  # Self-checking testbench
├── Makefile                # Build automation
└── README.md               # This file
```

## Features

- **Transparent Serdes Interface**: 20-bit per lane parallel data interface
- **Ready/Valid Handshake**: Clean handshake interface for upper layer applications
- **8b10b Encoding/Decoding**: Full JESD204 8b10b coding support
- **Pattern Alignment**: Automatic comma (K28.5) detection and alignment
- **Elastic Buffer**: FWFT FIFO for clock domain crossing and back-pressure handling
- **Configurable**: Supports different DATA_PATH_WIDTH settings

## Quick Start

### Prerequisites

- Icarus Verilog (iverilog)
- GTKWave (optional, for waveform viewing)

### Running Simulation

1. **Single test** (default bitshift=0):
   ```bash
   make simulate
   ```

2. **Full sweep test** (bitshift 0-9):
   ```bash
   make test
   ```

3. **Using the script directly**:
   ```bash
   ./script/run_sim.sh
   ```

### Using the Source Files

The `script/filelist.f` contains all source files needed for synthesis or simulation tools. Paths are relative to the `code/` directory.

For synthesis tools, use:
```
-f script/filelist.f
```

## Interface Description

### Top-Level Wrapper (`jesd204_soft_pcs_wrapper`)

#### Clock/Reset
- `clk`: Clock input
- `reset`: Synchronous reset

#### TX Interface (Application → Serdes)
- `tx_valid`: Data valid from application
- `tx_ready`: Ready to accept data
- `tx_data[15:0]`: 8-bit characters (2 symbols per beat)
- `tx_charisk[1:0]`: K-character flags

#### RX Interface (Serdes → Application)
- `rx_valid`: Data valid to application
- `rx_ready`: Application ready to accept data
- `rx_data[15:0]`: 8-bit characters (2 symbols per beat)
- `rx_charisk[1:0]`: K-character flags
- `rx_notintable[1:0]`: Not-in-table error flags
- `rx_disperr[1:0]`: Disparity error flags

#### Serdes Interface
- `serdes_tx_data[19:0]`: 20-bit encoded data to serdes
- `serdes_rx_data[19:0]`: 20-bit data from serdes

#### Status
- `rx_overflow`: Elastic buffer overflow flag
- `rx_enable`: Enable RX capture (after pattern alignment)

## Configuration Parameters

- `DATA_PATH_WIDTH`: Number of 10b symbols per clock (default: 2 for 20-bit lanes)
- `IFC_TYPE`: Interface type (0 for parallel, default)

## License

BSD-1-Clause (same as original ADI HDL repository)