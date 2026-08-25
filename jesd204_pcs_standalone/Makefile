####################################################################################
# JESD204 Soft PCS Wrapper - Standalone Project
####################################################################################

# Project directories
CODE_DIR := code
SCRIPT_DIR := script
SIM_DIR := sim

# Output directory
OUT_DIR := /tmp/logs

# Source files
SRCS := $(SIM_DIR)/soft_pcs_wrapper_tb.v \
        $(CODE_DIR)/jesd204_soft_pcs_wrapper.v \
        $(CODE_DIR)/jesd204_soft_pcs_fifo.v \
        $(CODE_DIR)/jesd204_soft_pcs_tx.v \
        $(CODE_DIR)/jesd204_8b10b_encoder.v \
        $(CODE_DIR)/jesd204_soft_pcs_rx.v \
        $(CODE_DIR)/jesd204_8b10b_decoder.v \
        $(CODE_DIR)/jesd204_pattern_align.v

# Default target
.PHONY: all
all: simulate

# Compile all sources
.PHONY: compile
compile:
	@mkdir -p $(OUT_DIR)
	iverilog -g2005 -o $(OUT_DIR)/pcs_wrapper.vvp $(SRCS)

# Run simulation with default bitshift (0)
.PHONY: simulate
simulate: compile
	vvp $(OUT_DIR)/pcs_wrapper.vvp

# Run full sweep test (bitshift 0-9)
.PHONY: test
test:
	@mkdir -p $(OUT_DIR)
	@pass=0; fail=0; \
	for bs in 0 1 2 3 4 5 6 7 8 9; do \
		iverilog -g2005 -P soft_pcs_wrapper_tb.BITSHIFT=$$bs -o $(OUT_DIR)/tb_$$bs.vvp $(SRCS); \
		res=$$(vvp $(OUT_DIR)/tb_$$bs.vvp | grep -E "SUCCESS|FAILED"); \
		echo "bitshift=$$bs : $$res"; \
		echo "$$res" | grep -q SUCCESS && pass=$$((pass+1)) || fail=$$((fail+1)); \
	done; \
	echo "==== PASS=$$pass FAIL=$$fail ===="; \
	[ "$$fail" -eq 0 ]

# Clean generated files
.PHONY: clean
clean:
	rm -rf $(OUT_DIR)/*.vvp $(OUT_DIR)/*.vcd

# Help
.PHONY: help
help:
	@echo "JESD204 Soft PCS Wrapper - Standalone Project"
	@echo ""
	@echo "Targets:"
	@echo "  all       - Run simulation (default)"
	@echo "  compile   - Compile all sources"
	@echo "  simulate  - Run simulation with default bitshift (0)"
	@echo "  test      - Run full sweep test (bitshift 0-9)"
	@echo "  clean     - Clean generated files"
	@echo "  help      - Show this help"
	@echo ""
	@echo "Directory structure:"
	@echo "  code/     - All Verilog source files"
	@echo "  script/   - Build scripts and file lists"
	@echo "  sim/      - Testbench files"