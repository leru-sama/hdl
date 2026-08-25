####################################################################################
# JESD204 PCS with Link Training - Standalone Project
####################################################################################

# Project directories
CODE_DIR := code
SCRIPT_DIR := script
SIM_DIR := sim

# Output directory
OUT_DIR := /tmp/logs

# Source files for link training version
SRCS_LINK_TRAINING := $(SIM_DIR)/jesd204_pcs_link_training_tb.v \
        $(CODE_DIR)/jesd204_pcs_link_training.v \
        $(CODE_DIR)/jesd204_tx_ctrl.v \
        $(CODE_DIR)/jesd204_tx_lane.v \
        $(CODE_DIR)/jesd204_rx_cgs.v \
        $(CODE_DIR)/jesd204_ilas_monitor.v \
        $(CODE_DIR)/jesd204_rx_ctrl.v \
        $(CODE_DIR)/jesd204_rx_lane.v \
        $(CODE_DIR)/jesd204_rx_frame_align.v \
        $(CODE_DIR)/jesd204_8b10b_encoder.v \
        $(CODE_DIR)/jesd204_8b10b_decoder.v \
        $(CODE_DIR)/jesd204_pattern_align.v \
        $(CODE_DIR)/jesd204_scrambler.v \
        $(CODE_DIR)/jesd204_lmfc.v \
        $(CODE_DIR)/jesd204_frame_mark.v \
        $(CODE_DIR)/jesd204_frame_align_replace.v \
        $(CODE_DIR)/align_mux.v \
        $(CODE_DIR)/elastic_buffer.v \
        $(CODE_DIR)/util_pipeline_stage.v \
        $(CODE_DIR)/sync_bits.v \
        $(CODE_DIR)/sync_event.v

# Source files for simple wrapper version
SRCS_SIMPLE := $(SIM_DIR)/soft_pcs_wrapper_tb.v \
        $(CODE_DIR)/jesd204_soft_pcs_wrapper.v \
        $(CODE_DIR)/jesd204_soft_pcs_fifo.v \
        $(CODE_DIR)/jesd204_soft_pcs_tx.v \
        $(CODE_DIR)/jesd204_8b10b_encoder.v \
        $(CODE_DIR)/jesd204_soft_pcs_rx.v \
        $(CODE_DIR)/jesd204_8b10b_decoder.v \
        $(CODE_DIR)/jesd204_pattern_align.v

# Default target
.PHONY: all
all: test_link_training

# Compile link training version
.PHONY: compile_link_training
compile_link_training:
	@mkdir -p $(OUT_DIR)
	iverilog -g2005 -o $(OUT_DIR)/pcs_link_training.vvp $(SRCS_LINK_TRAINING)

# Run link training simulation
.PHONY: simulate_link_training
simulate_link_training: compile_link_training
	vvp $(OUT_DIR)/pcs_link_training.vvp

# Compile simple wrapper version
.PHONY: compile_simple
compile_simple:
	@mkdir -p $(OUT_DIR)
	iverilog -g2005 -o $(OUT_DIR)/pcs_simple.vvp $(SRCS_SIMPLE)

# Run simple wrapper simulation
.PHONY: simulate_simple
simulate_simple: compile_simple
	vvp $(OUT_DIR)/pcs_simple.vvp

# Run simple wrapper sweep test (bitshift 0-9)
.PHONY: test_simple
test_simple:
	@mkdir -p $(OUT_DIR)
	@pass=0; fail=0; \
	for bs in 0 1 2 3 4 5 6 7 8 9; do \
		iverilog -g2005 -P soft_pcs_wrapper_tb.BITSHIFT=$$bs -o $(OUT_DIR)/tb_$$bs.vvp $(SRCS_SIMPLE); \
		res=$$(vvp $(OUT_DIR)/tb_$$bs.vvp | grep -E "SUCCESS|FAILED"); \
		echo "bitshift=$$bs : $$res"; \
		echo "$$res" | grep -q SUCCESS && pass=$$((pass+1)) || fail=$$((fail+1)); \
	done; \
	echo "==== PASS=$$pass FAIL=$$fail ===="; \
	[ "$$fail" -eq 0 ]

# Run link training test
.PHONY: test_link_training
test_link_training: simulate_link_training

# Sweep many random per-lane skew patterns (2-DUT test) to prove robustness
.PHONY: test_link_training_sweep
test_link_training_sweep:
	@mkdir -p $(OUT_DIR)
	@pass=0; fail=0; \
	for sd in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do \
		iverilog -g2005 -P jesd204_pcs_link_training_tb.SEED=$$sd \
			-o $(OUT_DIR)/lt_$$sd.vvp $(SRCS_LINK_TRAINING) 2>/dev/null; \
		out=$$(vvp $(OUT_DIR)/lt_$$sd.vvp 2>/dev/null); \
		res=$$(echo "$$out" | grep -E "SUCCESS|FAILED"); \
		skew=$$(echo "$$out" | grep "Lane skews" | sed 's/.*: //'); \
		echo "seed=$$sd  skews=[$$skew] : $$res"; \
		echo "$$res" | grep -q SUCCESS && pass=$$((pass+1)) || fail=$$((fail+1)); \
	done; \
	echo "==== PASS=$$pass FAIL=$$fail ===="; \
	[ "$$fail" -eq 0 ]

# Run all tests
.PHONY: test
test: test_link_training test_simple

# Clean generated files
.PHONY: clean
clean:
	rm -rf $(OUT_DIR)/*.vvp $(OUT_DIR)/*.vcd

# Help
.PHONY: help
help:
	@echo "JESD204 PCS with Link Training - Standalone Project"
	@echo ""
	@echo "Targets:"
	@echo "  all                    - Run link training test (default)"
	@echo "  compile_link_training  - Compile link training version"
	@echo "  simulate_link_training - Run link training simulation"
	@echo "  compile_simple         - Compile simple wrapper version"
	@echo "  simulate_simple        - Run simple wrapper simulation"
	@echo "  test_simple            - Run simple wrapper sweep test"
	@echo "  test_link_training     - Run link training test"
	@echo "  test                   - Run all tests"
	@echo "  clean                  - Clean generated files"
	@echo "  help                   - Show this help"
	@echo ""
	@echo "Directory structure:"
	@echo "  code/     - All Verilog source files"
	@echo "  script/   - Build scripts and file lists"
	@echo "  sim/      - Testbench files"