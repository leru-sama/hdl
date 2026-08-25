#!/usr/bin/env bash
# ***************************************************************************
# Self-checking iverilog simulation for jesd204_soft_pcs_wrapper.
# Sweeps the modeled serdes bit-slip (0..9) through the loopback testbench.
# ***************************************************************************
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CODE_DIR="$PROJECT_DIR/code"
SIM_DIR="$PROJECT_DIR/sim"
OUT="${OUT:-/tmp/logs}"
mkdir -p "$OUT"

# Source file list
SRCS="\
  $SIM_DIR/soft_pcs_wrapper_tb.v \
  $CODE_DIR/jesd204_soft_pcs_wrapper.v \
  $CODE_DIR/jesd204_soft_pcs_fifo.v \
  $CODE_DIR/jesd204_soft_pcs_tx.v \
  $CODE_DIR/jesd204_8b10b_encoder.v \
  $CODE_DIR/jesd204_soft_pcs_rx.v \
  $CODE_DIR/jesd204_8b10b_decoder.v \
  $CODE_DIR/jesd204_pattern_align.v"

pass=0; fail=0
for bs in 0 1 2 3 4 5 6 7 8 9; do
  iverilog -g2005 -P soft_pcs_wrapper_tb.BITSHIFT=$bs -o "$OUT/tb_$bs.vvp" $SRCS
  res=$(vvp "$OUT/tb_$bs.vvp" | grep -E "SUCCESS|FAILED")
  echo "bitshift=$bs : $res"
  echo "$res" | grep -q SUCCESS && pass=$((pass+1)) || fail=$((fail+1))
done
echo "==== PASS=$pass FAIL=$fail ===="
[ "$fail" -eq 0 ]