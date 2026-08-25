#!/usr/bin/env bash
# ***************************************************************************
# JESD204 PCS Link Training Simulation Script
# ***************************************************************************
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CODE_DIR="$PROJECT_DIR/code"
SIM_DIR="$PROJECT_DIR/sim"
OUT="${OUT:-/tmp/logs}"
mkdir -p "$OUT"

# Source file list for link training version
SRCS="\
  $SIM_DIR/jesd204_pcs_link_training_tb.v \
  $CODE_DIR/jesd204_pcs_link_training.v \
  $CODE_DIR/jesd204_tx_ctrl.v \
  $CODE_DIR/jesd204_tx_lane.v \
  $CODE_DIR/jesd204_rx_cgs.v \
  $CODE_DIR/jesd204_ilas_monitor.v \
  $CODE_DIR/jesd204_rx_ctrl.v \
  $CODE_DIR/jesd204_rx_lane.v \
  $CODE_DIR/jesd204_rx_frame_align.v \
  $CODE_DIR/jesd204_8b10b_encoder.v \
  $CODE_DIR/jesd204_8b10b_decoder.v \
  $CODE_DIR/jesd204_pattern_align.v \
  $CODE_DIR/jesd204_scrambler.v \
  $CODE_DIR/jesd204_lmfc.v \
  $CODE_DIR/jesd204_frame_mark.v \
  $CODE_DIR/jesd204_frame_align_replace.v \
  $CODE_DIR/align_mux.v \
  $CODE_DIR/elastic_buffer.v \
  $CODE_DIR/util_pipeline_stage.v \
  $CODE_DIR/sync_bits.v \
  $CODE_DIR/sync_event.v"

echo "=== JESD204 PCS Link Training - 2-DUT Simulation ==="
echo "Compiling..."
iverilog -g2005 -o "$OUT/pcs_link_training.vvp" $SRCS

echo "Running single simulation (default skew seed)..."
vvp "$OUT/pcs_link_training.vvp"

# Optional: sweep random per-lane skew patterns with SWEEP=1
if [ "${SWEEP:-0}" = "1" ]; then
  echo ""
  echo "=== Skew sweep (20 random seeds) ==="
  pass=0; fail=0
  for sd in $(seq 1 20); do
    iverilog -g2005 -P jesd204_pcs_link_training_tb.SEED=$sd \
      -o "$OUT/lt_$sd.vvp" $SRCS 2>/dev/null
    res=$(vvp "$OUT/lt_$sd.vvp" 2>/dev/null | grep -E "SUCCESS|FAILED")
    echo "seed=$sd : $res"
    echo "$res" | grep -q SUCCESS && pass=$((pass+1)) || fail=$((fail+1))
  done
  echo "==== PASS=$pass FAIL=$fail ===="
fi

echo "=== Simulation Complete ==="