// JESD204 PCS with Link Training - Source File List
// Paths relative to the project root directory

// Top-level wrapper with link training
code/jesd204_pcs_link_training.v

// Original soft PCS wrapper (for reference)
code/jesd204_soft_pcs_wrapper.v
code/jesd204_soft_pcs_fifo.v

// TX modules
code/jesd204_soft_pcs_tx.v
code/jesd204_8b10b_encoder.v
code/jesd204_tx_ctrl.v
code/jesd204_tx_lane.v

// RX modules
code/jesd204_soft_pcs_rx.v
code/jesd204_8b10b_decoder.v
code/jesd204_pattern_align.v
code/jesd204_rx_cgs.v
code/jesd204_ilas_monitor.v
code/jesd204_rx_ctrl.v
code/jesd204_rx_lane.v
code/jesd204_rx_frame_align.v
code/align_mux.v
code/elastic_buffer.v

// Common modules
code/jesd204_scrambler.v
code/jesd204_lmfc.v
code/jesd204_frame_mark.v
code/jesd204_frame_align_replace.v

// Utility modules
code/util_pipeline_stage.v
code/sync_bits.v
code/sync_event.v