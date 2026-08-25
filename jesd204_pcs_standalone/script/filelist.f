// JESD204 PCS with Link Training - Source File List
// All paths relative to the 'code' directory

// Top-level wrapper with link training
jesd204_pcs_link_training.v

// Original soft PCS wrapper (for reference)
jesd204_soft_pcs_wrapper.v
jesd204_soft_pcs_fifo.v

// TX modules
jesd204_soft_pcs_tx.v
jesd204_8b10b_encoder.v
jesd204_tx_ctrl.v
jesd204_tx_lane.v

// RX modules
jesd204_soft_pcs_rx.v
jesd204_8b10b_decoder.v
jesd204_pattern_align.v
jesd204_rx_cgs.v
jesd204_ilas_monitor.v
jesd204_rx_ctrl.v
jesd204_rx_lane.v
jesd204_rx_frame_align.v
align_mux.v
elastic_buffer.v

// Common modules
jesd204_scrambler.v
jesd204_lmfc.v
jesd204_frame_mark.v
jesd204_frame_align_replace.v

// Utility modules
util_pipeline_stage.v
sync_bits.v
sync_event.v