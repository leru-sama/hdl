// JESD204 Soft PCS Wrapper - Source File List
// All paths relative to the 'code' directory

// Wrapper modules
jesd204_soft_pcs_wrapper.v
jesd204_soft_pcs_fifo.v

// TX modules (8b10b encoder)
jesd204_soft_pcs_tx.v
jesd204_8b10b_encoder.v

// RX modules (pattern align + 8b10b decoder)
jesd204_soft_pcs_rx.v
jesd204_pattern_align.v
jesd204_8b10b_decoder.v