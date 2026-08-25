####################################################################################
## Copyright (c) 2026 Analog Devices, Inc.
### SPDX short identifier: BSD-1-Clause
####################################################################################

LIBRARY_NAME := jesd204_soft_pcs_wrapper

GENERIC_DEPS += jesd204_soft_pcs_fifo.v
GENERIC_DEPS += jesd204_soft_pcs_wrapper.v

GENERIC_DEPS += ../jesd204_soft_pcs_tx/jesd204_8b10b_encoder.v
GENERIC_DEPS += ../jesd204_soft_pcs_tx/jesd204_soft_pcs_tx.v
GENERIC_DEPS += ../jesd204_soft_pcs_rx/jesd204_8b10b_decoder.v
GENERIC_DEPS += ../jesd204_soft_pcs_rx/jesd204_pattern_align.v
GENERIC_DEPS += ../jesd204_soft_pcs_rx/jesd204_soft_pcs_rx.v

include ../../scripts/library.mk
