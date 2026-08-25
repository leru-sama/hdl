// ***************************************************************************
// ***************************************************************************
// Copyright (C) 2026 Analog Devices, Inc. All rights reserved.
// SPDX short identifier: ADIJESD204
// ***************************************************************************
// ***************************************************************************

`timescale 1ns/100ps

module jesd204_soft_pcs_top #(
  parameter NUM_LANES = 1,
  parameter DATA_PATH_WIDTH = 4,
  parameter INVERT_INPUTS = 0,
  parameter INVERT_OUTPUTS = 0,
  parameter IFC_TYPE = 0
) (
  input clk,
  input reset,

  // Upper layer Transmit (TX) Interface with Valid/Ready
  input tx_valid,
  output tx_ready,
  input [NUM_LANES*DATA_PATH_WIDTH*8-1:0] tx_data,
  input [NUM_LANES*DATA_PATH_WIDTH-1:0] tx_charisk,

  // SerDes Physical (PHY) Parallel Data Interface
  output [NUM_LANES*(DATA_PATH_WIDTH*10 + IFC_TYPE*40)-1:0] phy_tx_data,
  input [NUM_LANES*(DATA_PATH_WIDTH*10 + IFC_TYPE*40)-1:0] phy_rx_data,

  // Upper layer Receive (RX) Interface with Valid/Ready
  input rx_pattern_align_en,
  output reg rx_valid,
  input rx_ready,
  output reg [NUM_LANES*DATA_PATH_WIDTH*8-1:0] rx_data,
  output reg [NUM_LANES*DATA_PATH_WIDTH-1:0] rx_charisk,
  output reg [NUM_LANES*DATA_PATH_WIDTH-1:0] rx_notintable,
  output reg [NUM_LANES*DATA_PATH_WIDTH-1:0] rx_disperr
);

  localparam TOTAL_BYTES = NUM_LANES * DATA_PATH_WIDTH;
  localparam [TOTAL_BYTES*8-1:0] K28_5_CHAR = {(TOTAL_BYTES){8'hBC}}; // K28.5 (comma)
  localparam [TOTAL_BYTES-1:0]   K28_5_ISK  = {(TOTAL_BYTES){1'b1}};

  // TX Logic
  // TX pipeline is always capable of receiving data when not in reset
  assign tx_ready = ~reset;

  wire [NUM_LANES*DATA_PATH_WIDTH*8-1:0] tx_char_s;
  wire [NUM_LANES*DATA_PATH_WIDTH-1:0]   tx_charisk_s;

  // Send idle K28.5 character when tx_valid is not active
  assign tx_char_s   = (tx_valid && tx_ready) ? tx_data : K28_5_CHAR;
  assign tx_charisk_s = (tx_valid && tx_ready) ? tx_charisk : K28_5_ISK;

  jesd204_soft_pcs_tx #(
    .NUM_LANES(NUM_LANES),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .INVERT_OUTPUTS(INVERT_OUTPUTS),
    .IFC_TYPE(IFC_TYPE)
  ) i_soft_pcs_tx (
    .clk(clk),
    .reset(reset),
    .char(tx_char_s),
    .charisk(tx_charisk_s),
    .data(phy_tx_data)
  );

  // RX Logic
  wire [NUM_LANES*DATA_PATH_WIDTH*8-1:0] rx_char_s;
  wire [NUM_LANES*DATA_PATH_WIDTH-1:0]   rx_charisk_s;
  wire [NUM_LANES*DATA_PATH_WIDTH-1:0]   rx_notintable_s;
  wire [NUM_LANES*DATA_PATH_WIDTH-1:0]   rx_disperr_s;

  jesd204_soft_pcs_rx #(
    .NUM_LANES(NUM_LANES),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .REGISTER_INPUTS(1),
    .INVERT_INPUTS(INVERT_INPUTS),
    .IFC_TYPE(IFC_TYPE)
  ) i_soft_pcs_rx (
    .clk(clk),
    .reset(reset),
    .patternalign_en(rx_pattern_align_en),
    .data(phy_rx_data),
    .char(rx_char_s),
    .charisk(rx_charisk_s),
    .notintable(rx_notintable_s),
    .disperr(rx_disperr_s)
  );

  // Buffer decoded RX data to upper layer
  always @(posedge clk) begin
    if (reset) begin
      rx_valid <= 1'b0;
      rx_data <= 'h0;
      rx_charisk <= 'h0;
      rx_notintable <= 'h0;
      rx_disperr <= 'h0;
    end else begin
      rx_data <= rx_char_s;
      rx_charisk <= rx_charisk_s;
      rx_notintable <= rx_notintable_s;
      rx_disperr <= rx_disperr_s;
      // Valid when no decoder errors and pattern alignment phase is completed
      rx_valid <= (rx_notintable_s == 'b0) && (rx_disperr_s == 'b0) && (!rx_pattern_align_en);
    end
  end

endmodule
