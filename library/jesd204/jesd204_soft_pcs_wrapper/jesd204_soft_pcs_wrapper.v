// ***************************************************************************
// ***************************************************************************
// Copyright (C) 2026 Analog Devices, Inc. All rights reserved.
// SPDX short identifier: ADIJESD204
// ***************************************************************************
// ***************************************************************************

`timescale 1ns/100ps

// -----------------------------------------------------------------------------
// jesd204_soft_pcs_wrapper
//
// Top-level wrapper around the JESD204 soft PCS (jesd204_soft_pcs_tx /
// jesd204_soft_pcs_rx) intended for use with a "transparent" (data pass-through)
// serdes that presents DATA_PATH_WIDTH*10 raw bits per lane in parallel each
// PCS clock.  For a serdes that exposes 20 bits per lane use the default
// DATA_PATH_WIDTH = 2 (2 x 10b 8b10b symbols = 20 bits per lane).
//
// The serdes-facing side runs free (one parallel beat every clock).  The
// application-facing side speaks a standard ready/valid handshake:
//
//   TX (application -> serdes):
//     * tx_ready is asserted whenever the encoder can accept a beat (always,
//       once out of reset, because the line must be fed every cycle).
//     * When the application presents a beat (tx_valid & tx_ready) the supplied
//       characters are 8b10b encoded and driven onto tx_serdes_data.
//     * When the application has no data, an idle /K28.5/ comma is inserted so
//       the serdes always carries a valid, DC-balanced symbol stream.
//
//   RX (serdes -> application):
//     * Incoming raw symbols are pattern aligned and 8b10b decoded.
//     * Decoded beats are pushed into a small elastic FIFO and presented to the
//       application through rx_valid / rx_ready.  Idle commas are optionally
//       filtered (RX_FILTER_IDLE) so the application only sees payload.
//     * rx_overflow pulses if the application back-pressures long enough to
//       overflow the elastic buffer (data is then dropped).
//
// TX and RX share a single PCS/serdes parallel clock domain (clk).
// -----------------------------------------------------------------------------

module jesd204_soft_pcs_wrapper #(
  parameter NUM_LANES       = 1,   // number of serdes lanes
  parameter DATA_PATH_WIDTH = 2,   // 8b10b symbols per lane per clock (2 => 20b/lane)
  parameter IFC_TYPE        = 0,   // 0: raw 10b*DPW, 1: Intel F-Tile 40b padded framing
  parameter INVERT_LANES    = 0,   // invert serdes bits (polarity swap)
  parameter REGISTER_INPUTS = 1,   // pipeline the RX serdes inputs
  parameter RX_FILTER_IDLE  = 1,   // drop idle /K28.5/ beats before the RX FIFO
  parameter RX_FIFO_ADDR_W  = 4    // RX elastic buffer depth = 2**RX_FIFO_ADDR_W
) (
  input clk,
  input reset,

  // ----------------------------------------------------------------
  // Application TX interface (ready/valid) -> serdes
  // ----------------------------------------------------------------
  input                                     tx_valid,
  output                                    tx_ready,
  input  [NUM_LANES*DATA_PATH_WIDTH*8-1:0]  tx_char,
  input  [NUM_LANES*DATA_PATH_WIDTH-1:0]    tx_charisk,

  // Raw parallel data to the transparent serdes (DATA_PATH_WIDTH*10 b / lane)
  output [NUM_LANES*(DATA_PATH_WIDTH*10 + IFC_TYPE*40)-1:0] tx_serdes_data,

  // ----------------------------------------------------------------
  // serdes -> Application RX interface (ready/valid)
  // ----------------------------------------------------------------
  input  [NUM_LANES*(DATA_PATH_WIDTH*10 + IFC_TYPE*40)-1:0] rx_serdes_data,
  input                                     rx_patternalign_en,
  input                                     rx_enable,

  output                                    rx_valid,
  input                                     rx_ready,
  output [NUM_LANES*DATA_PATH_WIDTH*8-1:0]  rx_char,
  output [NUM_LANES*DATA_PATH_WIDTH-1:0]    rx_charisk,
  output [NUM_LANES*DATA_PATH_WIDTH-1:0]    rx_notintable,
  output [NUM_LANES*DATA_PATH_WIDTH-1:0]    rx_disperr,
  output                                    rx_overflow
);

  // Total number of 8b10b symbols across all lanes per beat
  localparam P       = NUM_LANES * DATA_PATH_WIDTH;
  localparam CHAR_W  = P * 8;
  localparam CTRL_W  = P;
  // FIFO element: {char, charisk, notintable, disperr}
  localparam FIFO_W  = CHAR_W + 3*CTRL_W;

  // K28.5 comma, the JESD204 alignment / idle character
  localparam [7:0] COMMA = 8'hBC;

  //---------------------------------------------------------------------------
  // TX datapath : application -> 8b10b encode -> serdes
  //---------------------------------------------------------------------------
  wire [CHAR_W-1:0] enc_char;
  wire [CTRL_W-1:0] enc_charisk;
  wire              tx_accept;

  // The encoder must be fed every cycle, so we are always ready out of reset.
  assign tx_ready  = ~reset;
  assign tx_accept = tx_valid & tx_ready;

  // Replicate the idle comma across every symbol position.
  wire [CHAR_W-1:0] comma_char_bus  = {P{COMMA}};
  wire [CTRL_W-1:0] comma_charisk   = {P{1'b1}};

  // Feed application data when accepted, otherwise insert an idle comma.
  assign enc_char    = tx_accept ? tx_char    : comma_char_bus;
  assign enc_charisk = tx_accept ? tx_charisk : comma_charisk;

  jesd204_soft_pcs_tx #(
    .NUM_LANES(NUM_LANES),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .INVERT_OUTPUTS(INVERT_LANES),
    .IFC_TYPE(IFC_TYPE)
  ) i_soft_pcs_tx (
    .clk(clk),
    .reset(reset),
    .char(enc_char),
    .charisk(enc_charisk),
    .data(tx_serdes_data)
  );

  //---------------------------------------------------------------------------
  // RX datapath : serdes -> pattern align + 8b10b decode -> elastic FIFO
  //---------------------------------------------------------------------------
  wire [CHAR_W-1:0] dec_char;
  wire [CTRL_W-1:0] dec_charisk;
  wire [CTRL_W-1:0] dec_notintable;
  wire [CTRL_W-1:0] dec_disperr;

  jesd204_soft_pcs_rx #(
    .NUM_LANES(NUM_LANES),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .REGISTER_INPUTS(REGISTER_INPUTS),
    .INVERT_INPUTS(INVERT_LANES),
    .IFC_TYPE(IFC_TYPE)
  ) i_soft_pcs_rx (
    .clk(clk),
    .reset(reset),
    .patternalign_en(rx_patternalign_en),
    .data(rx_serdes_data),
    .char(dec_char),
    .charisk(dec_charisk),
    .notintable(dec_notintable),
    .disperr(dec_disperr)
  );

  // A beat is "idle" when every symbol position carries the /K28.5/ comma.
  wire beat_is_idle = (dec_charisk == {P{1'b1}}) &&
                      (dec_char == {P{COMMA}});

  wire rx_push = rx_enable & ~reset &
                 ~(RX_FILTER_IDLE[0] & beat_is_idle);

  wire              fifo_empty;
  wire [FIFO_W-1:0] fifo_dout;

  jesd204_soft_pcs_fifo #(
    .DATA_WIDTH(FIFO_W),
    .ADDR_WIDTH(RX_FIFO_ADDR_W)
  ) i_rx_fifo (
    .clk(clk),
    .reset(reset),
    .wr(rx_push),
    .din({dec_char, dec_charisk, dec_notintable, dec_disperr}),
    .full(),
    .overflow(rx_overflow),
    .rd(rx_valid & rx_ready),
    .dout(fifo_dout),
    .empty(fifo_empty)
  );

  assign rx_valid = ~fifo_empty;
  assign {rx_char, rx_charisk, rx_notintable, rx_disperr} = fifo_dout;

endmodule
