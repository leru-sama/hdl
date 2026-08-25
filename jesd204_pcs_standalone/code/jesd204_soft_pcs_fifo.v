// ***************************************************************************
// ***************************************************************************
// Copyright (C) 2026 Analog Devices, Inc. All rights reserved.
// SPDX short identifier: ADIJESD204
// ***************************************************************************
// ***************************************************************************

`timescale 1ns/100ps

// Small synchronous first-word-fall-through (FWFT) elastic buffer used by the
// soft PCS wrapper to bridge the free-running serdes datapath to a ready/valid
// application interface.
//
//   * din is captured on wr & ~full
//   * dout is valid whenever ~empty (FWFT, no read latency)
//   * dout is popped on rd & ~empty
//   * overflow pulses when a write is attempted while full (data is dropped)

module jesd204_soft_pcs_fifo #(
  parameter DATA_WIDTH = 8,
  parameter ADDR_WIDTH = 4
) (
  input clk,
  input reset,

  input                   wr,
  input  [DATA_WIDTH-1:0] din,
  output                  full,
  output                  overflow,

  input                   rd,
  output [DATA_WIDTH-1:0] dout,
  output                  empty
);

  localparam DEPTH = (1 << ADDR_WIDTH);

  reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
  reg [ADDR_WIDTH:0]   wptr = {(ADDR_WIDTH+1){1'b0}};
  reg [ADDR_WIDTH:0]   rptr = {(ADDR_WIDTH+1){1'b0}};

  assign empty = (wptr == rptr);
  assign full  = (wptr[ADDR_WIDTH] != rptr[ADDR_WIDTH]) &&
                 (wptr[ADDR_WIDTH-1:0] == rptr[ADDR_WIDTH-1:0]);
  assign overflow = wr & full;

  always @(posedge clk) begin
    if (reset == 1'b1) begin
      wptr <= {(ADDR_WIDTH+1){1'b0}};
    end else if (wr == 1'b1 && full == 1'b0) begin
      mem[wptr[ADDR_WIDTH-1:0]] <= din;
      wptr <= wptr + 1'b1;
    end
  end

  always @(posedge clk) begin
    if (reset == 1'b1) begin
      rptr <= {(ADDR_WIDTH+1){1'b0}};
    end else if (rd == 1'b1 && empty == 1'b0) begin
      rptr <= rptr + 1'b1;
    end
  end

  assign dout = mem[rptr[ADDR_WIDTH-1:0]];

endmodule
