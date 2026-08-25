// ***************************************************************************
// ***************************************************************************
// Copyright (C) 2026 Analog Devices, Inc. All rights reserved.
// SPDX short identifier: ADIJESD204
// ***************************************************************************
// ***************************************************************************

`timescale 1ns/100ps

module soft_pcs_top_tb;
  parameter VCD_FILE = "soft_pcs_top_tb.vcd";
  parameter NUM_LANES = 2;
  parameter DATA_PATH_WIDTH = 4;

  `include "tb_base.v"

  reg reset = 1'b1;

  reg tx_valid = 1'b0;
  wire tx_ready;
  reg [NUM_LANES*DATA_PATH_WIDTH*8-1:0] tx_data = 'h0;
  reg [NUM_LANES*DATA_PATH_WIDTH-1:0] tx_charisk = 'h0;

  wire [NUM_LANES*DATA_PATH_WIDTH*10-1:0] phy_tx_data;
  reg [NUM_LANES*DATA_PATH_WIDTH*10-1:0] phy_rx_data = 'h0;

  reg rx_pattern_align_en = 1'b1;
  wire rx_valid;
  reg rx_ready = 1'b1;
  wire [NUM_LANES*DATA_PATH_WIDTH*8-1:0] rx_data;
  wire [NUM_LANES*DATA_PATH_WIDTH-1:0] rx_charisk;
  wire [NUM_LANES*DATA_PATH_WIDTH-1:0] rx_notintable;
  wire [NUM_LANES*DATA_PATH_WIDTH-1:0] rx_disperr;

  // Unaligned pipeline for bitshift simulation
  wire [NUM_LANES*DATA_PATH_WIDTH*10+9:0] phy_tx_data_full[0:NUM_LANES-1];
  reg [8:0] phy_tx_data_d1[0:NUM_LANES-1];
  reg [3:0] bitshift = 4'h0;

  genvar l;
  generate
    for (l = 0; l < NUM_LANES; l = l + 1) begin: gen_lane_delay
      always @(posedge clk) begin
        phy_tx_data_d1[l] <= phy_tx_data[l*DATA_PATH_WIDTH*10 + DATA_PATH_WIDTH*10-1 : l*DATA_PATH_WIDTH*10 + DATA_PATH_WIDTH*10-9];
      end

      assign phy_tx_data_full[l] = {phy_tx_data[l*DATA_PATH_WIDTH*10+:DATA_PATH_WIDTH*10], phy_tx_data_d1[l]};

      always @(*) begin
        phy_rx_data[l*DATA_PATH_WIDTH*10+:DATA_PATH_WIDTH*10] <= phy_tx_data_full[l][bitshift+:DATA_PATH_WIDTH*10];
      end
    end
  endgenerate

  jesd204_soft_pcs_top #(
    .NUM_LANES(NUM_LANES),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH)
  ) i_soft_pcs_top (
    .clk(clk),
    .reset(reset),

    .tx_valid(tx_valid),
    .tx_ready(tx_ready),
    .tx_data(tx_data),
    .tx_charisk(tx_charisk),

    .phy_tx_data(phy_tx_data),
    .phy_rx_data(phy_rx_data),

    .rx_pattern_align_en(rx_pattern_align_en),
    .rx_valid(rx_valid),
    .rx_ready(rx_ready),
    .rx_data(rx_data),
    .rx_charisk(rx_charisk),
    .rx_notintable(rx_notintable),
    .rx_disperr(rx_disperr)
  );

  integer test_counter = 0;
  reg [31:0] test_payload = 32'h12345678;

  initial begin
    #100;
    reset = 1'b0;
  end

  always @(posedge clk) begin
    if (!reset) begin
      test_counter <= test_counter + 1;

      // Enable pattern alignment during first phase
      if (test_counter < 100) begin
        rx_pattern_align_en <= 1'b1;
        tx_valid <= 1'b0; // Send K28.5 idle
      end else begin
        rx_pattern_align_en <= 1'b0;
        tx_valid <= 1'b1;
        tx_charisk <= {NUM_LANES*DATA_PATH_WIDTH{1'b0}};

        test_payload <= test_payload + 1'b1;
        tx_data <= {(NUM_LANES*DATA_PATH_WIDTH/4){test_payload}};
      end

      if (test_counter == 300) begin
        $finish;
      end
    end
  end

endmodule
