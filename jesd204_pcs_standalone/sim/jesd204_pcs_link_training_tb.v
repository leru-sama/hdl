// ***************************************************************************
// JESD204 PCS Link Training Testbench
// Verifies CGS -> ILAS -> DATA with 4-lane random skew (0-16 cycles)
// ***************************************************************************
`timescale 1ns/100ps

module jesd204_pcs_link_training_tb;

  parameter NUM_LANES = 4;
  parameter NUM_LINKS = 1;
  parameter DATA_PATH_WIDTH = 2;
  parameter CLK_PERIOD = 10;
  parameter MAX_SKEW = 16;

  reg clk;
  reg reset;

  reg [9:0] cfg_octets_per_multiframe;
  reg [7:0] cfg_octets_per_frame;
  reg [NUM_LANES-1:0] cfg_lanes_disable;
  reg [NUM_LINKS-1:0] cfg_links_disable;
  reg cfg_disable_scrambler;
  reg cfg_disable_char_replacement;
  reg [7:0] cfg_mframes_per_ilas;
  reg cfg_skip_ilas;
  reg cfg_continuous_cgs;
  reg cfg_continuous_ilas;

  reg tx_valid;
  wire tx_ready;
  reg [DATA_PATH_WIDTH*8*NUM_LANES-1:0] tx_data;
  reg [DATA_PATH_WIDTH*NUM_LANES-1:0] tx_charisk;

  wire rx_valid;
  reg rx_ready;
  wire [DATA_PATH_WIDTH*8*NUM_LANES-1:0] rx_data;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_charisk;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_notintable;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_disperr;

  wire [DATA_PATH_WIDTH*10*NUM_LANES-1:0] serdes_tx_data;
  wire [DATA_PATH_WIDTH*10*NUM_LANES-1:0] serdes_rx_data;

  wire [1:0] status_ctrl_state;
  wire [2*NUM_LANES-1:0] status_lane_cgs_state;
  wire [NUM_LANES-1:0] status_lane_ifs_ready;
  wire [NUM_LINKS-1:0] sync_n;
  wire [NUM_LANES-1:0] ilas_config_valid;
  wire [NUM_LANES*2-1:0] ilas_config_addr;

  integer tx_count, rx_count, error_count;
  integer lane0_delay, lane1_delay, lane2_delay, lane3_delay;

  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // ---------------------------------------------------------------
  // Per-lane skew injection (0-16 cycles each)
  // ---------------------------------------------------------------
  reg [19:0] lane0_pipe [0:MAX_SKEW];
  reg [19:0] lane1_pipe [0:MAX_SKEW];
  reg [19:0] lane2_pipe [0:MAX_SKEW];
  reg [19:0] lane3_pipe [0:MAX_SKEW];
  integer i;

  initial begin
    lane0_delay = 0;
    lane1_delay = $urandom_range(0, MAX_SKEW);
    lane2_delay = $urandom_range(0, MAX_SKEW);
    lane3_delay = $urandom_range(0, MAX_SKEW);
    $display("Lane delays: %0d, %0d, %0d, %0d", lane0_delay, lane1_delay, lane2_delay, lane3_delay);
  end

  always @(posedge clk) begin
    lane0_pipe[0] <= serdes_tx_data[19:0];
    lane1_pipe[0] <= serdes_tx_data[39:20];
    lane2_pipe[0] <= serdes_tx_data[59:40];
    lane3_pipe[0] <= serdes_tx_data[79:60];
    for (i = 1; i <= MAX_SKEW; i = i + 1) begin
      lane0_pipe[i] <= lane0_pipe[i-1];
      lane1_pipe[i] <= lane1_pipe[i-1];
      lane2_pipe[i] <= lane2_pipe[i-1];
      lane3_pipe[i] <= lane3_pipe[i-1];
    end
  end

  assign serdes_rx_data[19:0]  = lane0_pipe[lane0_delay];
  assign serdes_rx_data[39:20] = lane1_pipe[lane1_delay];
  assign serdes_rx_data[59:40] = lane2_pipe[lane2_delay];
  assign serdes_rx_data[79:60] = lane3_pipe[lane3_delay];

  // ---------------------------------------------------------------
  // DUT
  // ---------------------------------------------------------------
  jesd204_pcs_link_training #(
    .NUM_LANES(NUM_LANES),
    .NUM_LINKS(NUM_LINKS),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .ENABLE_FRAME_ALIGN_CHECK(0),
    .ENABLE_CHAR_REPLACE(0),
    .ELASTIC_BUFFER_SIZE(256)
  ) dut (
    .clk(clk),
    .reset(reset),
    .cfg_octets_per_multiframe(cfg_octets_per_multiframe),
    .cfg_octets_per_frame(cfg_octets_per_frame),
    .cfg_lanes_disable(cfg_lanes_disable),
    .cfg_links_disable(cfg_links_disable),
    .cfg_disable_scrambler(cfg_disable_scrambler),
    .cfg_disable_char_replacement(cfg_disable_char_replacement),
    .cfg_mframes_per_ilas(cfg_mframes_per_ilas),
    .cfg_skip_ilas(cfg_skip_ilas),
    .cfg_continuous_cgs(cfg_continuous_cgs),
    .cfg_continuous_ilas(cfg_continuous_ilas),
    .tx_valid(tx_valid),
    .tx_ready(tx_ready),
    .tx_data(tx_data),
    .tx_charisk(tx_charisk),
    .rx_valid(rx_valid),
    .rx_ready(rx_ready),
    .rx_data(rx_data),
    .rx_charisk(rx_charisk),
    .rx_notintable(rx_notintable),
    .rx_disperr(rx_disperr),
    .serdes_tx_data(serdes_tx_data),
    .serdes_rx_data(serdes_rx_data),
    .status_ctrl_state(status_ctrl_state),
    .status_lane_cgs_state(status_lane_cgs_state),
    .status_lane_ifs_ready(status_lane_ifs_ready),
    .sync_n(sync_n),
    .ilas_config_valid(ilas_config_valid),
    .ilas_config_addr(ilas_config_addr),
    .ilas_config_data(),
    .status_err_statistics_cnt(),
    .event_frame_alignment_error());

  // ---------------------------------------------------------------
  // Test
  // ---------------------------------------------------------------
  initial begin
    reset = 1;
    cfg_octets_per_multiframe = 10'd64;
    cfg_octets_per_frame = 8'd4;
    cfg_lanes_disable = 4'b0000;
    cfg_links_disable = 1'b0;
    cfg_disable_scrambler = 1'b1;
    cfg_disable_char_replacement = 1'b1;
    cfg_mframes_per_ilas = 8'd4;
    cfg_skip_ilas = 1'b0;
    cfg_continuous_cgs = 1'b0;
    cfg_continuous_ilas = 1'b0;
    tx_valid = 1'b0;
    tx_data = 64'b0;
    tx_charisk = 8'b0;
    rx_ready = 1'b1;
    tx_count = 0;
    rx_count = 0;
    error_count = 0;

    repeat(20) @(posedge clk);
    reset = 0;
    repeat(20) @(posedge clk);

    $display("[%0t] Waiting for link training (4 lanes, max skew %0d cycles)...", $time, MAX_SKEW);

    wait(status_ctrl_state == 2'b11);
    $display("[%0t] TX entered DATA state", $time);

    wait(rx_valid == 1'b1);
    $display("[%0t] RX buffer released - lane alignment complete!", $time);

    repeat(50) @(posedge clk);

    $display("[%0t] Starting data transfer...", $time);

    fork
      begin : tx_proc
        integer k;
        for (k = 0; k < 100; k = k + 1) begin
          @(posedge clk);
          if (tx_ready) begin
            tx_valid <= 1'b1;
            tx_data <= {4{k[7:0], k[7:0]}};
            tx_charisk <= 8'b0;
            tx_count <= tx_count + 1;
          end else begin
            tx_valid <= 1'b0;
          end
        end
        tx_valid <= 1'b0;
      end
      begin : rx_proc
        integer k;
        for (k = 0; k < 100; k = k + 1) begin
          @(posedge clk);
          if (rx_valid && rx_ready) rx_count <= rx_count + 1;
        end
      end
    join

    repeat(200) @(posedge clk);

    $display("========================================");
    $display("4-Lane Skew Test Results:");
    $display("  Lane 0 delay: %0d cycles", lane0_delay);
    $display("  Lane 1 delay: %0d cycles", lane1_delay);
    $display("  Lane 2 delay: %0d cycles", lane2_delay);
    $display("  Lane 3 delay: %0d cycles", lane3_delay);
    $display("  TX count: %0d", tx_count);
    $display("  RX count: %0d", rx_count);
    $display("  Final state: %0b", status_ctrl_state);
    $display("========================================");

    if (status_ctrl_state == 2'b11 && rx_valid && tx_count > 0)
      $display("SUCCESS: 4-lane link training with skew completed");
    else
      $display("FAILED");

    $finish;
  end

  initial begin
    #2000000;
    $display("TIMEOUT");
    $finish;
  end

  reg [1:0] prev_state;
  always @(posedge clk) begin
    if (!reset && status_ctrl_state !== prev_state) begin
      case (status_ctrl_state)
        2'b00: $display("[%0t] State: RESET", $time);
        2'b01: $display("[%0t] State: CGS", $time);
        2'b10: $display("[%0t] State: ILAS", $time);
        2'b11: $display("[%0t] State: DATA", $time);
      endcase
      prev_state <= status_ctrl_state;
    end
  end

  initial begin
    $dumpfile("jesd204_pcs_link_training_tb.vcd");
    $dumpvars(0, jesd204_pcs_link_training_tb);
  end

endmodule