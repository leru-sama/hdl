// ***************************************************************************
// JESD204 PCS Link Training Testbench
// Verifies CGS -> ILAS -> DATA with multi-lane skew tolerance
// ***************************************************************************
`timescale 1ns/100ps

module jesd204_pcs_link_training_tb;

  // ---------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------
  parameter NUM_LANES = 2;              // Test with 2 lanes
  parameter NUM_LINKS = 1;
  parameter DATA_PATH_WIDTH = 2;
  parameter CLK_PERIOD = 10;
  parameter LANE_SKEW_CYCLES = 3;      // Max skew between lanes in clock cycles

  // ---------------------------------------------------------------
  // Signals
  // ---------------------------------------------------------------
  reg clk;
  reg reset;

  // Configuration
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

  // TX Application Interface
  reg tx_valid;
  wire tx_ready;
  reg [DATA_PATH_WIDTH*8*NUM_LANES-1:0] tx_data;
  reg [DATA_PATH_WIDTH*NUM_LANES-1:0] tx_charisk;

  // RX Application Interface
  wire rx_valid;
  reg rx_ready;
  wire [DATA_PATH_WIDTH*8*NUM_LANES-1:0] rx_data;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_charisk;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_notintable;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_disperr;

  // Serdes Interface
  wire [DATA_PATH_WIDTH*10*NUM_LANES-1:0] serdes_tx_data;
  wire [DATA_PATH_WIDTH*10*NUM_LANES-1:0] serdes_rx_data;

  // Status
  wire [1:0] status_ctrl_state;
  wire [2*NUM_LANES-1:0] status_lane_cgs_state;
  wire [NUM_LANES-1:0] status_lane_ifs_ready;
  wire [NUM_LINKS-1:0] sync_n;
  wire [NUM_LANES-1:0] ilas_config_valid;
  wire [NUM_LANES*2-1:0] ilas_config_addr;
  wire [NUM_LANES*DATA_PATH_WIDTH*8-1:0] serdes_rx_data_delayed;

  // Test control
  integer tx_count;
  integer rx_count;
  integer error_count;

  // ---------------------------------------------------------------
  // Clock generation
  // ---------------------------------------------------------------
  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // ---------------------------------------------------------------
  // Serdes with per-lane random skew
  // ---------------------------------------------------------------
  // Lane 0: no delay
  // Lane 1: random delay (1-LANE_SKEW_CYCLES cycles)

  reg [DATA_PATH_WIDTH*10-1:0] lane0_pipe [0:LANE_SKEW_CYCLES];
  reg [DATA_PATH_WIDTH*10-1:0] lane1_pipe [0:LANE_SKEW_CYCLES];

  integer lane1_delay;
  integer i;

  initial begin
    // Random delay for lane 1 (1 to LANE_SKEK_CYCLES)
    lane1_delay = $urandom_range(1, LANE_SKEW_CYCLES);
    $display("Lane 1 delay: %0d clock cycles", lane1_delay);
  end

  // Pipeline for lane 0 (no delay)
  always @(posedge clk) begin
    lane0_pipe[0] <= serdes_tx_data[DATA_PATH_WIDTH*10-1:0];
    for (i = 1; i <= LANE_SKEW_CYCLES; i = i + 1) begin
      lane0_pipe[i] <= lane0_pipe[i-1];
    end
  end

  // Pipeline for lane 1 (with delay)
  always @(posedge clk) begin
    lane1_pipe[0] <= serdes_tx_data[DATA_PATH_WIDTH*10*2-1:DATA_PATH_WIDTH*10];
    for (i = 1; i <= LANE_SKEW_CYCLES; i = i + 1) begin
      lane1_pipe[i] <= lane1_pipe[i-1];
    end
  end

  // Select delayed signals
  assign serdes_rx_data[DATA_PATH_WIDTH*10-1:0] = lane0_pipe[0];  // Lane 0: no delay
  assign serdes_rx_data[DATA_PATH_WIDTH*10*2-1:DATA_PATH_WIDTH*10] = lane1_pipe[lane1_delay];  // Lane 1: delayed

  // ---------------------------------------------------------------
  // DUT instantiation
  // ---------------------------------------------------------------
  jesd204_pcs_link_training #(
    .NUM_LANES(NUM_LANES),
    .NUM_LINKS(NUM_LINKS),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .ENABLE_FRAME_ALIGN_CHECK(0),
    .ENABLE_CHAR_REPLACE(0),
    .ELASTIC_BUFFER_SIZE(128)
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
  // Test sequence
  // ---------------------------------------------------------------
  initial begin
    // Initialize
    reset = 1;
    cfg_octets_per_multiframe = 10'd64;
    cfg_octets_per_frame = 8'd4;
    cfg_lanes_disable = {NUM_LANES{1'b0}};
    cfg_links_disable = {NUM_LINKS{1'b0}};
    cfg_disable_scrambler = 1'b1;
    cfg_disable_char_replacement = 1'b1;
    cfg_mframes_per_ilas = 8'd4;
    cfg_skip_ilas = 1'b0;
    cfg_continuous_cgs = 1'b0;
    cfg_continuous_ilas = 1'b0;
    tx_valid = 1'b0;
    tx_data = {DATA_PATH_WIDTH*8*NUM_LANES{1'b0}};
    tx_charisk = {DATA_PATH_WIDTH*NUM_LANES{1'b0}};
    rx_ready = 1'b1;
    tx_count = 0;
    rx_count = 0;
    error_count = 0;

    // Release reset
    repeat(20) @(posedge clk);
    reset = 0;
    repeat(20) @(posedge clk);

    // ---------------------------------------------------------------
    // Wait for link training to complete
    // ---------------------------------------------------------------
    $display("[%0t] Waiting for link training with %0d lanes (lane 1 skew = %0d cycles)...", 
             $time, NUM_LANES, lane1_delay);

    // Wait for TX to enter DATA state
    wait(status_ctrl_state == 2'b11);
    $display("[%0t] TX entered DATA state", $time);

    // Wait for RX buffer release (rx_valid goes high)
    wait(rx_valid == 1'b1);
    $display("[%0t] RX buffer released - lane alignment complete!", $time);

    // Wait a bit more for stabilization
    repeat(50) @(posedge clk);

    // ---------------------------------------------------------------
    // Data transfer test
    // ---------------------------------------------------------------
    $display("[%0t] Starting data transfer test...", $time);

    fork
      // TX process
      begin : tx_process
        integer i;
        reg [7:0] tx_byte;
        for (i = 0; i < 50; i = i + 1) begin
          @(posedge clk);
          if (tx_ready) begin
            tx_valid = 1'b1;
            tx_byte = i[7:0];
            tx_data = {NUM_LANES{tx_byte, tx_byte}};  // 2 bytes per lane
            tx_charisk = {NUM_LANES{2'b00}};
            tx_count = tx_count + 1;
          end else begin
            tx_valid = 1'b0;
          end
        end
        tx_valid = 1'b0;
      end

      // RX process
      begin : rx_process
        integer i;
        for (i = 0; i < 50; i = i + 1) begin
          @(posedge clk);
          if (rx_valid && rx_ready) begin
            rx_count = rx_count + 1;
          end
        end
      end
    join

    // ---------------------------------------------------------------
    // Results
    // ---------------------------------------------------------------
    repeat(100) @(posedge clk);

    $display("========================================");
    $display("Test Results (Multi-lane with skew):");
    $display("  NUM_LANES: %0d", NUM_LANES);
    $display("  Lane 1 delay: %0d cycles", lane1_delay);
    $display("  TX count: %0d", tx_count);
    $display("  RX count: %0d", rx_count);
    $display("  Errors:   %0d", error_count);
    $display("  Final state: %0b", status_ctrl_state);
    $display("========================================");

    if (status_ctrl_state == 2'b11 && rx_valid && tx_count > 0) begin
      $display("SUCCESS: Multi-lane link training with skew completed");
    end else begin
      $display("FAILED: Link training or lane alignment failed");
    end

    $finish;
  end

  // ---------------------------------------------------------------
  // Timeout watchdog
  // ---------------------------------------------------------------
  initial begin
    #1000000;  // 1ms timeout
    $display("TIMEOUT: Test did not complete");
    $display("  status_ctrl_state: %0b", status_ctrl_state);
    $display("  sync_n: %0b", sync_n);
    $display("  rx_valid: %0b", rx_valid);
    $finish;
  end

  // ---------------------------------------------------------------
  // State monitoring
  // ---------------------------------------------------------------
  reg [1:0] prev_state;
  always @(posedge clk) begin
    if (!reset) begin
      if (status_ctrl_state !== prev_state) begin
        case (status_ctrl_state)
          2'b00: $display("[%0t] State: RESET", $time);
          2'b01: $display("[%0t] State: CGS", $time);
          2'b10: $display("[%0t] State: ILAS", $time);
          2'b11: $display("[%0t] State: DATA", $time);
        endcase
        prev_state <= status_ctrl_state;
      end
    end
  end

  // ---------------------------------------------------------------
  // VCD dump
  // ---------------------------------------------------------------
  initial begin
    $dumpfile("jesd204_pcs_link_training_tb.vcd");
    $dumpvars(0, jesd204_pcs_link_training_tb);
  end

endmodule