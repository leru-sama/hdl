// ***************************************************************************
// JESD204 PCS Link Training Testbench
// 2 DUTs: TX device <-> RX device with per-lane random skew
// ***************************************************************************
`timescale 1ns/100ps

module jesd204_pcs_link_training_tb;

  parameter NUM_LANES = 4;
  parameter NUM_LINKS = 1;
  parameter DATA_PATH_WIDTH = 2;
  parameter CLK_PERIOD = 10;
  parameter MAX_SKEW = 16;

  // ---------------------------------------------------------------
  // TX Device signals
  // ---------------------------------------------------------------
  reg tx_clk, tx_reset;
  reg [9:0] tx_cfg_octets_per_multiframe;
  reg [7:0] tx_cfg_octets_per_frame;
  reg [NUM_LANES-1:0] tx_cfg_lanes_disable;
  reg [NUM_LINKS-1:0] tx_cfg_links_disable;
  reg tx_cfg_disable_scrambler;
  reg tx_cfg_disable_char_replacement;
  reg [7:0] tx_cfg_mframes_per_ilas;
  reg tx_cfg_skip_ilas;
  reg tx_cfg_continuous_cgs;
  reg tx_cfg_continuous_ilas;

  reg tx_app_valid;
  wire tx_app_ready;
  reg [DATA_PATH_WIDTH*8*NUM_LANES-1:0] tx_app_data;
  reg [DATA_PATH_WIDTH*NUM_LANES-1:0] tx_app_charisk;

  wire [DATA_PATH_WIDTH*10*NUM_LANES-1:0] tx_serdes_data;
  wire [1:0] tx_status_state;
  wire [NUM_LINKS-1:0] tx_sync_n;

  // ---------------------------------------------------------------
  // RX Device signals
  // ---------------------------------------------------------------
  reg rx_clk, rx_reset;
  reg [9:0] rx_cfg_octets_per_multiframe;
  reg [7:0] rx_cfg_octets_per_frame;
  reg [NUM_LANES-1:0] rx_cfg_lanes_disable;
  reg [NUM_LINKS-1:0] rx_cfg_links_disable;
  reg rx_cfg_disable_scrambler;
  reg rx_cfg_disable_char_replacement;
  reg [7:0] rx_cfg_mframes_per_ilas;
  reg rx_cfg_skip_ilas;
  reg rx_cfg_continuous_cgs;
  reg rx_cfg_continuous_ilas;

  wire rx_app_valid;
  reg rx_app_ready;
  wire [DATA_PATH_WIDTH*8*NUM_LANES-1:0] rx_app_data;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_app_charisk;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_app_notintable;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_app_disperr;

  wire [DATA_PATH_WIDTH*10*NUM_LANES-1:0] rx_serdes_data;
  wire [1:0] rx_status_state;
  wire [NUM_LINKS-1:0] rx_sync_n;
  wire [NUM_LANES-1:0] rx_ilas_config_valid;

  // ---------------------------------------------------------------
  // Serdes with per-lane random skew
  // ---------------------------------------------------------------
  reg [19:0] lane0_pipe [0:MAX_SKEW];
  reg [19:0] lane1_pipe [0:MAX_SKEW];
  reg [19:0] lane2_pipe [0:MAX_SKEW];
  reg [19:0] lane3_pipe [0:MAX_SKEW];
  integer lane_delay [0:NUM_LANES-1];
  integer i;

  initial begin
    for (i = 0; i < NUM_LANES; i = i + 1) begin
      lane_delay[i] = $urandom_range(0, MAX_SKEW);
      $display("Lane %0d delay: %0d cycles", i, lane_delay[i]);
    end
  end

  always @(posedge rx_clk) begin
    lane0_pipe[0] <= tx_serdes_data[19:0];
    lane1_pipe[0] <= tx_serdes_data[39:20];
    lane2_pipe[0] <= tx_serdes_data[59:40];
    lane3_pipe[0] <= tx_serdes_data[79:60];
    for (i = 1; i <= MAX_SKEW; i = i + 1) begin
      lane0_pipe[i] <= lane0_pipe[i-1];
      lane1_pipe[i] <= lane1_pipe[i-1];
      lane2_pipe[i] <= lane2_pipe[i-1];
      lane3_pipe[i] <= lane3_pipe[i-1];
    end
  end

  assign rx_serdes_data[19:0]  = lane0_pipe[lane_delay[0]];
  assign rx_serdes_data[39:20] = lane1_pipe[lane_delay[1]];
  assign rx_serdes_data[59:40] = lane2_pipe[lane_delay[2]];
  assign rx_serdes_data[79:60] = lane3_pipe[lane_delay[3]];

  // ---------------------------------------------------------------
  // Clock generation (independent clocks)
  // ---------------------------------------------------------------
  initial tx_clk = 0;
  always #(CLK_PERIOD/2) tx_clk = ~tx_clk;

  initial rx_clk = 0;
  always #(CLK_PERIOD/2 + 1) rx_clk = ~rx_clk;

  // ---------------------------------------------------------------
  // TX Device (DUT 1) - receives sync_request_n from RX device
  // ---------------------------------------------------------------
  jesd204_pcs_link_training #(
    .NUM_LANES(NUM_LANES),
    .NUM_LINKS(NUM_LINKS),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .ENABLE_FRAME_ALIGN_CHECK(0),
    .ENABLE_CHAR_REPLACE(0),
    .ELASTIC_BUFFER_SIZE(256)
  ) tx_device (
    .clk(tx_clk),
    .reset(tx_reset),
    .cfg_octets_per_multiframe(tx_cfg_octets_per_multiframe),
    .cfg_octets_per_frame(tx_cfg_octets_per_frame),
    .cfg_lanes_disable(tx_cfg_lanes_disable),
    .cfg_links_disable(tx_cfg_links_disable),
    .cfg_disable_scrambler(tx_cfg_disable_scrambler),
    .cfg_disable_char_replacement(tx_cfg_disable_char_replacement),
    .cfg_mframes_per_ilas(tx_cfg_mframes_per_ilas),
    .cfg_skip_ilas(tx_cfg_skip_ilas),
    .cfg_continuous_cgs(tx_cfg_continuous_cgs),
    .cfg_continuous_ilas(tx_cfg_continuous_ilas),
    .tx_valid(tx_app_valid),
    .tx_ready(tx_app_ready),
    .tx_data(tx_app_data),
    .tx_charisk(tx_app_charisk),
    .rx_valid(),
    .rx_ready(1'b0),
    .rx_data(),
    .rx_charisk(),
    .rx_notintable(),
    .rx_disperr(),
    .serdes_tx_data(tx_serdes_data),
    .serdes_rx_data({NUM_LANES{20'b0}}),
    .sync_request_n(rx_sync_n),  // RX device sync_n drives TX device
    .status_ctrl_state(tx_status_state),
    .status_lane_cgs_state(),
    .status_lane_ifs_ready(),
    .sync_n(tx_sync_n),  // TX device's own sync_n (unused)
    .ilas_config_valid(),
    .ilas_config_addr(),
    .ilas_config_data(),
    .status_err_statistics_cnt(),
    .event_frame_alignment_error());

  // ---------------------------------------------------------------
  // RX Device (DUT 2) - no sync_request needed (always ready)
  // ---------------------------------------------------------------
  jesd204_pcs_link_training #(
    .NUM_LANES(NUM_LANES),
    .NUM_LINKS(NUM_LINKS),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .ENABLE_FRAME_ALIGN_CHECK(0),
    .ENABLE_CHAR_REPLACE(0),
    .ELASTIC_BUFFER_SIZE(256)
  ) rx_device (
    .clk(rx_clk),
    .reset(rx_reset),
    .cfg_octets_per_multiframe(rx_cfg_octets_per_multiframe),
    .cfg_octets_per_frame(rx_cfg_octets_per_frame),
    .cfg_lanes_disable(rx_cfg_lanes_disable),
    .cfg_links_disable(rx_cfg_links_disable),
    .cfg_disable_scrambler(rx_cfg_disable_scrambler),
    .cfg_disable_char_replacement(rx_cfg_disable_char_replacement),
    .cfg_mframes_per_ilas(rx_cfg_mframes_per_ilas),
    .cfg_skip_ilas(rx_cfg_skip_ilas),
    .cfg_continuous_cgs(rx_cfg_continuous_cgs),
    .cfg_continuous_ilas(rx_cfg_continuous_ilas),
    .tx_valid(1'b0),
    .tx_ready(),
    .tx_data({NUM_LANES{16'b0}}),
    .tx_charisk({NUM_LANES{2'b0}}),
    .rx_valid(rx_app_valid),
    .rx_ready(rx_app_ready),
    .rx_data(rx_app_data),
    .rx_charisk(rx_app_charisk),
    .rx_notintable(rx_app_notintable),
    .rx_disperr(rx_app_disperr),
    .serdes_tx_data(),
    .serdes_rx_data(rx_serdes_data),
    .sync_request_n({NUM_LINKS{1'b1}}),  // No sync request from TX side
    .status_ctrl_state(rx_status_state),
    .status_lane_cgs_state(),
    .status_lane_ifs_ready(),
    .sync_n(rx_sync_n),  // RX device's sync_n goes to TX device
    .ilas_config_valid(rx_ilas_config_valid),
    .ilas_config_addr(),
    .ilas_config_data(),
    .status_err_statistics_cnt(),
    .event_frame_alignment_error());

  // ---------------------------------------------------------------
  // Test control
  // ---------------------------------------------------------------
  integer tx_count, rx_count;
  reg [7:0] expected_data [0:99];
  integer match_count, mismatch_count;

  // ---------------------------------------------------------------
  // Test sequence
  // ---------------------------------------------------------------
  initial begin
    tx_reset = 1;
    rx_reset = 1;
    tx_cfg_octets_per_multiframe = 10'd64;
    tx_cfg_octets_per_frame = 8'd4;
    tx_cfg_lanes_disable = 4'b0000;
    tx_cfg_links_disable = 1'b0;
    tx_cfg_disable_scrambler = 1'b1;
    tx_cfg_disable_char_replacement = 1'b1;
    tx_cfg_mframes_per_ilas = 8'd4;
    tx_cfg_skip_ilas = 1'b0;
    tx_cfg_continuous_cgs = 1'b0;
    tx_cfg_continuous_ilas = 1'b0;

    rx_cfg_octets_per_multiframe = 10'd64;
    rx_cfg_octets_per_frame = 8'd4;
    rx_cfg_lanes_disable = 4'b0000;
    rx_cfg_links_disable = 1'b0;
    rx_cfg_disable_scrambler = 1'b1;
    rx_cfg_disable_char_replacement = 1'b1;
    rx_cfg_mframes_per_ilas = 8'd4;
    rx_cfg_skip_ilas = 1'b0;
    rx_cfg_continuous_cgs = 1'b0;
    rx_cfg_continuous_ilas = 1'b0;

    tx_app_valid = 1'b0;
    tx_app_data = 64'b0;
    tx_app_charisk = 8'b0;
    rx_app_ready = 1'b1;
    tx_count = 0;
    rx_count = 0;
    match_count = 0;
    mismatch_count = 0;

    for (i = 0; i < 100; i = i + 1)
      expected_data[i] = i[7:0];

    repeat(20) @(posedge tx_clk);
    tx_reset = 0;
    rx_reset = 0;
    repeat(20) @(posedge tx_clk);

    $display("[%0t] Starting 2-DUT test (4 lanes, max skew %0d cycles)...", $time, MAX_SKEW);

    wait(tx_status_state == 2'b11);
    $display("[%0t] TX device entered DATA state", $time);

    // Wait for RX to complete lane alignment
    wait(rx_app_valid == 1'b1);
    $display("[%0t] RX buffer released - lane alignment complete!", $time);

    repeat(50) @(posedge tx_clk);

    $display("[%0t] Sending data from TX to RX...", $time);

    // Debug: check serdes signals
    $display("[%0t] TX serdes[19:0]=%020b", $time, tx_serdes_data[19:0]);
    $display("[%0t] RX serdes[19:0]=%020b", $time, rx_serdes_data[19:0]);
    $display("[%0t] TX phy_data[15:0]=%016h", $time, tx_device.tx_phy_data[15:0]);
    $display("[%0t] RX phy_data[15:0]=%016h", $time, rx_device.rx_phy_data[15:0]);

    fork
      begin : tx_send
        integer k;
        for (k = 0; k < 100; k = k + 1) begin
          @(posedge tx_clk);
          if (tx_app_ready) begin
            tx_app_valid <= 1'b1;
            tx_app_data <= {4{expected_data[k], expected_data[k]}};
            tx_app_charisk <= 8'b0;
            tx_count <= tx_count + 1;
          end else begin
            tx_app_valid <= 1'b0;
          end
        end
        tx_app_valid <= 1'b0;
      end

      begin : rx_check
        integer k;
        reg [7:0] rx_byte;
        for (k = 0; k < 100; k = k + 1) begin
          @(posedge rx_clk);
          if (rx_app_valid && rx_app_ready) begin
            rx_byte = rx_app_data[7:0];
            if (k < 10)
              $display("[%0t] RX[%0d]: got=%02h exp=%02h data=%08h", $time, k, rx_byte, expected_data[k], rx_app_data[31:0]);
            if (rx_byte == expected_data[k])
              match_count = match_count + 1;
            else begin
              mismatch_count = mismatch_count + 1;
            end
            rx_count = rx_count + 1;
          end
        end
      end
    join

    repeat(200) @(posedge tx_clk);

    $display("========================================");
    $display("2-DUT Test Results:");
    $display("  Lane delays: %0d, %0d, %0d, %0d", lane_delay[0], lane_delay[1], lane_delay[2], lane_delay[3]);
    $display("  TX sent:     %0d", tx_count);
    $display("  RX received: %0d", rx_count);
    $display("  Matches:     %0d", match_count);
    $display("  Mismatches:  %0d", mismatch_count);
    $display("  TX state: %0b, RX state: %0b", tx_status_state, rx_status_state);
    $display("========================================");

    if (tx_status_state == 2'b11 && rx_status_state == 2'b11 &&
        rx_app_valid && tx_count > 0 && mismatch_count == 0)
      $display("SUCCESS: 2-DUT link training and data transfer completed");
    else
      $display("FAILED");

    $finish;
  end

  initial begin
    #3000000;
    $display("TIMEOUT: tx_state=%0b rx_state=%0b", tx_status_state, rx_status_state);
    $finish;
  end

  reg [1:0] tx_prev_state, rx_prev_state;
  always @(posedge tx_clk) begin
    if (!tx_reset && tx_status_state !== tx_prev_state) begin
      case (tx_status_state)
        2'b00: $display("[%0t] TX: RESET", $time);
        2'b01: $display("[%0t] TX: CGS", $time);
        2'b10: $display("[%0t] TX: ILAS", $time);
        2'b11: $display("[%0t] TX: DATA", $time);
      endcase
      tx_prev_state <= tx_status_state;
    end
  end
  always @(posedge rx_clk) begin
    if (!rx_reset && rx_status_state !== rx_prev_state) begin
      case (rx_status_state)
        2'b00: $display("[%0t] RX: RESET", $time);
        2'b01: $display("[%0t] RX: CGS", $time);
        2'b10: $display("[%0t] RX: ILAS", $time);
        2'b11: $display("[%0t] RX: DATA", $time);
      endcase
      rx_prev_state <= rx_status_state;
    end
  end
  always @(posedge rx_clk) begin
    if (!rx_reset && rx_status_state !== rx_prev_state) begin
      case (rx_status_state)
        2'b00: $display("[%0t] RX: RESET", $time);
        2'b01: $display("[%0t] RX: CGS", $time);
        2'b10: $display("[%0t] RX: ILAS", $time);
        2'b11: $display("[%0t] RX: DATA", $time);
      endcase
      rx_prev_state <= rx_status_state;
    end
  end

  initial begin
    $dumpfile("jesd204_pcs_link_training_tb.vcd");
    $dumpvars(0, jesd204_pcs_link_training_tb);
  end

endmodule