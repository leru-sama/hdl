// ***************************************************************************
// JESD204 PCS Link Training Testbench - 2 DUTs point-to-point
//
// Two independent jesd204_pcs_link_training instances:
//   * tx_device  : application -> serdes  (drives the line)
//   * rx_device  : serdes -> application  (recovers the link)
//
// The two are connected only through a 4-lane "serdes" model that injects a
// random per-lane skew (0..MAX_SKEW parallel-clock cycles).  The RX parallel
// clock is frequency-locked to the TX (as a real CDR-recovered clock would be)
// but phase-shifted, so the elastic buffers must absorb both the phase offset
// and the inter-lane skew.
//
// Data integrity is checked with a latency-tolerant scoreboard:
//   * TX transmits a fixed marker PREAMBLE followed by a per-lane distinct
//     incrementing payload, one beat per parallel clock while in DATA.
//   * RX collects every valid application beat into a stream.
//   * After the run we locate the PREAMBLE in the RX stream and verify the
//     payload that follows - for every lane and every octet - proving both
//     data integrity and correct lane de-skew/alignment.
//
// NOTE: Frame-align error detection requires /A/ (/K28.5,D=3/) and /F/
// (/K28.5,D=7/) characters in the data stream at the expected EOMF/EOF
// boundaries.  With random application data these characters are not present,
// so the error counter stays at zero.  The frame-align check path is enabled
// and wired (ENABLE_FRAME_ALIGN_ERR_RESET=1), and the resync path is
// exercised by the link training itself (CGS -> ILAS -> DATA).
// ***************************************************************************
`timescale 1ns/100ps

module jesd204_pcs_link_training_tb;

  parameter NUM_LANES = 4;
  parameter NUM_LINKS = 1;
  parameter DATA_PATH_WIDTH = 4;
  parameter CLK_PERIOD = 10;
  parameter MAX_SKEW = 16;
  parameter SEED = 1;

  parameter PREAMBLE_LEN = 4;
  parameter PAYLOAD_LEN  = 256;

  localparam DW = DATA_PATH_WIDTH*8*NUM_LANES;
  localparam CW = DATA_PATH_WIDTH*NUM_LANES;
  localparam SW = DATA_PATH_WIDTH*10*NUM_LANES;

  function [7:0] preamble_byte;
    input integer idx;
    begin
      case (idx)
        0: preamble_byte = 8'hF0;
        1: preamble_byte = 8'hE1;
        2: preamble_byte = 8'hD2;
        3: preamble_byte = 8'hC3;
        default: preamble_byte = 8'hAA;
      endcase
    end
  endfunction

  function [7:0] payload_byte;
    input integer p;
    input integer lane;
    input integer octet;
    begin
      payload_byte = (p + lane*17 + octet*128) & 8'hFF;
    end
  endfunction

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
  reg [DW-1:0] tx_app_data;
  reg [CW-1:0] tx_app_charisk;

  wire [SW-1:0] tx_serdes_data;
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
  wire [DW-1:0] rx_app_data;
  wire [CW-1:0] rx_app_charisk;
  wire [CW-1:0] rx_app_notintable;
  wire [CW-1:0] rx_app_disperr;

  reg  [SW-1:0] rx_serdes_data;
  wire [1:0] rx_status_state;
  wire [NUM_LINKS-1:0] rx_sync_n;
  wire [NUM_LANES-1:0] rx_ilas_config_valid;
  wire rx_event_frame_alignment_error;
  wire [NUM_LANES-1:0] rx_frame_align_err_cnt_0;

  // ---------------------------------------------------------------
  // Serdes model: per-lane random skew (0..MAX_SKEW parallel cycles)
  // ---------------------------------------------------------------
  reg [39:0] lane_pipe [0:NUM_LANES-1][0:MAX_SKEW];
  integer lane_delay [0:NUM_LANES-1];
  integer i;
  integer sh_i, sh_s;
  integer cm_i;
  integer ts;

  integer seed;
  initial begin
    seed = SEED;
    for (i = 0; i < NUM_LANES; i = i + 1) begin
      lane_delay[i] = {$random(seed)} % (MAX_SKEW + 1);
      $display("Lane %0d skew: %0d cycles", i, lane_delay[i]);
    end
  end

  always @(posedge rx_clk) begin
    for (sh_i = 0; sh_i < NUM_LANES; sh_i = sh_i + 1) begin
      lane_pipe[sh_i][0] <= tx_serdes_data[sh_i*40 +: 40];
      for (sh_s = 1; sh_s <= MAX_SKEW; sh_s = sh_s + 1)
        lane_pipe[sh_i][sh_s] <= lane_pipe[sh_i][sh_s-1];
    end
  end

  always @(*) begin
    for (cm_i = 0; cm_i < NUM_LANES; cm_i = cm_i + 1)
      rx_serdes_data[cm_i*40 +: 40] = lane_pipe[cm_i][lane_delay[cm_i]];
  end

  // ---------------------------------------------------------------
  // Clocks
  // ---------------------------------------------------------------
  initial tx_clk = 0;
  always #(CLK_PERIOD/2) tx_clk = ~tx_clk;

  initial begin
    rx_clk = 0;
    #3;
    forever #(CLK_PERIOD/2) rx_clk = ~rx_clk;
  end

  // ---------------------------------------------------------------
  // TX Device (DUT 1)
  // ---------------------------------------------------------------
  jesd204_pcs_link_training #(
    .NUM_LANES(NUM_LANES),
    .NUM_LINKS(NUM_LINKS),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .ENABLE_FRAME_ALIGN_CHECK(1),
    .ENABLE_CHAR_REPLACE(1),
    .ENABLE_FRAME_ALIGN_ERR_RESET(1),
    .FRAME_ALIGN_ERR_THRESHOLD(8'd16),
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
    .serdes_rx_data({SW{1'b0}}),
    .sync_request_n(rx_sync_n),
    .status_ctrl_state(tx_status_state),
    .status_lane_cgs_state(),
    .status_lane_ifs_ready(),
    .sync_n(tx_sync_n),
    .ilas_config_valid(),
    .ilas_config_addr(),
    .ilas_config_data(),
    .status_err_statistics_cnt(),
    .status_frame_align_err_cnt_0(),
    .event_frame_alignment_error());

  // ---------------------------------------------------------------
  // RX Device (DUT 2)
  // ---------------------------------------------------------------
  jesd204_pcs_link_training #(
    .NUM_LANES(NUM_LANES),
    .NUM_LINKS(NUM_LINKS),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .ENABLE_FRAME_ALIGN_CHECK(1),
    .ENABLE_CHAR_REPLACE(1),
    .ENABLE_FRAME_ALIGN_ERR_RESET(1),
    .FRAME_ALIGN_ERR_THRESHOLD(8'd16),
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
    .tx_data({DW{1'b0}}),
    .tx_charisk({CW{1'b0}}),
    .rx_valid(rx_app_valid),
    .rx_ready(rx_app_ready),
    .rx_data(rx_app_data),
    .rx_charisk(rx_app_charisk),
    .rx_notintable(rx_app_notintable),
    .rx_disperr(rx_app_disperr),
    .serdes_tx_data(),
    .serdes_rx_data(rx_serdes_data),
    .sync_request_n({NUM_LINKS{1'b1}}),
    .status_ctrl_state(rx_status_state),
    .status_lane_cgs_state(),
    .status_lane_ifs_ready(),
    .sync_n(rx_sync_n),
    .ilas_config_valid(rx_ilas_config_valid),
    .ilas_config_addr(),
    .ilas_config_data(),
    .status_err_statistics_cnt(),
    .status_frame_align_err_cnt_0(rx_frame_align_err_cnt_0),
    .event_frame_alignment_error(rx_event_frame_alignment_error));

  // ---------------------------------------------------------------
  // Common configuration
  // ---------------------------------------------------------------
  task apply_cfg;
    begin
      tx_cfg_octets_per_multiframe = 10'd32;
      tx_cfg_octets_per_frame = 8'd4;
      tx_cfg_lanes_disable = {NUM_LANES{1'b0}};
      tx_cfg_links_disable = {NUM_LINKS{1'b0}};
      tx_cfg_disable_scrambler = 1'b1;
      tx_cfg_disable_char_replacement = 1'b0;
      tx_cfg_mframes_per_ilas = 8'd4;
      tx_cfg_skip_ilas = 1'b0;
      tx_cfg_continuous_cgs = 1'b0;
      tx_cfg_continuous_ilas = 1'b0;

      rx_cfg_octets_per_multiframe = 10'd32;
      rx_cfg_octets_per_frame = 8'd4;
      rx_cfg_lanes_disable = {NUM_LANES{1'b0}};
      rx_cfg_links_disable = {NUM_LINKS{1'b0}};
      rx_cfg_disable_scrambler = 1'b1;
      rx_cfg_disable_char_replacement = 1'b0;
      rx_cfg_mframes_per_ilas = 8'd4;
      rx_cfg_skip_ilas = 1'b0;
      rx_cfg_continuous_cgs = 1'b0;
      rx_cfg_continuous_ilas = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------
  // Reference and capture streams
  // ---------------------------------------------------------------
  localparam TOTAL = PREAMBLE_LEN + PAYLOAD_LEN;
  localparam CAP_MAX = 4096;

  reg [DW-1:0] tx_ref [0:TOTAL-1];
  integer tx_beats;
  reg [DW-1:0] rx_cap [0:CAP_MAX-1];
  integer rx_beats;
  integer match_count, mismatch_count, anchor;

  function [DW-1:0] gen_word;
    input integer bn;
    integer l, p;
    reg [DW-1:0] w;
    begin
      w = {DW{1'b0}};
      if (bn < PREAMBLE_LEN) begin
        for (l = 0; l < NUM_LANES; l = l + 1) begin
          w[(l*DATA_PATH_WIDTH+0)*8 +: 8] = preamble_byte(bn);
          w[(l*DATA_PATH_WIDTH+1)*8 +: 8] = preamble_byte(bn);
          w[(l*DATA_PATH_WIDTH+2)*8 +: 8] = preamble_byte(bn);
          w[(l*DATA_PATH_WIDTH+3)*8 +: 8] = preamble_byte(bn);
        end
      end else begin
        p = bn - PREAMBLE_LEN;
        for (l = 0; l < NUM_LANES; l = l + 1) begin
          w[(l*DATA_PATH_WIDTH+0)*8 +: 8] = payload_byte(p, l, 0);
          w[(l*DATA_PATH_WIDTH+1)*8 +: 8] = payload_byte(p, l, 1);
          w[(l*DATA_PATH_WIDTH+2)*8 +: 8] = payload_byte(p, l, 2);
          w[(l*DATA_PATH_WIDTH+3)*8 +: 8] = payload_byte(p, l, 3);
        end
      end
      gen_word = w;
    end
  endfunction

  // ---------------------------------------------------------------
  // Main stimulus
  // ---------------------------------------------------------------
  initial begin
    tx_reset = 1;
    rx_reset = 1;
    apply_cfg;

    tx_app_valid = 1'b0;
    tx_app_data  = {DW{1'b0}};
    tx_app_charisk = {CW{1'b0}};
    rx_app_ready = 1'b1;
    tx_beats = 0;
    rx_beats = 0;
    match_count = 0;
    mismatch_count = 0;
    anchor = -1;

    repeat(20) @(posedge tx_clk);
    tx_reset = 0;
    rx_reset = 0;
    repeat(20) @(posedge tx_clk);

    $display("[%0t] Starting 2-DUT test (%0d lanes, DPW=%0d, max skew %0d cycles)...",
             $time, NUM_LANES, DATA_PATH_WIDTH, MAX_SKEW);

    wait(tx_status_state == 2'b11);
    $display("[%0t] TX device reached DATA state", $time);
    wait(rx_app_valid == 1'b1);
    $display("[%0t] RX de-skew complete, application data valid", $time);

    repeat(40) @(posedge tx_clk);

    $display("[%0t] Transmitting %0d marker + %0d payload beats...",
             $time, PREAMBLE_LEN, PAYLOAD_LEN);

    for (ts = 0; ts < TOTAL; ts = ts + 1) begin
      @(posedge tx_clk);
      tx_app_valid <= 1'b1;
      tx_app_charisk <= {CW{1'b0}};
      tx_app_data <= gen_word(ts);
      tx_ref[ts] = gen_word(ts);
      tx_beats = tx_beats + 1;
    end
    @(posedge tx_clk);
    tx_app_valid <= 1'b0;
    tx_app_data  <= {DW{1'b0}};

    repeat(600) @(posedge tx_clk);

    verify_stream;

    $display("========================================");
    $display("2-DUT Test Results (DPW=%0d):", DATA_PATH_WIDTH);
    $display("  Lane skews:  %0d, %0d, %0d, %0d",
             lane_delay[0], lane_delay[1], lane_delay[2], lane_delay[3]);
    $display("  TX beats:    %0d", tx_beats);
    $display("  RX beats:    %0d", rx_beats);
    $display("  Anchor idx:  %0d", anchor);
    $display("  Matches:     %0d", match_count);
    $display("  Mismatches:  %0d", mismatch_count);
    $display("  Frame-align err cnt (lane 0): %0d", rx_frame_align_err_cnt_0);
    $display("  TX state:    %0b", tx_status_state);
    $display("========================================");

    if (tx_status_state == 2'b11 && anchor >= 0 &&
        match_count == PAYLOAD_LEN && mismatch_count == 0)
      $display("SUCCESS: 2-DUT link training + data transfer verified across %0d lanes (DPW=%0d)",
               NUM_LANES, DATA_PATH_WIDTH);
    else
      $display("FAILED");

    $finish;
  end

  // ---------------------------------------------------------------
  // RX capture
  // ---------------------------------------------------------------
  always @(posedge rx_clk) begin
    if (!rx_reset && rx_app_valid && rx_app_ready && rx_beats < CAP_MAX) begin
      rx_cap[rx_beats] <= rx_app_data;
      rx_beats <= rx_beats + 1;
    end
  end

  // ---------------------------------------------------------------
  // Scoreboard
  // ---------------------------------------------------------------
  task verify_stream;
    integer idx, k, l, oct, ok;
    reg [7:0] got, exp;
    begin
      anchor = -1;
      for (idx = 0; idx + PREAMBLE_LEN <= rx_beats && anchor < 0; idx = idx + 1) begin
        ok = 1;
        for (k = 0; k < PREAMBLE_LEN && ok; k = k + 1)
          if (rx_cap[idx+k] !== tx_ref[k]) ok = 0;
        if (ok) anchor = idx;
      end

      if (anchor < 0) begin
        $display("[%0t] ERROR: preamble not found in RX capture (%0d beats)",
                 $time, rx_beats);
        disable verify_stream;
      end

      if (anchor + TOTAL > rx_beats) begin
        $display("[%0t] ERROR: incomplete capture: anchor=%0d TOTAL=%0d beats=%0d",
                 $time, anchor, TOTAL, rx_beats);
        disable verify_stream;
      end

      $display("[%0t] Preamble anchored at RX beat %0d", $time, anchor);

      for (k = 0; k < PAYLOAD_LEN; k = k + 1) begin
        ok = 1;
        for (l = 0; l < NUM_LANES; l = l + 1) begin
          for (oct = 0; oct < DATA_PATH_WIDTH; oct = oct + 1) begin
            got = rx_cap[anchor+PREAMBLE_LEN+k][(l*DATA_PATH_WIDTH+oct)*8 +: 8];
            exp = payload_byte(k, l, oct);
            if (got !== exp) begin
              ok = 0;
              if (mismatch_count < 10)
                $display("[%0t] MISMATCH beat %0d lane %0d oct %0d: got=%02h exp=%02h",
                         $time, k, l, oct, got, exp);
            end
          end
        end
        if (ok) match_count = match_count + 1;
        else    mismatch_count = mismatch_count + 1;
      end
    end
  endtask

  // ---------------------------------------------------------------
  // Timeout
  // ---------------------------------------------------------------
  initial begin
    #5000000;
    $display("TIMEOUT: tx_state=%0b rx_valid=%0b rx_beats=%0d",
             tx_status_state, rx_app_valid, rx_beats);
    $finish;
  end

  // ---------------------------------------------------------------
  // State tracing
  // ---------------------------------------------------------------
  reg [1:0] tx_prev_state;
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

  reg [1:0] rx_prev_state;
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
