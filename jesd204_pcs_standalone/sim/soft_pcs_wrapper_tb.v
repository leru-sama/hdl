// ***************************************************************************
// ***************************************************************************
// Copyright (C) 2026 Analog Devices, Inc. All rights reserved.
// SPDX short identifier: ADIJESD204
// ***************************************************************************
// ***************************************************************************

`timescale 1ns/100ps

// Self-checking loopback test for jesd204_soft_pcs_wrapper.
//
//   application TX  ->  wrapper (encode)  ->  serdes line
//                                              |  (bit-rotation models an
//                                              |   arbitrary serdes bit slip)
//   application RX  <-  wrapper (decode)  <-  serdes line
//
// The test:
//   1. Holds the application idle (tx_valid=0) so the wrapper streams /K28.5/
//      commas while the RX pattern aligner locks (rx_patternalign_en=1).
//   2. Drops pattern-align, enables RX capture, and streams an incrementing
//      character sequence through the TX ready/valid interface, randomly
//      de-asserting tx_valid to exercise idle insertion.
//   3. Applies random back-pressure on the RX ready/valid interface and checks
//      that the recovered character stream increments monotonically with no
//      8b10b errors, no FIFO overflow, and that the expected number of payload
//      characters is received.

module soft_pcs_wrapper_tb;

  parameter VCD_FILE        = "soft_pcs_wrapper_tb.vcd";
  parameter NUM_LANES       = 1;
  parameter DATA_PATH_WIDTH = 2;                 // 20 bits per lane
  parameter BITSHIFT        = 3;                 // modeled serdes bit slip (0..9)
  parameter NUM_PAYLOAD     = 2000;              // payload characters to check

  localparam P        = NUM_LANES*DATA_PATH_WIDTH;
  localparam LANE_W   = DATA_PATH_WIDTH*10;      // raw bits per lane (20)
  localparam SERDES_W = NUM_LANES*LANE_W;

  reg clk = 1'b0;
  reg reset = 1'b1;
  integer failed = 0;

  always #5 clk = ~clk;

  // --------------------------------------------------------------------------
  // DUT connections
  // --------------------------------------------------------------------------
  reg                 tx_valid = 1'b0;
  wire                tx_ready;
  reg  [P*8-1:0]      tx_char = {P*8{1'b0}};
  reg  [P-1:0]        tx_charisk = {P{1'b0}};
  wire [SERDES_W-1:0] tx_serdes_data;

  reg  [SERDES_W-1:0] rx_serdes_data;
  reg                 rx_patternalign_en = 1'b1;
  reg                 rx_enable = 1'b0;
  wire                rx_valid;
  reg                 rx_ready = 1'b0;
  wire [P*8-1:0]      rx_char;
  wire [P-1:0]        rx_charisk;
  wire [P-1:0]        rx_notintable;
  wire [P-1:0]        rx_disperr;
  wire                rx_overflow;

  jesd204_soft_pcs_wrapper #(
    .NUM_LANES(NUM_LANES),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
    .RX_FILTER_IDLE(1),
    .RX_FIFO_ADDR_W(4)
  ) dut (
    .clk(clk),
    .reset(reset),

    .tx_valid(tx_valid),
    .tx_ready(tx_ready),
    .tx_char(tx_char),
    .tx_charisk(tx_charisk),
    .tx_serdes_data(tx_serdes_data),

    .rx_serdes_data(rx_serdes_data),
    .rx_patternalign_en(rx_patternalign_en),
    .rx_enable(rx_enable),
    .rx_valid(rx_valid),
    .rx_ready(rx_ready),
    .rx_char(rx_char),
    .rx_charisk(rx_charisk),
    .rx_notintable(rx_notintable),
    .rx_disperr(rx_disperr),
    .rx_overflow(rx_overflow)
  );

  // --------------------------------------------------------------------------
  // Transparent serdes model with a fixed bit-slip so the RX aligner has to
  // recover the symbol boundary (same technique as soft_pcs_loopback_tb).
  //   line = { current_beat, previous_beat[8:0] }[BITSHIFT +: SERDES_W]
  // --------------------------------------------------------------------------
  reg  [8:0]          serdes_hist = 9'h0;
  wire [SERDES_W+8:0] serdes_full = {tx_serdes_data, serdes_hist};

  always @(posedge clk) begin
    serdes_hist <= tx_serdes_data[SERDES_W-1 -: 9];
    rx_serdes_data <= serdes_full[BITSHIFT +: SERDES_W];
  end

  // --------------------------------------------------------------------------
  // TX stimulus : incrementing 8-bit payload, DATA_PATH_WIDTH chars per beat.
  // --------------------------------------------------------------------------
  integer i;
  reg [7:0]  tx_cnt = 8'h00;
  reg [6:0]  burst_cnt = 7'h00;
  reg data_phase = 1'b0;

  always @(*) begin
    // pack DATA_PATH_WIDTH consecutive counter values into the beat
    for (i = 0; i < P; i = i + 1) begin
      tx_char[i*8 +: 8] = tx_cnt + i[7:0];
    end
    tx_charisk = {P{1'b0}};
  end

  // Bursty payload: 32 data beats followed by a 32-beat idle gap.  The idle gap
  // is a run of /K28.5/ commas long enough that the RX decodes fully-idle beats
  // (both symbols comma) which the wrapper filters out - this leaves the RX
  // elastic buffer real slack to absorb application back-pressure.  When
  // tx_valid is de-asserted the wrapper inserts idle commas on the line.
  always @(posedge clk) begin
    if (reset) begin
      tx_valid  <= 1'b0;
      tx_cnt    <= 8'h00;
      burst_cnt <= 7'h00;
    end else if (data_phase) begin
      burst_cnt <= burst_cnt + 1'b1;
      tx_valid  <= ~burst_cnt[5];        // 32 beats valid, 32 beats idle
      if (tx_valid && tx_ready) begin
        tx_cnt <= tx_cnt + P[7:0];
      end
    end else begin
      tx_valid <= 1'b0;   // idle: wrapper inserts commas for alignment
    end
  end

  // --------------------------------------------------------------------------
  // RX checker : random back-pressure + monotonic sequence verification.
  // --------------------------------------------------------------------------
  integer   rx_count = 0;
  integer   idle_count = 0;
  reg       seeded = 1'b0;
  reg [7:0] expect_char = 8'h00;
  reg [7:0] sym;
  reg [31:0] rx_lfsr = 32'h1357BDF0;

  // Bounded back-pressure generator: the serdes RX is free-running, so the
  // elastic buffer only absorbs SHORT stalls - the application must sustain the
  // line rate on average.  We stall for at most 6 cycles and stay ready for at
  // least 8 cycles between stalls, which keeps the depth-16 FIFO from
  // overflowing while still exercising the ready/valid handshake.
  integer stall_cnt = 0;
  integer since_stall = 0;

  always @(posedge clk) begin
    if (reset) begin
      rx_ready    <= 1'b0;
      rx_lfsr     <= 32'h1357BDF0;
      stall_cnt   <= 0;
      since_stall <= 0;
    end else begin
      rx_lfsr <= {rx_lfsr[30:0], rx_lfsr[31]^rx_lfsr[20]^rx_lfsr[2]^rx_lfsr[0]};
      if (stall_cnt > 0) begin
        rx_ready    <= 1'b0;
        stall_cnt   <= stall_cnt - 1;
        since_stall <= 0;
      end else if (since_stall >= 16 && rx_lfsr[1:0] == 2'b00) begin
        rx_ready    <= 1'b0;
        stall_cnt   <= 1 + (rx_lfsr[4:3] % 4);   // 1..4 cycles
        since_stall <= 0;
      end else begin
        rx_ready    <= 1'b1;
        since_stall <= since_stall + 1;
      end
    end
  end

  // NOTE: blocking assignments are used deliberately so the accumulators are
  // updated sequentially across the DATA_PATH_WIDTH symbols within one beat.
  always @(posedge clk) begin
    if (!reset && rx_overflow) begin
      $display("[%0t] ERROR: RX elastic buffer overflow", $time);
      failed = failed + 1;
    end

    if (!reset && rx_valid && rx_ready) begin
      // unpack DATA_PATH_WIDTH characters from the beat and verify each in order
      for (i = 0; i < P; i = i + 1) begin
        sym = rx_char[i*8 +: 8];
        if (rx_notintable[i] || rx_disperr[i]) begin
          $display("[%0t] ERROR: 8b10b error pos %0d nit=%b disperr=%b char=%02x",
                   $time, i, rx_notintable[i], rx_disperr[i], sym);
          failed = failed + 1;
        end else if (rx_charisk[i]) begin
          // control symbol: the only one the wrapper ever inserts is /K28.5/.
          // With sub-beat serdes bit-slip an inserted idle can appear as a
          // single comma symbol inside a data beat - just skip it.
          if (sym !== 8'hBC) begin
            $display("[%0t] ERROR: unexpected control symbol %02x pos %0d",
                     $time, sym, i);
            failed = failed + 1;
          end
          idle_count = idle_count + 1;
        end else if (!seeded) begin
          seeded      = 1'b1;
          expect_char = sym + 8'd1;
          rx_count    = rx_count + 1;
        end else begin
          if (sym !== expect_char) begin
            $display("[%0t] ERROR: seq mismatch got %02x expected %02x (count %0d)",
                     $time, sym, expect_char, rx_count);
            failed = failed + 1;
          end
          expect_char = expect_char + 8'd1;
          rx_count    = rx_count + 1;
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Test sequence
  // --------------------------------------------------------------------------
  initial begin
    $dumpfile(VCD_FILE);
    $dumpvars(0, soft_pcs_wrapper_tb);

    // reset
    repeat (8) @(posedge clk);
    reset <= 1'b0;

    // Phase 1: alignment (wrapper streams commas while aligner locks)
    rx_patternalign_en <= 1'b1;
    rx_enable          <= 1'b0;
    repeat (DATA_PATH_WIDTH*100) @(posedge clk);

    // settle: freeze alignment, let the decode pipeline flush aligned commas
    rx_patternalign_en <= 1'b0;
    repeat (32) @(posedge clk);

    // Phase 2: enable capture and start the payload stream
    rx_enable  <= 1'b1;
    data_phase <= 1'b1;

    // wait until enough payload characters have been checked
    while (rx_count < NUM_PAYLOAD && failed == 0) @(posedge clk);

    // stop generating and drain
    data_phase <= 1'b0;
    repeat (64) @(posedge clk);

    if (failed == 0 && rx_count >= NUM_PAYLOAD)
      $display("SUCCESS - %0d payload characters verified (bitshift=%0d)",
               rx_count, BITSHIFT);
    else
      $display("FAILED - failed=%0d rx_count=%0d", failed, rx_count);
    $finish;
  end

  // watchdog
  initial begin
    #2000000;
    $display("FAILED - timeout (rx_count=%0d)", rx_count);
    $finish;
  end

endmodule
