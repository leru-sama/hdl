// ***************************************************************************
// JESD204 PCS with Link Training and Frame-Align Error Detection
// Complete PCS with CGS, ILAS, and data phase support
// ***************************************************************************
// Copyright (C) 2024 Analog Devices, Inc. All rights reserved.
// SPDX short identifier: ADIJESD204
// ***************************************************************************
//
// Top-level wrapper integrating:
//   - TX: scrambler, char-replace (/A/ /F/), 8b10b encoder
//   - RX: 8b10b decoder, descrambler, frame-align monitor,
//         per-lane elastic buffer (lane de-skew), CGS detect
//   - Link training FSM: CGS -> ILAS -> DATA
//   - Frame-align error detection -> auto resync (ENABLE_FRAME_ALIGN_ERR_RESET)
//
// ===== Signal Reference =====
//
// -- Clock / Reset --
//   clk                  : Single clock domain for both TX and RX datapaths.
//                          In silicon this would typically be the device clock;
//                          in simulation both sides share this clock and the
//                          serdes model injects per-lane skew.
//   reset                 : Synchronous active-high reset.
//
// -- Configuration --
//   cfg_octets_per_multiframe [9:0] : Number of octets in one multiframe (K).
//                          Must be a multiple of DATA_PATH_WIDTH. Typical: 32.
//   cfg_octets_per_frame      [7:0] : Number of octets in one frame (F).
//                          Supported values depend on DATA_PATH_WIDTH.
//                          For DPW=8: F=8 is a common choice.
//   cfg_lanes_disable  [NUM_LANES-1:0] : Per-lane disable (1 = lane disabled).
//   cfg_links_disable  [NUM_LINKS-1:0] : Per-link disable (1 = link disabled).
//   cfg_disable_scrambler          : 1 = bypass scrambler/descrambler.
//   cfg_disable_char_replacement   : 1 = do not insert /A/ /F/ alignment chars.
//   cfg_mframes_per_ilas       [7:0] : Number of multiframes in one ILAS sequence (M).
//   cfg_skip_ilas                   : 1 = skip ILAS, go straight from CGS to DATA.
//   cfg_continuous_cgs              : 1 = stay in CGS (used for initial bring-up).
//   cfg_continuous_ilas             : 1 = repeat ILAS continuously.
//
// -- TX Application Interface (ready/valid handshake) --
//   tx_valid                    : Application data valid strobe.
//   tx_ready                    : PCS ready to accept tx_data (during DATA phase).
//   tx_data [DATA_PATH_WIDTH*8*NUM_LANES-1:0] : Application payload, packed as
//                          [lane0_char0, lane0_char1, ..., lane3_char7].
//   tx_charisk  [DATA_PATH_WIDTH*NUM_LANES-1:0] : Per-character control flag.
//                          1 = this 8b character is a control character (Kx.y).
//
// -- RX Application Interface (ready/valid handshake) --
//   rx_valid                    : rx_data / rx_charisk are valid this cycle.
//   rx_ready                    : Application is ready to consume RX data.
//   rx_data [DATA_PATH_WIDTH*8*NUM_LANES-1:0] : Decoded application payload.
//   rx_charisk  [DATA_PATH_WIDTH*NUM_LANES-1:0] : Per-character control flag.
//   rx_notintable [DATA_PATH_WIDTH*NUM_LANES-1:0] : 8b10b not-in-table error per char.
//   rx_disperr    [DATA_PATH_WIDTH*NUM_LANES-1:0] : 8b10b disparity error per char.
//
// -- Serdes Interface --
//   serdes_tx_data [DATA_PATH_WIDTH*10*NUM_LANES-1:0] : Parallel 8b10b-encoded
//                          symbols to the serdes TX. Packed per lane:
//                          [lane0_sym0, lane0_sym1, ..., lane3_sym7].
//                          Each symbol is 10 bits wide.
//                          = DATA_PATH_WIDTH * 10 bits per lane.
//   serdes_rx_data [DATA_PATH_WIDTH*10*NUM_LANES-1:0] : Parallel 8b10b symbols
//                          from the serdes RX (CDR-recovered clock domain).
//                          Same packing as serdes_tx_data.
//
// -- Link Control / Status (between two PCS instances) --
//   sync_request_n [NUM_LINKS-1:0] : From the remote RX device. ACTIVE-LOW.
//                          Driven by the RX's rx_ctrl: low = "please send CGS".
//                          This is the input that triggers the TX state machine
//                          to leave WAIT and enter CGS -> ILAS -> DATA.
//   status_ctrl_state        [1:0] : Current TX state machine state.
//                          2'b00 = RESET, 2'b01 = CGS, 2'b10 = ILAS, 2'b11 = DATA.
//   status_lane_cgs_state [2*NUM_LANES-1:0] : Per-lane CGS detector state (2 bits each).
//   status_lane_ifs_ready  [NUM_LANES-1:0] : Per-lane ILAS monitor IFS ready.
//   sync_n            [NUM_LINKS-1:0] : To the remote TX device. ACTIVE-LOW.
//                          Driven by the local RX's rx_ctrl: low = "I am in CGS,
//                          please keep sending K28.5". Registered for CDC.
//
// -- ILAS Configuration (from RX ILAS monitor to upper layer) --
//   ilas_config_valid  [NUM_LANES-1:0] : One-hot: lane i has valid ILAS config.
//   ilas_config_addr   [NUM_LANES*2-1:0] : 2-bit config address per lane
//                          (0=R/ADJCNT, 1=Q/ADJDIR, 2=F/MF, 3=K/CS).
//   ilas_config_data   [NUM_LANES*DATA_PATH_WIDTH*8-1:0] : Config data per lane.
//
// -- Error Status --
//   status_err_statistics_cnt [32*NUM_LANES-1:0] : 32-bit error counter per lane.
//                          Increments on 8b10b not-in-table / disparity errors.
//                          Reset on each event_data_phase (start of DATA).
//   status_frame_align_err_cnt_0 [NUM_LANES-1:1:0] : Frame-align error counter
//                          for lane 0 (8 bits). Mirror of the internal per-lane
//                          frame_align_err_cnt[i] for the first lane only;
//                          use this for quick debug visibility.
//   event_frame_alignment_error           : Sticky OR of per-lane
//                          frame_align_err_thresh_met. Pulses high when any
//                          enabled lane has accumulated >= FRAME_ALIGN_ERR_THRESHOLD
//                          frame-align errors and ENABLE_FRAME_ALIGN_ERR_RESET=1.
//                          This triggers rx_ctrl to drop back to RESET/CGS.
//
// ===== Parameters =====
//   NUM_LANES                  : Number of serdes lanes (default 1).
//   NUM_LINKS                  : Number of JESD204 links (default 1).
//   DATA_PATH_WIDTH            : 8b symbols per clock per lane.
//                               8 = 80b/lane serdes interface (default, 256b total).
//                               4 = 40b/lane.  2 = 20b/lane.
//   ENABLE_FRAME_ALIGN_CHECK   : 1 = instantiate rx_frame_align to monitor /A/ /F/
//                                positioning (default 1).
//   ENABLE_CHAR_REPLACE        : 1 = TX inserts /A/ at EOMF, /F/ at EOF
//                                via jesd204_frame_align_replace (default 1).
//   ENABLE_FRAME_ALIGN_ERR_RESET: 1 = rx_ctrl resets to CGS when frame-align
//                                error threshold is crossed (default 1).
//   FRAME_ALIGN_ERR_THRESHOLD  : Error count before asserting resync (default 16).
//   ELASTIC_BUFFER_SIZE        : Per-lane elastic buffer depth in bits (default 256).
//
`timescale 1ns/100ps

module jesd204_pcs_link_training #(
  parameter NUM_LANES = 1,
  parameter NUM_LINKS = 1,
  parameter DATA_PATH_WIDTH = 8,  // 8 for 80-bit serdes (64B per lane per cycle at 4 lanes = 256B total)
  parameter ENABLE_FRAME_ALIGN_CHECK = 1,
  parameter ENABLE_CHAR_REPLACE = 1,
  parameter ENABLE_FRAME_ALIGN_ERR_RESET = 1,
  parameter FRAME_ALIGN_ERR_THRESHOLD = 8'd16,
  parameter ELASTIC_BUFFER_SIZE = 256
) (
  input clk,
  input reset,

  input [9:0] cfg_octets_per_multiframe,
  input [7:0] cfg_octets_per_frame,
  input [NUM_LANES-1:0] cfg_lanes_disable,
  input [NUM_LINKS-1:0] cfg_links_disable,
  input cfg_disable_scrambler,
  input cfg_disable_char_replacement,
  input [7:0] cfg_mframes_per_ilas,
  input cfg_skip_ilas,
  input cfg_continuous_cgs,
  input cfg_continuous_ilas,

  input tx_valid,
  output tx_ready,
  input [DATA_PATH_WIDTH*8*NUM_LANES-1:0] tx_data,
  input [DATA_PATH_WIDTH*NUM_LANES-1:0] tx_charisk,

  output rx_valid,
  input rx_ready,
  output [DATA_PATH_WIDTH*8*NUM_LANES-1:0] rx_data,
  output [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_charisk,
  output [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_notintable,
  output [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_disperr,

  output [DATA_PATH_WIDTH*10*NUM_LANES-1:0] serdes_tx_data,
  input [DATA_PATH_WIDTH*10*NUM_LANES-1:0] serdes_rx_data,

  input [NUM_LINKS-1:0] sync_request_n,
  output [1:0] status_ctrl_state,
  output [2*NUM_LANES-1:0] status_lane_cgs_state,
  output [NUM_LANES-1:0] status_lane_ifs_ready,
  output [NUM_LINKS-1:0] sync_n,

  output [NUM_LANES-1:0] ilas_config_valid,
  output [NUM_LANES*2-1:0] ilas_config_addr,
  output [NUM_LANES*DATA_PATH_WIDTH*8-1:0] ilas_config_data,

  output [32*NUM_LANES-1:0] status_err_statistics_cnt,
  output [NUM_LANES-1:0] status_frame_align_err_cnt_0,
  output event_frame_alignment_error
);

  localparam DPW_LOG2 = DATA_PATH_WIDTH == 8 ? 3 : DATA_PATH_WIDTH == 4 ? 2 : 1;

  wire [DATA_PATH_WIDTH*8*NUM_LANES-1:0] tx_phy_data;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] tx_phy_charisk;
  wire [NUM_LANES-1:0] lane_cgs_enable;
  wire tx_ctrl_ready;
  wire [DATA_PATH_WIDTH*8*NUM_LANES-1:0] ilas_data;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] ilas_charisk;
  wire [1:0] ilas_config_addr_tx;
  wire ilas_config_rd;
  wire [DATA_PATH_WIDTH*8*NUM_LANES-1:0] ilas_config_data_tx;
  wire eof_reset;
  wire tx_ready_nx;
  wire tx_next_mf_ready;

  wire [DATA_PATH_WIDTH*8*NUM_LANES-1:0] rx_phy_data;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_phy_charisk;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_phy_notintable;
  wire [DATA_PATH_WIDTH*NUM_LANES-1:0] rx_phy_disperr;
  wire [NUM_LANES-1:0] cgs_reset;
  wire [NUM_LANES-1:0] cgs_ready;
  wire [NUM_LANES-1:0] ifs_reset;
  wire phy_en_char_align;
  wire [NUM_LINKS-1:0] sync_n_int;
  wire latency_monitor_reset;
  wire event_data_phase;
  wire lmfc_edge;

  wire [DATA_PATH_WIDTH-1:0] sof, eof, somf, eomf;

  reg  [NUM_LANES-1:0] frame_align_err_thresh_met;
  wire [7:0] frame_align_err_cnt_w [0:NUM_LANES-1];
  reg  [7:0] frame_align_err_cnt   [0:NUM_LANES-1];

  // ---------------------------------------------------------------
  // LMFC
  // ---------------------------------------------------------------
  jesd204_lmfc #(
    .LINK_MODE(1),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH)
  ) i_lmfc (
    .clk(clk),
    .reset(reset),
    .sysref(1'b0),
    .cfg_octets_per_multiframe(cfg_octets_per_multiframe),
    .cfg_beats_per_multiframe(cfg_octets_per_multiframe[9:DPW_LOG2]),
    .cfg_lmfc_offset(8'd0),
    .cfg_sysref_oneshot(1'b0),
    .cfg_sysref_disable(1'b1),
    .lmfc_edge(lmfc_edge),
    .lmfc_clk(),
    .lmfc_counter(),
    .lmc_edge(),
    .lmc_quarter_edge(),
    .eoemb(),
    .sysref_edge(),
    .sysref_alignment_error());

  // ---------------------------------------------------------------
  // Internal signals
  // ---------------------------------------------------------------
  //
  // -- TX internal --
  // tx_phy_data / tx_phy_charisk  : Post-scramble, pre-8b10b-encode data.
  // lane_cgs_enable               : Per-lane CGS enable (from tx_ctrl).
  // tx_ctrl_ready                 : TX ready to accept app data (= tx_ready).
  // ilas_data / ilas_charisk      : ILAS payload from tx_ctrl to tx_lane.
  // ilas_config_addr_tx           : ILAS config read address from tx_ctrl.
  // ilas_config_rd                : ILAS config read strobe from tx_ctrl.
  // ilas_config_data_tx           : ILAS config data read by tx_ctrl (from app).
  // eof_reset                     : EOF reset pulse (resets ILAS/tx_ctrl state).
  // tx_ready_nx                   : Next-cycle tx_ready (pipeline).
  // tx_next_mf_ready              : TX ready at next multiframe boundary.
  //
  // -- RX internal --
  // rx_phy_data / rx_phy_charisk  : Post-8b10b-decode, pre-descramble data.
  // rx_phy_notintable / rx_phy_disperr : 8b10b decode error flags per char.
  // cgs_reset                     : Per-lane CGS reset (from rx_ctrl).
  // cgs_ready                     : Per-lane CGS detected (from rx_cgs).
  // ifs_reset                     : Per-lane IFS (inter-frame sync) reset.
  // phy_en_char_align             : Enable char-align mux in rx_lane.
  // sync_n_int                    : Internal sync_n before CDC register.
  // latency_monitor_reset         : Reset latency monitor in rx_ctrl.
  // event_data_phase              : Pulse on CGS->SYNCHRONIZED transition.
  // lmfc_edge                     : Local multi-frame clock edge (from lmfc).
  //
  // -- Frame marking --
  // sof / eof                     : Start/end of frame per DPW position.
  // somf / eomf                   : Start/end of multiframe per DPW position.
  //
  // -- Frame-align error monitoring --
  // frame_align_err_thresh_met    : Per-lane flag: error count >= threshold.
  // frame_align_err_cnt_w         : Raw 8-bit error counter from rx_lane.
  // frame_align_err_cnt           : Registered copy of the error counter.
  jesd204_frame_mark #(
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH)
  ) i_frame_mark (
    .clk(clk),
    .reset(eof_reset),
    .cfg_octets_per_multiframe(cfg_octets_per_multiframe),
    .cfg_beats_per_multiframe(cfg_octets_per_multiframe[9:DPW_LOG2]),
    .cfg_octets_per_frame(cfg_octets_per_frame),
    .sof(sof),
    .eof(eof),
    .somf(somf),
    .eomf(eomf));

  // ---------------------------------------------------------------
  // TX Control State Machine
  // ---------------------------------------------------------------
  jesd204_tx_ctrl #(
    .NUM_LANES(NUM_LANES),
    .NUM_LINKS(NUM_LINKS),
    .DATA_PATH_WIDTH(DATA_PATH_WIDTH)
  ) i_tx_ctrl (
    .clk(clk),
    .reset(reset),
    .sync(sync_request_n),
    .lmfc_edge(lmfc_edge),
    .somf(somf),
    .somf_early2(somf),
    .eomf(eomf),
    .lane_cgs_enable(lane_cgs_enable),
    .eof_reset(eof_reset),
    .tx_ready(tx_ctrl_ready),
    .tx_ready_nx(tx_ready_nx),
    .tx_next_mf_ready(tx_next_mf_ready),
    .ilas_data(ilas_data),
    .ilas_charisk(ilas_charisk),
    .ilas_config_addr(ilas_config_addr_tx),
    .ilas_config_rd(ilas_config_rd),
    .ilas_config_data(ilas_config_data_tx),
    .cfg_lanes_disable(cfg_lanes_disable),
    .cfg_links_disable(cfg_links_disable),
    .cfg_continuous_cgs(cfg_continuous_cgs),
    .cfg_continuous_ilas(cfg_continuous_ilas),
    .cfg_skip_ilas(cfg_skip_ilas),
    .cfg_mframes_per_ilas(cfg_mframes_per_ilas),
    .cfg_octets_per_multiframe(cfg_octets_per_multiframe),
    .ctrl_manual_sync_request(1'b0),
    .status_sync(),
    .status_state(status_ctrl_state));

  assign tx_ready = tx_ctrl_ready;

  // ---------------------------------------------------------------
  // TX Lane
  // ---------------------------------------------------------------
  genvar i;
  generate
    for (i = 0; i < NUM_LANES; i = i + 1) begin : gen_tx_lane
      localparam D_START = i * DATA_PATH_WIDTH * 8;
      localparam D_STOP = D_START + DATA_PATH_WIDTH * 8 - 1;
      localparam C_START = i * DATA_PATH_WIDTH;
      localparam C_STOP = C_START + DATA_PATH_WIDTH - 1;

      jesd204_tx_lane #(
        .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
        .ENABLE_CHAR_REPLACE(ENABLE_CHAR_REPLACE)
      ) i_tx_lane (
        .clk(clk),
        .eof(eof),
        .eomf(eomf),
        .cgs_enable(lane_cgs_enable[i]),
        .ilas_data(ilas_data[D_STOP:D_START]),
        .ilas_charisk(ilas_charisk[C_STOP:C_START]),
        .tx_data(tx_data[D_STOP:D_START]),
        .tx_ready(tx_ctrl_ready),
        .phy_data(tx_phy_data[D_STOP:D_START]),
        .phy_charisk(tx_phy_charisk[C_STOP:C_START]),
        .cfg_octets_per_frame(cfg_octets_per_frame),
        .cfg_disable_char_replacement(cfg_disable_char_replacement),
        .cfg_disable_scrambler(cfg_disable_scrambler));
    end
  endgenerate

  // ---------------------------------------------------------------
  // 8b10b Encoder (TX)
  // ---------------------------------------------------------------
  generate
    for (i = 0; i < NUM_LANES; i = i + 1) begin : gen_tx_encoder
      wire [DATA_PATH_WIDTH:0] disparity_chain;
      reg disparity_reg = 1'b0;

      assign disparity_chain[0] = disparity_reg;

      always @(posedge clk) begin
        if (reset) begin
          disparity_reg <= 1'b0;
        end else begin
          disparity_reg <= disparity_chain[DATA_PATH_WIDTH];
        end
      end

      genvar j;
      for (j = 0; j < DATA_PATH_WIDTH; j = j + 1) begin : gen_enc_char
        localparam CHAR_START = (i * DATA_PATH_WIDTH + j) * 8;
        localparam CHAR_STOP = CHAR_START + 7;
        localparam OUT_START = (i * DATA_PATH_WIDTH + j) * 10;
        localparam OUT_STOP = OUT_START + 9;

        jesd204_8b10b_encoder i_tx_encoder (
          .in_disparity(disparity_chain[j]),
          .in_char(tx_phy_data[CHAR_STOP:CHAR_START]),
          .in_charisk(tx_phy_charisk[i * DATA_PATH_WIDTH + j]),
          .out_char(serdes_tx_data[OUT_STOP:OUT_START]),
          .out_disparity(disparity_chain[j+1]));
      end
    end
  endgenerate

  // ---------------------------------------------------------------
  // 8b10b Decoder (RX)
  // ---------------------------------------------------------------
  generate
    for (i = 0; i < NUM_LANES; i = i + 1) begin : gen_rx_decoder
      wire [DATA_PATH_WIDTH:0] disparity_chain_rx;
      reg disparity_reg_rx = 1'b0;

      assign disparity_chain_rx[0] = disparity_reg_rx;

      always @(posedge clk) begin
        if (reset) begin
          disparity_reg_rx <= 1'b0;
        end else begin
          disparity_reg_rx <= disparity_chain_rx[DATA_PATH_WIDTH];
        end
      end

      genvar j;
      for (j = 0; j < DATA_PATH_WIDTH; j = j + 1) begin : gen_dec_char
        localparam IN_START = (i * DATA_PATH_WIDTH + j) * 10;
        localparam IN_STOP = IN_START + 9;
        localparam CHAR_START = (i * DATA_PATH_WIDTH + j) * 8;
        localparam CHAR_STOP = CHAR_START + 7;

        jesd204_8b10b_decoder i_rx_decoder (
          .in_disparity(disparity_chain_rx[j]),
          .in_char(serdes_rx_data[IN_STOP:IN_START]),
          .out_char(rx_phy_data[CHAR_STOP:CHAR_START]),
          .out_charisk(rx_phy_charisk[i * DATA_PATH_WIDTH + j]),
          .out_notintable(rx_phy_notintable[i * DATA_PATH_WIDTH + j]),
          .out_disperr(rx_phy_disperr[i * DATA_PATH_WIDTH + j]),
          .out_disparity(disparity_chain_rx[j+1]));
      end
    end
  endgenerate

  // ---------------------------------------------------------------
  // Frame-align error capture (from rx_lane outputs)
  // ---------------------------------------------------------------
  generate
    for (i = 0; i < NUM_LANES; i = i + 1) begin : gen_cap_err_cnt
      always @(posedge clk) begin
        if (reset)
          frame_align_err_cnt[i] <= 8'd0;
        else
          frame_align_err_cnt[i] <= frame_align_err_cnt_w[i];
      end
    end
  endgenerate

  // ---------------------------------------------------------------
  // Frame-align error threshold monitoring
  // ---------------------------------------------------------------
  generate
    for (i = 0; i < NUM_LANES; i = i + 1) begin : gen_frame_align_mon
      always @(posedge clk) begin
        if (reset) begin
          frame_align_err_thresh_met[i] <= 1'b0;
        end else if (ENABLE_FRAME_ALIGN_ERR_RESET) begin
          if (frame_align_err_cnt[i] >= FRAME_ALIGN_ERR_THRESHOLD)
            frame_align_err_thresh_met[i] <= 1'b1;
          else if (cgs_ready[i])
            frame_align_err_thresh_met[i] <= 1'b0;
        end else begin
          frame_align_err_thresh_met[i] <= 1'b0;
        end
      end
    end
  endgenerate

  assign event_frame_alignment_error = |frame_align_err_thresh_met;

  // ---------------------------------------------------------------
  // RX Control State Machine
  // ---------------------------------------------------------------
  jesd204_rx_ctrl #(
    .NUM_LANES(NUM_LANES),
    .NUM_LINKS(NUM_LINKS),
    .ENABLE_FRAME_ALIGN_ERR_RESET(ENABLE_FRAME_ALIGN_ERR_RESET)
  ) i_rx_ctrl (
    .clk(clk),
    .reset(reset),
    .cfg_lanes_disable(cfg_lanes_disable),
    .cfg_links_disable(cfg_links_disable),
    .phy_ready(1'b1),
    .phy_en_char_align(phy_en_char_align),
    .cgs_reset(cgs_reset),
    .cgs_ready(cgs_ready),
    .ifs_reset(ifs_reset),
    .lmfc_edge(lmfc_edge),
    .frame_align_err_thresh_met(frame_align_err_thresh_met),
    .sync(sync_n_int),
    .latency_monitor_reset(latency_monitor_reset),
    .status_state(),
    .event_data_phase(event_data_phase));

  reg sync_n_reg = 1'b1;
  always @(posedge clk) begin
    sync_n_reg <= sync_n_int;
  end
  assign sync_n = sync_n_reg;

  // ---------------------------------------------------------------
  // RX Lane
  // ---------------------------------------------------------------
  wire [NUM_LANES-1:0] buffer_ready_n;
  wire all_buffer_ready_n;
  reg buffer_release_n = 1'b1;
  reg buffer_release_opportunity = 1'b0;
  reg [7:0] lmfc_counter_reg = 8'd0;

  always @(posedge clk) begin
    if (reset) begin
      lmfc_counter_reg <= 8'd0;
    end else if (lmfc_edge) begin
      lmfc_counter_reg <= 8'd0;
    end else begin
      lmfc_counter_reg <= lmfc_counter_reg + 1'b1;
    end
  end

  always @(posedge clk) begin
    if (lmfc_counter_reg == 8'd2) begin
      buffer_release_opportunity <= 1'b1;
    end else begin
      buffer_release_opportunity <= 1'b0;
    end
  end

  assign all_buffer_ready_n = |buffer_ready_n;

  reg [3:0] release_delay = 0;
  always @(posedge clk) begin
    if (reset) begin
      buffer_release_n <= 1'b1;
      release_delay <= 0;
    end else if (!all_buffer_ready_n && buffer_release_opportunity) begin
      if (release_delay == 4'd8) begin
        buffer_release_n <= 1'b0;
      end else begin
        release_delay <= release_delay + 1'b1;
      end
    end else if (all_buffer_ready_n) begin
      release_delay <= 0;
    end
  end

  generate
    for (i = 0; i < NUM_LANES; i = i + 1) begin : gen_rx_lane
      localparam D_START = i * DATA_PATH_WIDTH * 8;
      localparam D_STOP = D_START + DATA_PATH_WIDTH * 8 - 1;
      localparam C_START = i * DATA_PATH_WIDTH;
      localparam C_STOP = C_START + DATA_PATH_WIDTH - 1;

      jesd204_rx_lane #(
        .DATA_PATH_WIDTH(DATA_PATH_WIDTH),
        .TPL_DATA_PATH_WIDTH(DATA_PATH_WIDTH),
        .CHAR_INFO_REGISTERED(0),
        .ALIGN_MUX_REGISTERED(1),
        .SCRAMBLER_REGISTERED(0),
        .ELASTIC_BUFFER_SIZE(ELASTIC_BUFFER_SIZE),
        .ENABLE_FRAME_ALIGN_CHECK(ENABLE_FRAME_ALIGN_CHECK),
        .ENABLE_CHAR_REPLACE(ENABLE_CHAR_REPLACE),
        .ASYNC_CLK(0)
      ) i_rx_lane (
        .clk(clk),
        .reset(reset),
        .device_clk(clk),
        .device_reset(reset),
        .phy_data(rx_phy_data[D_STOP:D_START]),
        .phy_charisk(rx_phy_charisk[C_STOP:C_START]),
        .phy_notintable(rx_phy_notintable[C_STOP:C_START]),
        .phy_disperr(rx_phy_disperr[C_STOP:C_START]),
        .cgs_reset(cgs_reset[i]),
        .cgs_ready(cgs_ready[i]),
        .ifs_reset(ifs_reset[i]),
        .rx_data(rx_data[D_STOP:D_START]),
        .buffer_ready_n(buffer_ready_n[i]),
        .buffer_release_n(buffer_release_n),
        .cfg_octets_per_multiframe(cfg_octets_per_multiframe),
        .cfg_octets_per_frame(cfg_octets_per_frame),
        .cfg_disable_char_replacement(cfg_disable_char_replacement),
        .cfg_disable_scrambler(cfg_disable_scrambler),
        .err_statistics_reset(event_data_phase),
        .ctrl_err_statistics_mask(3'b000),
        .status_err_statistics_cnt(status_err_statistics_cnt[32*i+31:32*i]),
        .ilas_config_valid(ilas_config_valid[i]),
        .ilas_config_addr(ilas_config_addr[2*i+1:2*i]),
        .ilas_config_data(ilas_config_data[D_STOP:D_START]),
        .status_cgs_state(status_lane_cgs_state[2*i+1:2*i]),
        .status_ifs_ready(status_lane_ifs_ready[i]),
        .status_frame_align(),
        .status_frame_align_err_cnt(frame_align_err_cnt_w[i]));
    end
  endgenerate

  assign status_frame_align_err_cnt_0 = frame_align_err_cnt[0];

  assign rx_valid = ~buffer_release_n;
  assign rx_charisk = rx_phy_charisk;
  assign rx_notintable = rx_phy_notintable;
  assign rx_disperr = rx_phy_disperr;

endmodule
