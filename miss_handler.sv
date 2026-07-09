// ============================================================
//  miss_handler.sv
//  Cache Miss Handler & Writeback Pump
//
//  Project : AXI4 Memory Controller with Adaptive Cache
//  Author  : Jatin Wadhera (CPG-6, TIET)
//
//  Description:
//    On a cache miss this module:
//      1. Issues an AXI4 INCR burst READ to external memory
//         to fill one full cache line (WORDS_PER_LINE beats)
//      2. Streams fill data back to cache_ctrl word-by-word
//    On a writeback request:
//      1. Issues an AXI4 INCR burst WRITE (dirty evicted line)
//      2. Awaits BRESP OKAY before acknowledging
//
//  AXI4 burst formula:
//    AWLEN/ARLEN = WORDS_PER_LINE - 1  (burst length = 16 → AWLEN=15)
//    AWSIZE/ARSIZE = $clog2(DATA_WIDTH/8)  (4 bytes → AWSIZE=2)
//    AWBURST/ARBURST = 2'b01  (INCR)
// ============================================================

`timescale 1ns/1ps

module miss_handler #(
    parameter DATA_WIDTH    = 32,
    parameter ADDR_WIDTH    = 32,
    parameter ID_WIDTH      = 4,
    parameter LINE_BYTES    = 64,
    parameter WORDS_PER_LINE= LINE_BYTES / (DATA_WIDTH/8)   // = 16
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ── From Cache Controller ────────────────────────────────
    input  logic                    miss_req,
    input  logic [ADDR_WIDTH-1:0]   miss_addr,
    input  logic [ID_WIDTH-1:0]     miss_id,
    input  logic                    miss_is_write,
    input  logic [DATA_WIDTH-1:0]   miss_wr_data,
    input  logic [DATA_WIDTH/8-1:0] miss_wr_strb,
    output logic                    miss_ack,

    // ── Line fill → Cache Controller ────────────────────────
    output logic [DATA_WIDTH-1:0]   fill_data,
    output logic                    fill_valid,
    output logic                    fill_last,
    output logic [ADDR_WIDTH-1:0]   fill_addr,

    // ── Writeback from Cache Controller ─────────────────────
    input  logic                    wb_req,
    input  logic [ADDR_WIDTH-1:0]   wb_addr,
    input  logic [DATA_WIDTH-1:0]   wb_data,
    input  logic                    wb_last,
    output logic                    wb_ack,

    // ── AXI4 Master port to External Memory ─────────────────
    // AW channel
    output logic [ID_WIDTH-1:0]     M_AWID,
    output logic [ADDR_WIDTH-1:0]   M_AWADDR,
    output logic [7:0]              M_AWLEN,
    output logic [2:0]              M_AWSIZE,
    output logic [1:0]              M_AWBURST,
    output logic                    M_AWVALID,
    input  logic                    M_AWREADY,

    // W channel
    output logic [DATA_WIDTH-1:0]   M_WDATA,
    output logic [DATA_WIDTH/8-1:0] M_WSTRB,
    output logic                    M_WLAST,
    output logic                    M_WVALID,
    input  logic                    M_WREADY,

    // B channel
    input  logic [ID_WIDTH-1:0]     M_BID,
    input  logic [1:0]              M_BRESP,
    input  logic                    M_BVALID,
    output logic                    M_BREADY,

    // AR channel
    output logic [ID_WIDTH-1:0]     M_ARID,
    output logic [ADDR_WIDTH-1:0]   M_ARADDR,
    output logic [7:0]              M_ARLEN,
    output logic [2:0]              M_ARSIZE,
    output logic [1:0]              M_ARBURST,
    output logic                    M_ARVALID,
    input  logic                    M_ARREADY,

    // R channel
    input  logic [ID_WIDTH-1:0]     M_RID,
    input  logic [DATA_WIDTH-1:0]   M_RDATA,
    input  logic [1:0]              M_RRESP,
    input  logic                    M_RLAST,
    input  logic                    M_RVALID,
    output logic                    M_RREADY
);

// ============================================================
//  Localparams
// ============================================================
localparam BEAT_BITS  = $clog2(WORDS_PER_LINE);
localparam AXI_SIZE   = $clog2(DATA_WIDTH/8);   // 2 for 32-bit
localparam BURST_INCR = 2'b01;

// Align address to cache line boundary
function automatic logic [ADDR_WIDTH-1:0] line_addr(input logic [ADDR_WIDTH-1:0] a);
    return {a[ADDR_WIDTH-1 : $clog2(LINE_BYTES)], {$clog2(LINE_BYTES){1'b0}}};
endfunction

// ============================================================
//  FSM
// ============================================================
typedef enum logic [2:0] {
    IDLE       = 3'd0,
    RD_ADDR    = 3'd1,     // issue AR
    RD_DATA    = 3'd2,     // collect R beats
    WB_ADDR    = 3'd3,     // issue AW for writeback
    WB_DATA    = 3'd4,     // send W beats
    WB_RESP    = 3'd5      // await BRESP
} state_t;

state_t state, next;

// Registered request
logic [ADDR_WIDTH-1:0]   req_addr_r;
logic [ID_WIDTH-1:0]     req_id_r;
logic [BEAT_BITS-1:0]    beat_cnt;
logic                    is_wb_r;

// ── Next state ───────────────────────────────────────────────
always_comb begin
    next = state;
    case (state)
        IDLE    : if (wb_req)   next = WB_ADDR;
                  else if (miss_req) next = RD_ADDR;
        RD_ADDR : if (M_ARREADY) next = RD_DATA;
        RD_DATA : if (M_RVALID && M_RLAST) next = IDLE;
        WB_ADDR : if (M_AWREADY) next = WB_DATA;
        WB_DATA : if (M_WVALID && M_WREADY && M_WLAST) next = WB_RESP;
        WB_RESP : if (M_BVALID && M_BREADY)             next = IDLE;
        default : next = IDLE;
    endcase
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= IDLE;
    else        state <= next;
end

// ============================================================
//  Datapath
// ============================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        req_addr_r  <= '0;
        req_id_r    <= '0;
        beat_cnt    <= '0;
        miss_ack    <= 1'b0;
        wb_ack      <= 1'b0;
        fill_valid  <= 1'b0;
        fill_last   <= 1'b0;
        // AXI outputs
        M_AWVALID <= 1'b0;
        M_WVALID  <= 1'b0;
        M_BREADY  <= 1'b0;
        M_ARVALID <= 1'b0;
        M_RREADY  <= 1'b0;
    end else begin
        miss_ack   <= 1'b0;
        wb_ack     <= 1'b0;
        fill_valid <= 1'b0;
        fill_last  <= 1'b0;

        case (state)
            // ── Arbitrate: writeback has priority ───────────
            IDLE: begin
                beat_cnt <= '0;
                if (wb_req) begin
                    req_addr_r <= wb_addr;
                    req_id_r   <= '0;
                    is_wb_r    <= 1'b1;
                end else if (miss_req) begin
                    req_addr_r <= line_addr(miss_addr);
                    req_id_r   <= miss_id;
                    is_wb_r    <= 1'b0;
                    miss_ack   <= 1'b1;
                end
            end

            // ── Issue AR burst for line fill ─────────────────
            RD_ADDR: begin
                M_ARVALID <= 1'b1;
                M_ARID    <= req_id_r;
                M_ARADDR  <= req_addr_r;
                M_ARLEN   <= WORDS_PER_LINE - 1;     // 15
                M_ARSIZE  <= AXI_SIZE[2:0];
                M_ARBURST <= BURST_INCR;
                M_RREADY  <= 1'b1;
                if (M_ARREADY) M_ARVALID <= 1'b0;
            end

            // ── Collect R beats, stream to cache ─────────────
            RD_DATA: begin
                M_RREADY <= 1'b1;
                if (M_RVALID) begin
                    fill_data  <= M_RDATA;
                    fill_valid <= 1'b1;
                    fill_addr  <= req_addr_r;
                    fill_last  <= M_RLAST;
                    beat_cnt   <= beat_cnt + 1;
                end
                if (M_RVALID && M_RLAST) M_RREADY <= 1'b0;
            end

            // ── Issue AW burst for writeback ─────────────────
            WB_ADDR: begin
                M_AWVALID <= 1'b1;
                M_AWID    <= req_id_r;
                M_AWADDR  <= req_addr_r;
                M_AWLEN   <= WORDS_PER_LINE - 1;
                M_AWSIZE  <= AXI_SIZE[2:0];
                M_AWBURST <= BURST_INCR;
                if (M_AWREADY) M_AWVALID <= 1'b0;
            end

            // ── Send W beats from writeback buffer ───────────
            WB_DATA: begin
                M_WVALID <= wb_req;
                M_WDATA  <= wb_data;
                M_WSTRB  <= '1;       // full-word writeback
                M_WLAST  <= wb_last;
                if (M_WREADY && wb_req) begin
                    wb_ack   <= 1'b1;
                    beat_cnt <= beat_cnt + 1;
                end
                if (M_WLAST && M_WREADY) M_WVALID <= 1'b0;
            end

            // ── Await BRESP ───────────────────────────────────
            WB_RESP: begin
                M_BREADY <= 1'b1;
                if (M_BVALID) begin
                    M_BREADY <= 1'b0;
                    beat_cnt <= '0;
                end
            end
        endcase
    end
end

endmodule
