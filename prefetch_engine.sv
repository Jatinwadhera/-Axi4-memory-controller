// ============================================================
//  prefetch_engine.sv
//  Stride-Based Hardware Prefetcher
//
//  Project : AXI4 Memory Controller with Adaptive Cache
//  Author  : Jatin Wadhera (CPG-6, TIET)
//
//  Description:
//    Implements a Reference Prediction Table (RPT) prefetcher.
//    The RPT has RPT_ENTRIES (16) entries, each tracking:
//      - Tag (upper bits of PC / request addr)
//      - Previous address
//      - Stride (delta between last two accesses)
//      - State machine per entry: INIT ? TRANSIENT ? STEADY ? NO-PRED
//
//    On STEADY state, issues a speculative prefetch
//    LOOKAHEAD_DIST lines ahead of the current address.
//
//  State machine per RPT entry:
//    INIT      ? first access,  stride unknown
//    TRANSIENT ? stride detected once, not yet confirmed
//    STEADY    ? stride confirmed, prefetch ENABLED
//    NO_PRED   ? stride changed, suppress prefetch
//
//  This module directly mirrors ML-based bank prediction
//  concepts from the capstone BRAM project - the RPT is
//  essentially a 1-D hardware predictor table.
// ============================================================

`timescale 1ns/1ps

module prefetch_engine #(
    parameter ADDR_WIDTH     = 32,
    parameter ID_WIDTH       = 4,
    parameter RPT_ENTRIES    = 16,         // must be power of 2
    parameter LOOKAHEAD_DIST = 2,          // prefetch N lines ahead
    parameter LINE_BYTES     = 64
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ?? Access stream from cache controller ?????????????????
    // Pulsed on every cache lookup (hit or miss)
    input  logic                    acc_valid,
    input  logic [ADDR_WIDTH-1:0]   acc_addr,

    // ?? Prefetch request ? miss handler ?????????????????????
    output logic                    pf_req,
    output logic [ADDR_WIDTH-1:0]   pf_addr,
    input  logic                    pf_ack,

    // ?? APB config ??????????????????????????????????????????
    input  logic                    cfg_pf_en,     // global enable
    input  logic [7:0]              cfg_lookahead, // dynamic distance

    // ?? Performance counter ??????????????????????????????????
    output logic [31:0]             perf_pf_issued
);

// ============================================================
//  RPT entry definition
// ============================================================
typedef enum logic [1:0] {
    S_INIT      = 2'd0,
    S_TRANSIENT = 2'd1,
    S_STEADY    = 2'd2,
    S_NO_PRED   = 2'd3
} rpt_state_t;

typedef struct packed {
    logic [ADDR_WIDTH-1:0]  tag;        // last access address
    logic signed [ADDR_WIDTH-1:0] stride; // detected stride (signed)
    rpt_state_t              state;
} rpt_entry_t;

rpt_entry_t rpt [RPT_ENTRIES-1:0];

localparam RPT_IDX_BITS = $clog2(RPT_ENTRIES);

// Simple direct-mapped RPT: index = addr[$clog2(LINE_BYTES) +: RPT_IDX_BITS]
function automatic logic [RPT_IDX_BITS-1:0] rpt_index(input logic [ADDR_WIDTH-1:0] a);
    return a[$clog2(LINE_BYTES) + RPT_IDX_BITS - 1 : $clog2(LINE_BYTES)];
endfunction

// Align address to cache line boundary
function automatic logic [ADDR_WIDTH-1:0] line_base(input logic [ADDR_WIDTH-1:0] a);
    return {a[ADDR_WIDTH-1 : $clog2(LINE_BYTES)], {$clog2(LINE_BYTES){1'b0}}};
endfunction

// ============================================================
//  Prefetch queue (single entry - can be widened)
// ============================================================
logic pf_pending;
logic [ADDR_WIDTH-1:0] pf_pending_addr;

// ============================================================
//  Hoisted combinational signals (XSIM: no automatic in always_ff)
// ============================================================
logic [RPT_IDX_BITS-1:0]     acc_idx;
logic [ADDR_WIDTH-1:0]        acc_line;
logic signed [ADDR_WIDTH-1:0] acc_stride;

always_comb begin
    acc_idx    = rpt_index(acc_addr);
    acc_line   = line_base(acc_addr);
    acc_stride = $signed(acc_line) - $signed(line_base(rpt[acc_idx].tag));
end

// ============================================================
//  Main update logic
// ============================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < RPT_ENTRIES; i++) begin
            rpt[i].tag    <= '0;
            rpt[i].stride <= '0;
            rpt[i].state  <= S_INIT;
        end
        pf_req         <= 1'b0;
        pf_pending     <= 1'b0;
        perf_pf_issued <= '0;
    end else begin
        pf_req <= 1'b0;

        // ?? Drain pending prefetch ??????????????????????????
        if (pf_pending && cfg_pf_en) begin
            pf_req  <= 1'b1;
            pf_addr <= pf_pending_addr;
            if (pf_ack) begin
                pf_pending     <= 1'b0;
                perf_pf_issued <= perf_pf_issued + 1;
            end
        end

        // ?? Process new access ??????????????????????????????
        if (acc_valid) begin

            case (rpt[acc_idx].state)
                S_INIT: begin
                    rpt[acc_idx].tag    <= acc_addr;
                    rpt[acc_idx].stride <= acc_stride;
                    rpt[acc_idx].state  <= S_TRANSIENT;
                end

                S_TRANSIENT: begin
                    rpt[acc_idx].tag <= acc_addr;
                    if (acc_stride == rpt[acc_idx].stride) begin
                        rpt[acc_idx].state <= S_STEADY;
                        pf_pending      <= 1'b1;
                        pf_pending_addr <= acc_line +
                            (rpt[acc_idx].stride * cfg_lookahead);
                    end else begin
                        rpt[acc_idx].stride <= acc_stride;
                        rpt[acc_idx].state  <= S_NO_PRED;
                    end
                end

                S_STEADY: begin
                    rpt[acc_idx].tag <= acc_addr;
                    if (acc_stride == rpt[acc_idx].stride) begin
                        pf_pending      <= 1'b1;
                        pf_pending_addr <= acc_line +
                            (rpt[acc_idx].stride * cfg_lookahead);
                    end else begin
                        rpt[acc_idx].stride <= acc_stride;
                        rpt[acc_idx].state  <= S_NO_PRED;
                    end
                end

                S_NO_PRED: begin
                    rpt[acc_idx].tag    <= acc_addr;
                    rpt[acc_idx].stride <= acc_stride;
                    rpt[acc_idx].state  <= S_TRANSIENT;
                end
            endcase
        end
    end
end

endmodule