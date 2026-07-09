// ============================================================
//  apb_regs.sv
//  APB Configuration & Performance Counter Register File
//
//  Project : AXI4 Memory Controller with Adaptive Cache
//  Author  : Jatin Wadhera (CPG-6, TIET)
//
//  APB Protocol Recap (AMBA APB3):
//    Setup phase  : PSEL=1, PENABLE=0  (address & control valid)
//    Access phase : PSEL=1, PENABLE=1  (data transfer)
//    Read  : PWRITE=0, PRDATA driven by slave
//    Write : PWRITE=1, PWDATA sampled by slave
//
//  Register Map (word-addressed, 32-bit):
//    Offset 0x00 : CTRL     [0]=cache_en [1]=pf_en [2]=ecc_en [3]=flush
//    Offset 0x04 : STATUS   [0]=busy     (read-only)
//    Offset 0x08 : PF_CFG   [7:0]=lookahead_dist
//    Offset 0x0C : HIT_CNT  (read-only) — saturating 32-bit
//    Offset 0x10 : MISS_CNT (read-only)
//    Offset 0x14 : WB_CNT   (read-only)
//    Offset 0x18 : PF_CNT   (read-only)
//    Offset 0x1C : CNT_CLR  [0]=write-1-to-clear all counters
// ============================================================

`timescale 1ns/1ps

module apb_regs #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  logic                    PCLK,
    input  logic                    PRESETn,

    // APB slave port
    input  logic [ADDR_WIDTH-1:0]   PADDR,
    input  logic                    PSEL,
    input  logic                    PENABLE,
    input  logic                    PWRITE,
    input  logic [DATA_WIDTH-1:0]   PWDATA,
    output logic [DATA_WIDTH-1:0]   PRDATA,
    output logic                    PREADY,
    output logic                    PSLVERR,

    // Configuration outputs → cache/prefetch
    output logic                    cfg_cache_en,
    output logic                    cfg_pf_en,
    output logic                    cfg_ecc_en,
    output logic                    cfg_flush,       // pulse
    output logic [7:0]              cfg_lookahead,

    // Status inputs ← cache
    input  logic                    sts_busy,

    // Performance counter inputs ← cache + prefetch
    input  logic [31:0]             perf_hit_count,
    input  logic [31:0]             perf_miss_count,
    input  logic [31:0]             perf_wb_count,
    input  logic [31:0]             perf_pf_issued
);

// ============================================================
//  Register definitions
// ============================================================
logic [DATA_WIDTH-1:0] reg_ctrl;       // 0x00
logic [DATA_WIDTH-1:0] reg_pf_cfg;     // 0x08
logic                  reg_cnt_clr;    // 0x1C (write-1 pulse)

// Latched (cleared) counter snapshots
logic [31:0] snap_hit, snap_miss, snap_wb, snap_pf;
logic        clr_pulse;

// ── PREADY always single-cycle ─────────────────────────────
assign PREADY  = 1'b1;
assign PSLVERR = 1'b0;

// ── Config decode ───────────────────────────────────────────
assign cfg_cache_en  = reg_ctrl[0];
assign cfg_pf_en     = reg_ctrl[1];
assign cfg_ecc_en    = reg_ctrl[2];
assign cfg_flush     = reg_ctrl[3];    // self-clearing one cycle after set
assign cfg_lookahead = reg_pf_cfg[7:0];

// ============================================================
//  Write path
// ============================================================
always_ff @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        reg_ctrl   <= 32'h0000_0001;   // cache ON by default
        reg_pf_cfg <= 32'h0000_0002;   // lookahead = 2
        clr_pulse  <= 1'b0;
    end else begin
        // Auto-clear flush bit (pulse semantics)
        if (reg_ctrl[3]) reg_ctrl[3] <= 1'b0;
        clr_pulse <= 1'b0;

        if (PSEL && PENABLE && PWRITE) begin
            case (PADDR[4:2])       // word address
                3'd0: reg_ctrl   <= PWDATA;
                3'd2: reg_pf_cfg <= PWDATA;
                3'd7: clr_pulse  <= PWDATA[0];   // CNT_CLR
                default: ;
            endcase
        end
    end
end

// ============================================================
//  Counter snapshot (latch hardware counters; clear on demand)
// ============================================================
always_ff @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn || clr_pulse) begin
        snap_hit  <= '0;
        snap_miss <= '0;
        snap_wb   <= '0;
        snap_pf   <= '0;
    end else begin
        snap_hit  <= perf_hit_count;
        snap_miss <= perf_miss_count;
        snap_wb   <= perf_wb_count;
        snap_pf   <= perf_pf_issued;
    end
end

// ============================================================
//  Read path
// ============================================================
always_comb begin
    PRDATA = '0;
    if (PSEL && !PWRITE) begin
        case (PADDR[4:2])
            3'd0: PRDATA = reg_ctrl;
            3'd1: PRDATA = {31'b0, sts_busy};
            3'd2: PRDATA = reg_pf_cfg;
            3'd3: PRDATA = snap_hit;
            3'd4: PRDATA = snap_miss;
            3'd5: PRDATA = snap_wb;
            3'd6: PRDATA = snap_pf;
            3'd7: PRDATA = '0;        // CNT_CLR is write-only
            default: PRDATA = 32'hDEAD_BEEF;  // unmapped
        endcase
    end
end

endmodule
