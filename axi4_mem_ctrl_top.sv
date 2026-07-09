// ============================================================
//  axi4_mem_ctrl_top.sv
//  Top-Level Integration
//
//  Project : AXI4 Memory Controller with Adaptive Cache
//  Author  : Jatin Wadhera (CPG-6, TIET)
//
//  Hierarchy:
//    axi4_mem_ctrl_top
//      ├── axi4_slave_if        (AXI4 slave, 5 channels)
//      ├── cache_ctrl           (4-way SA cache, 8KB)
//      ├── miss_handler         (line fill + writeback pump)
//      ├── prefetch_engine      (stride RPT prefetcher)
//      └── apb_regs             (config + perf counters)
// ============================================================

`timescale 1ns/1ps

module axi4_mem_ctrl_top #(
    parameter DATA_WIDTH  = 32,
    parameter ADDR_WIDTH  = 32,
    parameter ID_WIDTH    = 4
)(
    // System
    input  logic                    ACLK,
    input  logic                    ARESETn,

    // ── AXI4 Slave port (from CPU/DMA) ──────────────────────
    input  logic [ID_WIDTH-1:0]     S_AWID,
    input  logic [ADDR_WIDTH-1:0]   S_AWADDR,
    input  logic [7:0]              S_AWLEN,
    input  logic [2:0]              S_AWSIZE,
    input  logic [1:0]              S_AWBURST,
    input  logic                    S_AWVALID,
    output logic                    S_AWREADY,

    input  logic [DATA_WIDTH-1:0]   S_WDATA,
    input  logic [DATA_WIDTH/8-1:0] S_WSTRB,
    input  logic                    S_WLAST,
    input  logic                    S_WVALID,
    output logic                    S_WREADY,

    output logic [ID_WIDTH-1:0]     S_BID,
    output logic [1:0]              S_BRESP,
    output logic                    S_BVALID,
    input  logic                    S_BREADY,

    input  logic [ID_WIDTH-1:0]     S_ARID,
    input  logic [ADDR_WIDTH-1:0]   S_ARADDR,
    input  logic [7:0]              S_ARLEN,
    input  logic [2:0]              S_ARSIZE,
    input  logic [1:0]              S_ARBURST,
    input  logic                    S_ARVALID,
    output logic                    S_ARREADY,

    output logic [ID_WIDTH-1:0]     S_RID,
    output logic [DATA_WIDTH-1:0]   S_RDATA,
    output logic [1:0]              S_RRESP,
    output logic                    S_RLAST,
    output logic                    S_RVALID,
    input  logic                    S_RREADY,

    // ── AXI4 Master port (to external SRAM/DDR) ─────────────
    output logic [ID_WIDTH-1:0]     M_AWID,
    output logic [ADDR_WIDTH-1:0]   M_AWADDR,
    output logic [7:0]              M_AWLEN,
    output logic [2:0]              M_AWSIZE,
    output logic [1:0]              M_AWBURST,
    output logic                    M_AWVALID,
    input  logic                    M_AWREADY,

    output logic [DATA_WIDTH-1:0]   M_WDATA,
    output logic [DATA_WIDTH/8-1:0] M_WSTRB,
    output logic                    M_WLAST,
    output logic                    M_WVALID,
    input  logic                    M_WREADY,

    input  logic [ID_WIDTH-1:0]     M_BID,
    input  logic [1:0]              M_BRESP,
    input  logic                    M_BVALID,
    output logic                    M_BREADY,

    output logic [ID_WIDTH-1:0]     M_ARID,
    output logic [ADDR_WIDTH-1:0]   M_ARADDR,
    output logic [7:0]              M_ARLEN,
    output logic [2:0]              M_ARSIZE,
    output logic [1:0]              M_ARBURST,
    output logic                    M_ARVALID,
    input  logic                    M_ARREADY,

    input  logic [ID_WIDTH-1:0]     M_RID,
    input  logic [DATA_WIDTH-1:0]   M_RDATA,
    input  logic [1:0]              M_RRESP,
    input  logic                    M_RLAST,
    input  logic                    M_RVALID,
    output logic                    M_RREADY,

    // ── APB Config port (from CPU config bus) ───────────────
    input  logic [ADDR_WIDTH-1:0]   APB_PADDR,
    input  logic                    APB_PSEL,
    input  logic                    APB_PENABLE,
    input  logic                    APB_PWRITE,
    input  logic [DATA_WIDTH-1:0]   APB_PWDATA,
    output logic [DATA_WIDTH-1:0]   APB_PRDATA,
    output logic                    APB_PREADY,
    output logic                    APB_PSLVERR
);

// ============================================================
//  Internal wires
// ============================================================

// AXI4 slave if → cache ctrl
logic                    int_wr_req, int_rd_req;
logic [ADDR_WIDTH-1:0]   int_wr_addr, int_rd_addr;
logic [DATA_WIDTH-1:0]   int_wr_data;
logic [DATA_WIDTH/8-1:0] int_wr_strb;
logic [ID_WIDTH-1:0]     int_wr_id, int_rd_id;
logic [7:0]              int_rd_len;
logic                    int_wr_ack, int_rd_ack;
logic [DATA_WIDTH-1:0]   int_rd_data;
logic                    int_rd_valid, int_rd_last;
logic [ID_WIDTH-1:0]     int_rd_resp_id, int_wr_resp_id;
logic                    int_wr_resp_valid;
logic [1:0]              int_wr_resp;

// cache ctrl ↔ miss handler
logic                    miss_req, miss_is_write, miss_ack;
logic [ADDR_WIDTH-1:0]   miss_addr;
logic [ID_WIDTH-1:0]     miss_id;
logic [DATA_WIDTH-1:0]   miss_wr_data;
logic [DATA_WIDTH/8-1:0] miss_wr_strb;
logic [DATA_WIDTH-1:0]   fill_data;
logic                    fill_valid, fill_last;
logic [ADDR_WIDTH-1:0]   fill_addr;
logic                    wb_req, wb_last, wb_ack;
logic [ADDR_WIDTH-1:0]   wb_addr;
logic [DATA_WIDTH-1:0]   wb_data;

// APB cfg
logic cfg_cache_en, cfg_pf_en, cfg_ecc_en, cfg_flush;
logic [7:0] cfg_lookahead;

// Perf counters
logic [31:0] perf_hit, perf_miss, perf_wb, perf_pf;

// Prefetch → miss handler (merged with cache misses)
logic                    pf_req, pf_ack;
logic [ADDR_WIDTH-1:0]   pf_addr;

// Combined miss+prefetch arbitration (simple: cache miss wins)
logic                    mh_miss_req;
logic [ADDR_WIDTH-1:0]   mh_miss_addr;
assign mh_miss_req  = miss_req  || (pf_req  && !miss_req);
assign mh_miss_addr = miss_req  ? miss_addr : pf_addr;
assign pf_ack       = pf_req && !miss_req && miss_ack;

// ============================================================
//  Module instantiations
// ============================================================

axi4_slave_if #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH),
    .ID_WIDTH  (ID_WIDTH)
) u_slave (
    .ACLK(ACLK), .ARESETn(ARESETn),
    .AWID(S_AWID), .AWADDR(S_AWADDR), .AWLEN(S_AWLEN),
    .AWSIZE(S_AWSIZE), .AWBURST(S_AWBURST), .AWVALID(S_AWVALID), .AWREADY(S_AWREADY),
    .WDATA(S_WDATA), .WSTRB(S_WSTRB), .WLAST(S_WLAST), .WVALID(S_WVALID), .WREADY(S_WREADY),
    .BID(S_BID), .BRESP(S_BRESP), .BVALID(S_BVALID), .BREADY(S_BREADY),
    .ARID(S_ARID), .ARADDR(S_ARADDR), .ARLEN(S_ARLEN),
    .ARSIZE(S_ARSIZE), .ARBURST(S_ARBURST), .ARVALID(S_ARVALID), .ARREADY(S_ARREADY),
    .RID(S_RID), .RDATA(S_RDATA), .RRESP(S_RRESP), .RLAST(S_RLAST),
    .RVALID(S_RVALID), .RREADY(S_RREADY),
    .int_wr_req(int_wr_req), .int_wr_addr(int_wr_addr),
    .int_wr_data(int_wr_data), .int_wr_strb(int_wr_strb),
    .int_wr_id(int_wr_id), .int_wr_ack(int_wr_ack),
    .int_rd_req(int_rd_req), .int_rd_addr(int_rd_addr),
    .int_rd_id(int_rd_id), .int_rd_len(int_rd_len), .int_rd_ack(int_rd_ack),
    .int_rd_data(int_rd_data), .int_rd_valid(int_rd_valid),
    .int_rd_last(int_rd_last), .int_rd_resp_id(int_rd_resp_id),
    .int_wr_resp_id(int_wr_resp_id), .int_wr_resp_valid(int_wr_resp_valid),
    .int_wr_resp(int_wr_resp)
);

cache_ctrl #(
    .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .ID_WIDTH(ID_WIDTH)
) u_cache (
    .clk(ACLK), .rst_n(ARESETn),
    .int_wr_req(int_wr_req), .int_wr_addr(int_wr_addr),
    .int_wr_data(int_wr_data), .int_wr_strb(int_wr_strb),
    .int_wr_id(int_wr_id), .int_wr_ack(int_wr_ack),
    .int_rd_req(int_rd_req), .int_rd_addr(int_rd_addr),
    .int_rd_id(int_rd_id), .int_rd_len(int_rd_len), .int_rd_ack(int_rd_ack),
    .int_rd_data(int_rd_data), .int_rd_valid(int_rd_valid),
    .int_rd_last(int_rd_last), .int_rd_resp_id(int_rd_resp_id),
    .int_wr_resp_id(int_wr_resp_id), .int_wr_resp_valid(int_wr_resp_valid),
    .int_wr_resp(int_wr_resp),
    .miss_req(miss_req), .miss_addr(miss_addr), .miss_id(miss_id),
    .miss_is_write(miss_is_write), .miss_wr_data(miss_wr_data),
    .miss_wr_strb(miss_wr_strb), .miss_ack(miss_ack),
    .fill_data(fill_data), .fill_valid(fill_valid),
    .fill_last(fill_last), .fill_addr(fill_addr),
    .wb_req(wb_req), .wb_addr(wb_addr), .wb_data(wb_data),
    .wb_last(wb_last), .wb_ack(wb_ack),
    .cfg_cache_en(cfg_cache_en), .cfg_flush(cfg_flush), .cfg_ecc_en(cfg_ecc_en),
    .perf_hit_count(perf_hit), .perf_miss_count(perf_miss),
    .perf_wb_count(perf_wb)
);

miss_handler #(
    .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .ID_WIDTH(ID_WIDTH)
) u_miss (
    .clk(ACLK), .rst_n(ARESETn),
    .miss_req(mh_miss_req), .miss_addr(mh_miss_addr), .miss_id(miss_id),
    .miss_is_write(miss_is_write), .miss_wr_data(miss_wr_data),
    .miss_wr_strb(miss_wr_strb), .miss_ack(miss_ack),
    .fill_data(fill_data), .fill_valid(fill_valid),
    .fill_last(fill_last), .fill_addr(fill_addr),
    .wb_req(wb_req), .wb_addr(wb_addr), .wb_data(wb_data),
    .wb_last(wb_last), .wb_ack(wb_ack),
    .M_AWID(M_AWID), .M_AWADDR(M_AWADDR), .M_AWLEN(M_AWLEN),
    .M_AWSIZE(M_AWSIZE), .M_AWBURST(M_AWBURST),
    .M_AWVALID(M_AWVALID), .M_AWREADY(M_AWREADY),
    .M_WDATA(M_WDATA), .M_WSTRB(M_WSTRB), .M_WLAST(M_WLAST),
    .M_WVALID(M_WVALID), .M_WREADY(M_WREADY),
    .M_BID(M_BID), .M_BRESP(M_BRESP), .M_BVALID(M_BVALID), .M_BREADY(M_BREADY),
    .M_ARID(M_ARID), .M_ARADDR(M_ARADDR), .M_ARLEN(M_ARLEN),
    .M_ARSIZE(M_ARSIZE), .M_ARBURST(M_ARBURST),
    .M_ARVALID(M_ARVALID), .M_ARREADY(M_ARREADY),
    .M_RID(M_RID), .M_RDATA(M_RDATA), .M_RRESP(M_RRESP),
    .M_RLAST(M_RLAST), .M_RVALID(M_RVALID), .M_RREADY(M_RREADY)
);

prefetch_engine #(
    .ADDR_WIDTH(ADDR_WIDTH), .ID_WIDTH(ID_WIDTH)
) u_prefetch (
    .clk(ACLK), .rst_n(ARESETn),
    .acc_valid(int_rd_req), .acc_addr(int_rd_addr),
    .pf_req(pf_req), .pf_addr(pf_addr), .pf_ack(pf_ack),
    .cfg_pf_en(cfg_pf_en), .cfg_lookahead(cfg_lookahead),
    .perf_pf_issued(perf_pf)
);

apb_regs #(
    .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)
) u_apb (
    .PCLK(ACLK), .PRESETn(ARESETn),
    .PADDR(APB_PADDR), .PSEL(APB_PSEL), .PENABLE(APB_PENABLE),
    .PWRITE(APB_PWRITE), .PWDATA(APB_PWDATA),
    .PRDATA(APB_PRDATA), .PREADY(APB_PREADY), .PSLVERR(APB_PSLVERR),
    .cfg_cache_en(cfg_cache_en), .cfg_pf_en(cfg_pf_en),
    .cfg_ecc_en(cfg_ecc_en), .cfg_flush(cfg_flush),
    .cfg_lookahead(cfg_lookahead),
    .sts_busy(miss_req || wb_req),
    .perf_hit_count(perf_hit), .perf_miss_count(perf_miss),
    .perf_wb_count(perf_wb), .perf_pf_issued(perf_pf)
);

endmodule
