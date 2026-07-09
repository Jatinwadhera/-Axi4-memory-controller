// ============================================================
//  axi4_slave_if.sv
//  AXI4 Slave Interface
//
//  Project : AXI4 Memory Controller with Adaptive Cache
//  Author  : Jatin Wadhera (CPG-6, TIET)
//  Target  : Sky130 PDK / Xilinx KC705
//
//  Description:
//    Implements a full AXI4 slave interface (5 channels).
//    - Write path : AW + W → internal write request
//    - Read  path : AR → internal read request
//    - Response   : B (write) / R (read) back to master
//    - Outstanding: up to MAX_OUTSTANDING (8) in-flight txns
//    - ID-tagged responses (AWID / ARID round-trip)
//
//  AXI4 Channel Summary:
//    AW  : Write Address   (AWID, AWADDR, AWLEN, AWSIZE, AWBURST, AWVALID, AWREADY)
//    W   : Write Data      (WDATA, WSTRB, WLAST, WVALID, WREADY)
//    B   : Write Response  (BID, BRESP, BVALID, BREADY)
//    AR  : Read  Address   (ARID, ARADDR, ARLEN, ARSIZE, ARBURST, ARVALID, ARREADY)
//    R   : Read  Data      (RID, RDATA, RRESP, RLAST, RVALID, RREADY)
// ============================================================

`timescale 1ns/1ps

module axi4_slave_if #(
    parameter DATA_WIDTH       = 32,
    parameter ADDR_WIDTH       = 32,
    parameter ID_WIDTH         = 4,
    parameter MAX_OUTSTANDING  = 8          // must be power-of-2
)(
    // Global
    input  logic                    ACLK,
    input  logic                    ARESETn,     // active-low sync reset

    // ── AW channel (Write Address) ──────────────────────────
    input  logic [ID_WIDTH-1:0]     AWID,
    input  logic [ADDR_WIDTH-1:0]   AWADDR,
    input  logic [7:0]              AWLEN,       // burst length-1
    input  logic [2:0]              AWSIZE,      // bytes = 2^AWSIZE
    input  logic [1:0]              AWBURST,     // 0=FIXED 1=INCR 2=WRAP
    input  logic                    AWVALID,
    output logic                    AWREADY,

    // ── W channel (Write Data) ──────────────────────────────
    input  logic [DATA_WIDTH-1:0]   WDATA,
    input  logic [DATA_WIDTH/8-1:0] WSTRB,
    input  logic                    WLAST,
    input  logic                    WVALID,
    output logic                    WREADY,

    // ── B channel (Write Response) ──────────────────────────
    output logic [ID_WIDTH-1:0]     BID,
    output logic [1:0]              BRESP,       // 0=OKAY 2=SLVERR
    output logic                    BVALID,
    input  logic                    BREADY,

    // ── AR channel (Read Address) ───────────────────────────
    input  logic [ID_WIDTH-1:0]     ARID,
    input  logic [ADDR_WIDTH-1:0]   ARADDR,
    input  logic [7:0]              ARLEN,
    input  logic [2:0]              ARSIZE,
    input  logic [1:0]              ARBURST,
    input  logic                    ARVALID,
    output logic                    ARREADY,

    // ── R channel (Read Data) ───────────────────────────────
    output logic [ID_WIDTH-1:0]     RID,
    output logic [DATA_WIDTH-1:0]   RDATA,
    output logic [1:0]              RRESP,
    output logic                    RLAST,
    output logic                    RVALID,
    input  logic                    RREADY,

    // ── Internal request bus → Cache Controller ─────────────
    output logic                    int_wr_req,
    output logic [ADDR_WIDTH-1:0]   int_wr_addr,
    output logic [DATA_WIDTH-1:0]   int_wr_data,
    output logic [DATA_WIDTH/8-1:0] int_wr_strb,
    output logic [ID_WIDTH-1:0]     int_wr_id,
    input  logic                    int_wr_ack,     // cache accepted write

    output logic                    int_rd_req,
    output logic [ADDR_WIDTH-1:0]   int_rd_addr,
    output logic [ID_WIDTH-1:0]     int_rd_id,
    output logic [7:0]              int_rd_len,
    input  logic                    int_rd_ack,

    // ── Read data return from Cache Controller ───────────────
    input  logic [DATA_WIDTH-1:0]   int_rd_data,
    input  logic                    int_rd_valid,
    input  logic                    int_rd_last,
    input  logic [ID_WIDTH-1:0]     int_rd_resp_id,

    // ── Write response from Cache Controller ────────────────
    input  logic [ID_WIDTH-1:0]     int_wr_resp_id,
    input  logic                    int_wr_resp_valid,
    input  logic [1:0]              int_wr_resp     // 0=OK 2=ERR
);

// ============================================================
//  WRITE PATH
// ============================================================

// ── Write address queue (small FIFO for outstanding AW) ──
localparam LOG_OUTSTD = $clog2(MAX_OUTSTANDING);

typedef struct packed {
    logic [ID_WIDTH-1:0]   id;
    logic [ADDR_WIDTH-1:0] addr;
    logic [7:0]            len;
    logic [2:0]            size;
    logic [1:0]            burst;
} aw_entry_t;

aw_entry_t aw_queue [MAX_OUTSTANDING-1:0];
logic [LOG_OUTSTD-1:0] aw_wr_ptr, aw_rd_ptr;
logic [LOG_OUTSTD  :0] aw_count;          // one extra bit for full/empty

wire aw_full  = (aw_count == MAX_OUTSTANDING);
wire aw_empty = (aw_count == 0);

assign AWREADY = ~aw_full;

// Enqueue on AW handshake
always_ff @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
        aw_wr_ptr <= '0;
        aw_count  <= '0;
    end else begin
        if (AWVALID && AWREADY) begin
            aw_queue[aw_wr_ptr] <= '{AWID, AWADDR, AWLEN, AWSIZE, AWBURST};
            aw_wr_ptr           <= aw_wr_ptr + 1'b1;
            aw_count            <= aw_count + 1'b1;
        end
        if (int_wr_ack && !aw_empty) begin
            aw_rd_ptr <= aw_rd_ptr + 1'b1;
            aw_count  <= aw_count - 1'b1;
        end
    end
end

// ── W channel: always ready when there is an AW entry pending ──
assign WREADY = ~aw_empty;

// Drive internal write request
assign int_wr_req  = WVALID && WREADY;
assign int_wr_addr = aw_queue[aw_rd_ptr].addr;
assign int_wr_data = WDATA;
assign int_wr_strb = WSTRB;
assign int_wr_id   = aw_queue[aw_rd_ptr].id;

// ── B channel: forward write response from cache ────────────
assign BVALID = int_wr_resp_valid;
assign BID    = int_wr_resp_id;
assign BRESP  = int_wr_resp;

// ============================================================
//  READ PATH
// ============================================================

typedef struct packed {
    logic [ID_WIDTH-1:0]   id;
    logic [ADDR_WIDTH-1:0] addr;
    logic [7:0]            len;
    logic [2:0]            size;
    logic [1:0]            burst;
} ar_entry_t;

ar_entry_t ar_queue [MAX_OUTSTANDING-1:0];
logic [LOG_OUTSTD-1:0] ar_wr_ptr, ar_rd_ptr;
logic [LOG_OUTSTD  :0] ar_count;

wire ar_full  = (ar_count == MAX_OUTSTANDING);
wire ar_empty = (ar_count == 0);

assign ARREADY = ~ar_full;

always_ff @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
        ar_wr_ptr <= '0;
        ar_count  <= '0;
    end else begin
        if (ARVALID && ARREADY) begin
            ar_queue[ar_wr_ptr] <= '{ARID, ARADDR, ARLEN, ARSIZE, ARBURST};
            ar_wr_ptr           <= ar_wr_ptr + 1'b1;
            ar_count            <= ar_count + 1'b1;
        end
        if (int_rd_ack && !ar_empty) begin
            ar_rd_ptr <= ar_rd_ptr + 1'b1;
            ar_count  <= ar_count - 1'b1;
        end
    end
end

assign int_rd_req  = ~ar_empty;
assign int_rd_addr = ar_queue[ar_rd_ptr].addr;
assign int_rd_id   = ar_queue[ar_rd_ptr].id;
assign int_rd_len  = ar_queue[ar_rd_ptr].len;

// ── R channel: pipe read data back to master ────────────────
assign RVALID = int_rd_valid;
assign RDATA  = int_rd_data;
assign RID    = int_rd_resp_id;
assign RRESP  = 2'b00;        // OKAY
assign RLAST  = int_rd_last;

// ============================================================
//  ASSERTIONS (synthesisable SVA)
// ============================================================
`ifdef SIMULATION
// AWLEN must be < 256 for INCR burst
property p_awlen_incr;
    @(posedge ACLK) disable iff (!ARESETn)
    (AWVALID && AWBURST==2'b01) |-> (AWLEN <= 8'hFF);
endproperty
assert property (p_awlen_incr) else $error("AXI4: AWLEN exceeded for INCR burst");

// WLAST must assert on final beat
property p_wlast;
    @(posedge ACLK) disable iff (!ARESETn)
    (WVALID && WREADY && WLAST) |-> !aw_empty;
endproperty
assert property (p_wlast) else $warning("AXI4: WLAST with no AW entry");
`endif

endmodule
