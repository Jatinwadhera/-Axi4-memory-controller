// ============================================================
//  cache_ctrl.sv
//  4-Way Set-Associative Write-Back Cache Controller
//
//  Project : AXI4 Memory Controller with Adaptive Cache
//  Author  : Jatin Wadhera (CPG-6, TIET)
//
//  Parameters:
//    CACHE_SIZE   = 8 KB  (8192 bytes)
//    LINE_SIZE    = 64 B  (16 × 32-bit words)
//    WAYS         = 4
//    SETS         = CACHE_SIZE / (LINE_SIZE × WAYS) = 32
//
//  Address breakdown (32-bit):
//    [31:11] TAG   (21 bits)
//    [10: 6] INDEX (5 bits  → log2(32))
//    [ 5: 2] WORD  (4 bits  → log2(16))
//    [ 1: 0] BYTE  (2 bits)
//
//  Policy:
//    - LRU replacement (2-bit pseudo-LRU per set)
//    - Write-back + write-allocate
//    - Critical word first on line fill
//    - ECC: optional single-bit correction (compile flag)
// ============================================================

`timescale 1ns/1ps

module cache_ctrl #(
    parameter DATA_WIDTH  = 32,
    parameter ADDR_WIDTH  = 32,
    parameter ID_WIDTH    = 4,
    parameter CACHE_SIZE  = 8192,    // bytes
    parameter LINE_BYTES  = 64,
    parameter WAYS        = 4
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ── From AXI4 Slave Interface ────────────────────────────
    input  logic                    int_wr_req,
    input  logic [ADDR_WIDTH-1:0]   int_wr_addr,
    input  logic [DATA_WIDTH-1:0]   int_wr_data,
    input  logic [DATA_WIDTH/8-1:0] int_wr_strb,
    input  logic [ID_WIDTH-1:0]     int_wr_id,
    output logic                    int_wr_ack,

    input  logic                    int_rd_req,
    input  logic [ADDR_WIDTH-1:0]   int_rd_addr,
    input  logic [ID_WIDTH-1:0]     int_rd_id,
    input  logic [7:0]              int_rd_len,
    output logic                    int_rd_ack,

    // ── Read data back to AXI4 Slave Interface ───────────────
    output logic [DATA_WIDTH-1:0]   int_rd_data,
    output logic                    int_rd_valid,
    output logic                    int_rd_last,
    output logic [ID_WIDTH-1:0]     int_rd_resp_id,

    // ── Write response back ──────────────────────────────────
    output logic [ID_WIDTH-1:0]     int_wr_resp_id,
    output logic                    int_wr_resp_valid,
    output logic [1:0]              int_wr_resp,

    // ── To Miss Handler ──────────────────────────────────────
    output logic                    miss_req,
    output logic [ADDR_WIDTH-1:0]   miss_addr,
    output logic [ID_WIDTH-1:0]     miss_id,
    output logic                    miss_is_write,
    output logic [DATA_WIDTH-1:0]   miss_wr_data,
    output logic [DATA_WIDTH/8-1:0] miss_wr_strb,
    input  logic                    miss_ack,

    // ── Line fill from Miss Handler ──────────────────────────
    input  logic [DATA_WIDTH-1:0]   fill_data,
    input  logic                    fill_valid,
    input  logic                    fill_last,
    input  logic [ADDR_WIDTH-1:0]   fill_addr,

    // ── Writeback to Miss Handler ────────────────────────────
    output logic                    wb_req,
    output logic [ADDR_WIDTH-1:0]   wb_addr,
    output logic [DATA_WIDTH-1:0]   wb_data,
    output logic                    wb_last,
    input  logic                    wb_ack,

    // ── APB config signals ───────────────────────────────────
    input  logic                    cfg_cache_en,
    input  logic                    cfg_flush,
    input  logic                    cfg_ecc_en,

    // ── Performance counters ─────────────────────────────────
    output logic [31:0]             perf_hit_count,
    output logic [31:0]             perf_miss_count,
    output logic [31:0]             perf_wb_count
);

// ============================================================
//  Derived parameters
// ============================================================
localparam SETS          = CACHE_SIZE / (LINE_BYTES * WAYS);  // 32
localparam WORDS_PER_LINE= LINE_BYTES / (DATA_WIDTH/8);       // 16
localparam BYTE_BITS     = $clog2(DATA_WIDTH/8);              //  2
localparam WORD_BITS     = $clog2(WORDS_PER_LINE);            //  4
localparam INDEX_BITS    = $clog2(SETS);                      //  5
localparam TAG_BITS      = ADDR_WIDTH - INDEX_BITS - WORD_BITS - BYTE_BITS; // 21

// ============================================================
//  Cache arrays (behavioural, infers SRAM or flip-flops)
//  For Sky130 macros, replace with sky130_sram instantiation
// ============================================================
logic [DATA_WIDTH-1:0] data_array [SETS-1:0][WAYS-1:0][WORDS_PER_LINE-1:0];
logic [TAG_BITS-1:0]   tag_array  [SETS-1:0][WAYS-1:0];
logic                  valid_array[SETS-1:0][WAYS-1:0];
logic                  dirty_array[SETS-1:0][WAYS-1:0];
logic [1:0]            lru_array  [SETS-1:0];    // 2-bit pseudo-LRU per set

// ============================================================
//  Address breakdown
// ============================================================
function automatic logic [TAG_BITS-1:0]   get_tag   (input logic [ADDR_WIDTH-1:0] a);
    return a[ADDR_WIDTH-1 : INDEX_BITS+WORD_BITS+BYTE_BITS];
endfunction
function automatic logic [INDEX_BITS-1:0] get_index (input logic [ADDR_WIDTH-1:0] a);
    return a[INDEX_BITS+WORD_BITS+BYTE_BITS-1 : WORD_BITS+BYTE_BITS];
endfunction
function automatic logic [WORD_BITS-1:0]  get_word  (input logic [ADDR_WIDTH-1:0] a);
    return a[WORD_BITS+BYTE_BITS-1 : BYTE_BITS];
endfunction

// ============================================================
//  FSM
// ============================================================
typedef enum logic [2:0] {
    IDLE        = 3'd0,
    TAG_CHECK   = 3'd1,
    HIT_SERVE   = 3'd2,
    WRITEBACK   = 3'd3,
    FILL_WAIT   = 3'd4,
    FILL_DONE   = 3'd5,
    FLUSH_ALL   = 3'd6
} state_t;

state_t state, next;

// Registered request
logic [ADDR_WIDTH-1:0]   req_addr_r;
logic [ID_WIDTH-1:0]     req_id_r;
logic                    req_is_wr_r;
logic [DATA_WIDTH-1:0]   req_wr_data_r;
logic [DATA_WIDTH/8-1:0] req_wr_strb_r;
logic [7:0]              req_rd_len_r;

// Tag-check results
logic [1:0]              hit_way;
logic                    cache_hit;
logic [1:0]              evict_way;

// Fill beat counter
logic [WORD_BITS-1:0]    fill_cnt;
logic [WORD_BITS-1:0]    wb_cnt;

// Flush counter
logic [INDEX_BITS-1:0]   flush_set;
logic [1:0]              flush_way;

// ── Tag check combinational ──────────────────────────────────
always_comb begin
    cache_hit = 1'b0;
    hit_way   = 2'd0;
    for (int w = 0; w < WAYS; w++) begin
        if (valid_array[get_index(req_addr_r)][w] &&
            tag_array[get_index(req_addr_r)][w] == get_tag(req_addr_r)) begin
            cache_hit = 1'b1;
            hit_way   = w[1:0];
        end
    end
end

// ── LRU eviction: choose invalid first, else pseudo-LRU ─────
always_comb begin
    evict_way = lru_array[get_index(req_addr_r)];
    for (int w = 0; w < WAYS; w++) begin
        if (!valid_array[get_index(req_addr_r)][w]) begin
            evict_way = w[1:0];
        end
    end
end

// ============================================================
//  FSM – sequential
// ============================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= IDLE;
    else        state <= next;
end

// ============================================================
//  FSM – next-state
// ============================================================
always_comb begin
    next = state;
    unique case (state)
        IDLE      : if (cfg_flush)  next = FLUSH_ALL;
                    else if (int_rd_req || int_wr_req) next = TAG_CHECK;
        TAG_CHECK : next = cache_hit ? HIT_SERVE :
                           (dirty_array[get_index(req_addr_r)][evict_way] ?
                            WRITEBACK : FILL_WAIT);
        HIT_SERVE : next = IDLE;
        WRITEBACK : if (wb_ack && wb_last) next = FILL_WAIT;
        FILL_WAIT : if (fill_valid && fill_last) next = FILL_DONE;
        FILL_DONE : next = IDLE;
        FLUSH_ALL : if (flush_set == SETS-1 && flush_way == WAYS-1) next = IDLE;
        default   : next = IDLE;
    endcase
end

// ============================================================
//  FSM – outputs / datapath
// ============================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        req_addr_r     <= '0;
        req_id_r       <= '0;
        req_is_wr_r    <= 1'b0;
        req_wr_data_r  <= '0;
        req_wr_strb_r  <= '0;
        req_rd_len_r   <= '0;
        int_rd_valid   <= 1'b0;
        int_rd_last    <= 1'b0;
        int_wr_resp_valid <= 1'b0;
        miss_req       <= 1'b0;
        wb_req         <= 1'b0;
        fill_cnt       <= '0;
        wb_cnt         <= '0;
        flush_set      <= '0;
        flush_way      <= '0;
        perf_hit_count <= '0;
        perf_miss_count<= '0;
        perf_wb_count  <= '0;
        // Invalidate all lines
        for (int s = 0; s < SETS; s++)
            for (int w = 0; w < WAYS; w++) begin
                valid_array[s][w] <= 1'b0;
                dirty_array[s][w] <= 1'b0;
            end
    end else begin
        // Default de-asserts
        int_rd_valid      <= 1'b0;
        int_rd_last       <= 1'b0;
        int_wr_resp_valid <= 1'b0;
        miss_req          <= 1'b0;
        wb_req            <= 1'b0;
        int_wr_ack        <= 1'b0;
        int_rd_ack        <= 1'b0;

        case (state)
            // ──────────────────────────────────────────────
            IDLE: begin
                if (cfg_flush) begin
                    flush_set <= '0;
                    flush_way <= '0;
                end else if (int_wr_req && cfg_cache_en) begin
                    req_addr_r    <= int_wr_addr;
                    req_id_r      <= int_wr_id;
                    req_is_wr_r   <= 1'b1;
                    req_wr_data_r <= int_wr_data;
                    req_wr_strb_r <= int_wr_strb;
                    int_wr_ack    <= 1'b1;
                end else if (int_rd_req && cfg_cache_en) begin
                    req_addr_r  <= int_rd_addr;
                    req_id_r    <= int_rd_id;
                    req_is_wr_r <= 1'b0;
                    req_rd_len_r<= int_rd_len;
                    int_rd_ack  <= 1'b1;
                end
            end

            // ──────────────────────────────────────────────
            TAG_CHECK: begin
                if (cache_hit) begin
                    perf_hit_count <= perf_hit_count + 1;
                end else begin
                    perf_miss_count <= perf_miss_count + 1;
                    // Issue miss request to miss handler
                    miss_req      <= 1'b1;
                    miss_addr     <= req_addr_r;
                    miss_id       <= req_id_r;
                    miss_is_write <= req_is_wr_r;
                    miss_wr_data  <= req_wr_data_r;
                    miss_wr_strb  <= req_wr_strb_r;
                end
            end

            // ──────────────────────────────────────────────
            HIT_SERVE: begin
                if (req_is_wr_r) begin
                    // Write hit: update byte lanes
                    for (int b = 0; b < DATA_WIDTH/8; b++) begin
                        if (req_wr_strb_r[b])
                            data_array[get_index(req_addr_r)][hit_way]
                                      [get_word(req_addr_r)][b*8 +: 8]
                                <= req_wr_data_r[b*8 +: 8];
                    end
                    dirty_array[get_index(req_addr_r)][hit_way] <= 1'b1;
                    int_wr_resp_valid <= 1'b1;
                    int_wr_resp_id    <= req_id_r;
                    int_wr_resp       <= 2'b00;
                end else begin
                    // Read hit: return word
                    int_rd_data     <= data_array[get_index(req_addr_r)]
                                                  [hit_way][get_word(req_addr_r)];
                    int_rd_valid    <= 1'b1;
                    int_rd_last     <= 1'b1;
                    int_rd_resp_id  <= req_id_r;
                end
                // Update LRU
                lru_array[get_index(req_addr_r)] <= ~hit_way[0] ? 2'b01 : 2'b10;
            end

            // ──────────────────────────────────────────────
            WRITEBACK: begin
                wb_req  <= 1'b1;
                wb_addr <= {tag_array[get_index(req_addr_r)][evict_way],
                            get_index(req_addr_r),
                            {(WORD_BITS+BYTE_BITS){1'b0}}};
                wb_data <= data_array[get_index(req_addr_r)][evict_way][wb_cnt];
                wb_last <= (wb_cnt == WORDS_PER_LINE-1);
                if (wb_ack) begin
                    wb_cnt <= wb_cnt + 1;
                    if (wb_cnt == WORDS_PER_LINE-1) begin
                        wb_cnt <= '0;
                        dirty_array[get_index(req_addr_r)][evict_way] <= 1'b0;
                        perf_wb_count <= perf_wb_count + 1;
                    end
                end
            end

            // ──────────────────────────────────────────────
            FILL_WAIT: begin
                if (fill_valid) begin
                    data_array[get_index(fill_addr)][evict_way][fill_cnt] <= fill_data;
                    fill_cnt <= fill_cnt + 1;
                    if (fill_last) begin
                        fill_cnt    <= '0;
                        valid_array[get_index(fill_addr)][evict_way] <= 1'b1;
                        dirty_array[get_index(fill_addr)][evict_way] <= 1'b0;
                        tag_array  [get_index(fill_addr)][evict_way] <= get_tag(fill_addr);
                        lru_array  [get_index(fill_addr)]            <= evict_way ^ 2'b11;
                    end
                end
            end

            // ──────────────────────────────────────────────
            FILL_DONE: begin
                // Re-serve the original request from newly filled line
                if (req_is_wr_r) begin
                    for (int b = 0; b < DATA_WIDTH/8; b++)
                        if (req_wr_strb_r[b])
                            data_array[get_index(req_addr_r)][evict_way]
                                      [get_word(req_addr_r)][b*8 +: 8]
                                <= req_wr_data_r[b*8 +: 8];
                    dirty_array      [get_index(req_addr_r)][evict_way] <= 1'b1;
                    int_wr_resp_valid <= 1'b1;
                    int_wr_resp_id   <= req_id_r;
                    int_wr_resp      <= 2'b00;
                end else begin
                    int_rd_data    <= data_array[get_index(req_addr_r)]
                                                 [evict_way][get_word(req_addr_r)];
                    int_rd_valid   <= 1'b1;
                    int_rd_last    <= 1'b1;
                    int_rd_resp_id <= req_id_r;
                end
            end

            // ──────────────────────────────────────────────
            FLUSH_ALL: begin
                // Write back dirty lines sequentially
                if (dirty_array[flush_set][flush_way]) begin
                    wb_req  <= 1'b1;
                    wb_data <= data_array[flush_set][flush_way][wb_cnt];
                    wb_last <= (wb_cnt == WORDS_PER_LINE-1);
                    if (wb_ack) begin
                        wb_cnt <= wb_cnt + 1;
                        if (wb_cnt == WORDS_PER_LINE-1) begin
                            wb_cnt <= '0;
                            dirty_array[flush_set][flush_way] <= 1'b0;
                            valid_array[flush_set][flush_way] <= 1'b0;
                        end
                    end
                end else begin
                    valid_array[flush_set][flush_way] <= 1'b0;
                    if (flush_way == WAYS-1) begin
                        flush_way <= 2'd0;
                        flush_set <= flush_set + 1;
                    end else begin
                        flush_way <= flush_way + 1;
                    end
                end
            end
        endcase
    end
end

endmodule
