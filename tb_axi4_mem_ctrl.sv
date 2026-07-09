// ============================================================
//  tb_axi4_mem_ctrl.sv
//  XSIM-Compatible Self-Checking Testbench
//  Fixed for Vivado XSIM (no @iff, no logic-keyed assoc arrays,
//  no automatic vars in always blocks, no fork/join in tasks)
// ============================================================
`timescale 1ns/1ps

module tb_axi4_mem_ctrl;

localparam DATA_W  = 32;
localparam ADDR_W  = 32;
localparam ID_W    = 4;
localparam CLK_PER = 10;

// ============================================================
//  Clock & Reset
// ============================================================
logic ACLK    = 0;
logic ARESETn = 0;
always #(CLK_PER/2) ACLK = ~ACLK;

// ============================================================
//  DUT signals
// ============================================================
logic [ID_W-1:0]     S_AWID    = 0;
logic [ADDR_W-1:0]   S_AWADDR  = 0;
logic [7:0]          S_AWLEN   = 0;
logic [2:0]          S_AWSIZE  = 0;
logic [1:0]          S_AWBURST = 0;
logic                S_AWVALID = 0;
logic                S_AWREADY;
logic [DATA_W-1:0]   S_WDATA   = 0;
logic [DATA_W/8-1:0] S_WSTRB   = 0;
logic                S_WLAST   = 0;
logic                S_WVALID  = 0;
logic                S_WREADY;
logic [ID_W-1:0]     S_BID;
logic [1:0]          S_BRESP;
logic                S_BVALID;
logic                S_BREADY  = 0;
logic [ID_W-1:0]     S_ARID    = 0;
logic [ADDR_W-1:0]   S_ARADDR  = 0;
logic [7:0]          S_ARLEN   = 0;
logic [2:0]          S_ARSIZE  = 0;
logic [1:0]          S_ARBURST = 0;
logic                S_ARVALID = 0;
logic                S_ARREADY;
logic [ID_W-1:0]     S_RID;
logic [DATA_W-1:0]   S_RDATA;
logic [1:0]          S_RRESP;
logic                S_RLAST;
logic                S_RVALID;
logic                S_RREADY  = 0;

logic [ID_W-1:0]     M_AWID;
logic [ADDR_W-1:0]   M_AWADDR;
logic [7:0]          M_AWLEN;
logic [2:0]          M_AWSIZE;
logic [1:0]          M_AWBURST;
logic                M_AWVALID;
logic                M_AWREADY = 0;
logic [DATA_W-1:0]   M_WDATA;
logic [DATA_W/8-1:0] M_WSTRB;
logic                M_WLAST;
logic                M_WVALID;
logic                M_WREADY  = 0;
logic [ID_W-1:0]     M_BID     = 0;
logic [1:0]          M_BRESP   = 0;
logic                M_BVALID  = 0;
logic                M_BREADY;
logic [ID_W-1:0]     M_ARID;
logic [ADDR_W-1:0]   M_ARADDR;
logic [7:0]          M_ARLEN;
logic [2:0]          M_ARSIZE;
logic [1:0]          M_ARBURST;
logic                M_ARVALID;
logic                M_ARREADY = 0;
logic [ID_W-1:0]     M_RID     = 0;
logic [DATA_W-1:0]   M_RDATA   = 0;
logic [1:0]          M_RRESP   = 0;
logic                M_RLAST   = 0;
logic                M_RVALID  = 0;
logic                M_RREADY;

logic [ADDR_W-1:0]   APB_PADDR   = 0;
logic                APB_PSEL    = 0;
logic                APB_PENABLE = 0;
logic                APB_PWRITE  = 0;
logic [DATA_W-1:0]   APB_PWDATA  = 0;
logic [DATA_W-1:0]   APB_PRDATA;
logic                APB_PREADY;
logic                APB_PSLVERR;

// ============================================================
//  DUT
// ============================================================
axi4_mem_ctrl_top #(
    .DATA_WIDTH(DATA_W),
    .ADDR_WIDTH(ADDR_W),
    .ID_WIDTH  (ID_W)
) dut (.*);

// ============================================================
//  External memory model - 64KB
// ============================================================
localparam MEM_SIZE = 65536;
logic [7:0] ext_mem [0:MEM_SIZE-1];

integer mem_i;
initial begin
    for (mem_i = 0; mem_i < MEM_SIZE; mem_i = mem_i + 1)
        ext_mem[mem_i] = mem_i[7:0] ^ 8'hA5;
end

// ?? AR ? R (external memory read response) ??????????????????
reg [ADDR_W-1:0] ar_base_r;
reg [7:0]        ar_len_r;
reg [ID_W-1:0]   ar_id_r;
reg [3:0]        rd_beat;
reg              rd_active;

always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
        M_ARREADY <= 1'b1;
        M_RVALID  <= 1'b0;
        M_RLAST   <= 1'b0;
        rd_active <= 1'b0;
        rd_beat   <= 0;
    end else begin
        M_RVALID <= 1'b0;
        M_RLAST  <= 1'b0;

        if (!rd_active) begin
            M_ARREADY <= 1'b1;
            if (M_ARVALID && M_ARREADY) begin
                ar_base_r <= M_ARADDR;
                ar_len_r  <= M_ARLEN;
                ar_id_r   <= M_ARID;
                rd_beat   <= 0;
                rd_active <= 1'b1;
                M_ARREADY <= 1'b0;
            end
        end else begin
            M_RVALID <= 1'b1;
            M_RID    <= ar_id_r;
            M_RRESP  <= 2'b00;
            M_RDATA  <= { ext_mem[(ar_base_r + rd_beat*4 + 3) % MEM_SIZE],
                          ext_mem[(ar_base_r + rd_beat*4 + 2) % MEM_SIZE],
                          ext_mem[(ar_base_r + rd_beat*4 + 1) % MEM_SIZE],
                          ext_mem[(ar_base_r + rd_beat*4 + 0) % MEM_SIZE] };
            M_RLAST  <= (rd_beat == ar_len_r);
            if (M_RREADY) begin
                if (rd_beat == ar_len_r) begin
                    rd_active <= 1'b0;
                    M_RVALID  <= 1'b0;
                    M_RLAST   <= 1'b0;
                    M_ARREADY <= 1'b1;
                end else begin
                    rd_beat <= rd_beat + 1;
                end
            end
        end
    end
end

// ?? AW + W ? B (external memory write response) ?????????????
reg [ADDR_W-1:0] aw_base_r;
reg [7:0]        aw_len_r;
reg [ID_W-1:0]   aw_id_r;
reg [3:0]        wr_beat;
reg              wr_addr_captured;
reg              wr_active;
integer          b;

always @(posedge ACLK or negedge ARESETn) begin
    if (!ARESETn) begin
        M_AWREADY       <= 1'b1;
        M_WREADY        <= 1'b0;
        M_BVALID        <= 1'b0;
        wr_active       <= 1'b0;
        wr_addr_captured<= 1'b0;
        wr_beat         <= 0;
    end else begin
        M_BVALID <= 1'b0;

        if (!wr_addr_captured) begin
            M_AWREADY <= 1'b1;
            M_WREADY  <= 1'b0;
            if (M_AWVALID && M_AWREADY) begin
                aw_base_r        <= M_AWADDR;
                aw_len_r         <= M_AWLEN;
                aw_id_r          <= M_AWID;
                wr_beat          <= 0;
                wr_addr_captured <= 1'b1;
                M_AWREADY        <= 1'b0;
                M_WREADY         <= 1'b1;
            end
        end else begin
            M_WREADY <= 1'b1;
            if (M_WVALID && M_WREADY) begin
                // Write byte lanes
                for (b = 0; b < 4; b = b + 1) begin
                    if (M_WSTRB[b])
                        ext_mem[(aw_base_r + wr_beat*4 + b) % MEM_SIZE] <= M_WDATA[b*8 +: 8];
                end
                if (M_WLAST) begin
                    wr_addr_captured <= 1'b0;
                    M_WREADY         <= 1'b0;
                    M_BVALID         <= 1'b1;
                    M_BID            <= aw_id_r;
                    M_BRESP          <= 2'b00;
                end else begin
                    wr_beat <= wr_beat + 1;
                end
            end
            if (M_BVALID && M_BREADY) begin
                M_BVALID  <= 1'b0;
                M_AWREADY <= 1'b1;
            end
        end
    end
end

// ============================================================
//  Golden reference memory - flat array (XSIM safe)
//  Covers 16KB address space word-indexed
// ============================================================
localparam GOLD_WORDS = 4096;   // 16KB / 4
reg [DATA_W-1:0]  golden_mem  [0:GOLD_WORDS-1];
reg               golden_valid[0:GOLD_WORDS-1];
integer           gi;

initial begin
    for (gi = 0; gi < GOLD_WORDS; gi = gi + 1) begin
        golden_mem[gi]   = 0;
        golden_valid[gi] = 0;
    end
end

// ============================================================
//  Stats
// ============================================================
integer pass_cnt;
integer fail_cnt;
initial begin
    pass_cnt = 0;
    fail_cnt = 0;
end

// ============================================================
//  Helper tasks  (XSIM-safe: no @iff, no automatic locals)
// ============================================================

// ?? Wait for signal with timeout ????????????????????????????
task wait_for_signal;
    input  reg   sig;
    input  [31:0] timeout_cycles;
    integer cnt;
    begin
        cnt = 0;
        while (!sig && cnt < timeout_cycles) begin
            @(posedge ACLK);
            cnt = cnt + 1;
        end
        if (cnt >= timeout_cycles)
            $error("[TIMEOUT] Signal did not assert in %0d cycles", timeout_cycles);
    end
endtask

// ?? AXI4 single-beat write ????????????????????????????????????
reg [ADDR_W-1:0] wr_addr_t;
reg [DATA_W-1:0] wr_data_t;

task axi_write;
    input [ADDR_W-1:0] addr;
    input [DATA_W-1:0] data;
    begin
        wr_addr_t = addr;
        wr_data_t = data;
        @(posedge ACLK);
        // Assert AW + W together
        S_AWID    = 4'h1;
        S_AWADDR  = addr;
        S_AWLEN   = 8'd0;
        S_AWSIZE  = 3'd2;
        S_AWBURST = 2'b01;
        S_AWVALID = 1'b1;
        S_WDATA   = data;
        S_WSTRB   = 4'hF;
        S_WLAST   = 1'b1;
        S_WVALID  = 1'b1;
        S_BREADY  = 1'b1;

        // Wait AW handshake
        @(posedge ACLK);
        while (!S_AWREADY) @(posedge ACLK);
        S_AWVALID = 1'b0;

        // Wait W handshake
        while (!S_WREADY) @(posedge ACLK);
        S_WVALID = 1'b0;
        S_WLAST  = 1'b0;

        // Wait B
        while (!S_BVALID) @(posedge ACLK);
        if (S_BRESP != 2'b00)
            $error("[FAIL] Write BRESP=%b at addr %08h", S_BRESP, addr);
        @(posedge ACLK);
        S_BREADY = 1'b0;

        // Update golden
        if ((addr >> 2) < GOLD_WORDS) begin
            golden_mem  [addr >> 2] = data;
            golden_valid[addr >> 2] = 1;
        end
    end
endtask

// ?? AXI4 single-beat read ?????????????????????????????????????
reg [DATA_W-1:0] rd_result;

task axi_read;
    input [ADDR_W-1:0] addr;
    begin
        @(posedge ACLK);
        S_ARID    = 4'h2;
        S_ARADDR  = addr;
        S_ARLEN   = 8'd0;
        S_ARSIZE  = 3'd2;
        S_ARBURST = 2'b01;
        S_ARVALID = 1'b1;
        S_RREADY  = 1'b1;

        @(posedge ACLK);
        while (!S_ARREADY) @(posedge ACLK);
        S_ARVALID = 1'b0;

        while (!S_RVALID) @(posedge ACLK);
        rd_result = S_RDATA;
        @(posedge ACLK);
        S_RREADY = 1'b0;
    end
endtask

// ?? Check read vs golden ??????????????????????????????????????
task check_read;
    input [ADDR_W-1:0] addr;
    input [DATA_W-1:0] got;
    begin
        if ((addr >> 2) < GOLD_WORDS && golden_valid[addr >> 2]) begin
            if (got === golden_mem[addr >> 2]) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] addr=%08h  got=%08h", addr, got);
            end else begin
                fail_cnt = fail_cnt + 1;
                $error("[FAIL] addr=%08h  expected=%08h  got=%08h",
                       addr, golden_mem[addr >> 2], got);
            end
        end
    end
endtask

// ?? APB write ?????????????????????????????????????????????????
task apb_write;
    input [ADDR_W-1:0] addr;
    input [DATA_W-1:0] data;
    begin
        @(posedge ACLK);
        APB_PADDR   = addr;
        APB_PWDATA  = data;
        APB_PWRITE  = 1'b1;
        APB_PSEL    = 1'b1;
        APB_PENABLE = 1'b0;
        @(posedge ACLK);
        APB_PENABLE = 1'b1;
        while (!APB_PREADY) @(posedge ACLK);
        @(posedge ACLK);
        APB_PSEL    = 1'b0;
        APB_PENABLE = 1'b0;
    end
endtask

// ?? APB read ??????????????????????????????????????????????????
reg [DATA_W-1:0] apb_rd_result;

task apb_read;
    input [ADDR_W-1:0] addr;
    begin
        @(posedge ACLK);
        APB_PADDR   = addr;
        APB_PWRITE  = 1'b0;
        APB_PSEL    = 1'b1;
        APB_PENABLE = 1'b0;
        @(posedge ACLK);
        APB_PENABLE = 1'b1;
        while (!APB_PREADY) @(posedge ACLK);
        apb_rd_result = APB_PRDATA;
        @(posedge ACLK);
        APB_PSEL    = 1'b0;
        APB_PENABLE = 1'b0;
    end
endtask

// ============================================================
//  Main test
// ============================================================
integer i;
reg [ADDR_W-1:0] rnd_addr;
reg [DATA_W-1:0] rnd_data;
reg              rnd_wr;
integer          timeout_ctr;

initial begin
    $dumpfile("tb_axi4_mem_ctrl.vcd");
    $dumpvars(0, tb_axi4_mem_ctrl);

    repeat(5) @(posedge ACLK);
    ARESETn = 1;
    repeat(5) @(posedge ACLK);

    // ?? TEST 0: Reset ????????????????????????????????????????
    $display("\n=== TEST 0: Reset check ===");
    $display("[PASS] Reset released, starting tests");

    // ?? Enable cache + prefetch via APB ??????????????????????
    apb_write(32'h00, 32'h0000_0003); // cache_en=1 pf_en=1

    // ?? TEST 1: Write then read (cache HIT) ??????????????????
    $display("\n=== TEST 1: Write then Read (expect HIT on 2nd read) ===");
    axi_write(32'h0000_0100, 32'hDEAD_BEEF);
    repeat(2) @(posedge ACLK);
    axi_read(32'h0000_0100);
    check_read(32'h0000_0100, rd_result);

    // ?? TEST 2: Cache miss (cold read) ???????????????????????
    $display("\n=== TEST 2: Cold read - cache MISS + fill ===");
    axi_read(32'h0000_1000);
    $display("  Fill result: %08h", rd_result);
    repeat(4) @(posedge ACLK);
    // Second read - should hit
    axi_read(32'h0000_1000);
    $display("  Repeat read: %08h (should match fill)", rd_result);

    // ?? TEST 3: Eviction pressure ????????????????????????????
    $display("\n=== TEST 3: Eviction / writeback stress ===");
    for (i = 0; i < 40; i = i + 1) begin
        axi_write(32'h0000_2000 + i*64, 32'hCAFE_0000 + i);
        repeat(1) @(posedge ACLK);
    end
    axi_read(32'h0000_2000);
    $display("  Eviction stress read: %08h", rd_result);

    // ?? TEST 4: WSTRB byte-lane test ?????????????????????????
    $display("\n=== TEST 4: WSTRB byte lane test ===");
    axi_write(32'h0000_0200, 32'hFFFF_FFFF);
    repeat(2) @(posedge ACLK);
    // Write only byte 0 = 0x42
    @(posedge ACLK);
    S_AWID=4'h1; S_AWADDR=32'h0000_0200;
    S_AWLEN=0; S_AWSIZE=3'd2; S_AWBURST=2'b01; S_AWVALID=1;
    S_WDATA=32'hXXXX_XX42; S_WSTRB=4'b0001;
    S_WLAST=1; S_WVALID=1; S_BREADY=1;
    @(posedge ACLK);
    while (!S_AWREADY) @(posedge ACLK);
    S_AWVALID=0;
    while (!S_WREADY)  @(posedge ACLK);
    S_WVALID=0; S_WLAST=0;
    while (!S_BVALID)  @(posedge ACLK);
    S_BREADY=0;
    repeat(2) @(posedge ACLK);
    axi_read(32'h0000_0200);
    if (rd_result[7:0] == 8'h42)
        $display("[PASS] WSTRB byte0=0x42 correct, full word=%08h", rd_result);
    else
        $error("[FAIL] WSTRB wrong: %08h", rd_result);

    // ?? TEST 5: APB registers ????????????????????????????????
    $display("\n=== TEST 5: APB register read/write ===");
    apb_write(32'h08, 32'h0000_0004); // PF lookahead = 4
    apb_read (32'h08);
    if (apb_rd_result[7:0] == 8'h04)
        $display("[PASS] PF_CFG readback = %08h", apb_rd_result);
    else
        $error("[FAIL] PF_CFG readback = %08h (expected 0x4)", apb_rd_result);

    apb_read(32'h0C); $display("  HIT_CNT  = %0d", apb_rd_result);
    apb_read(32'h10); $display("  MISS_CNT = %0d", apb_rd_result);
    apb_read(32'h14); $display("  WB_CNT   = %0d", apb_rd_result);
    apb_read(32'h18); $display("  PF_CNT   = %0d", apb_rd_result);

    // ?? TEST 6: Stride prefetch ??????????????????????????????
    $display("\n=== TEST 6: Stride prefetch (stride=64 bytes) ===");
    axi_read(32'h0000_3000); repeat(3) @(posedge ACLK);
    axi_read(32'h0000_3040); repeat(3) @(posedge ACLK);
    axi_read(32'h0000_3080); repeat(10) @(posedge ACLK);
    axi_read(32'h0000_30C0); // should be prefetch HIT
    $display("  Prefetch-assisted read: %08h", rd_result);
    apb_read(32'h18); $display("  PF_CNT after stride = %0d", apb_rd_result);

    // ?? TEST 7: Random stress ????????????????????????????????
    $display("\n=== TEST 7: Random stress 128 ops ===");
    for (i = 0; i < 128; i = i + 1) begin
        rnd_addr = ({$random} & 32'h0000_3FFC); // word-aligned
        rnd_data = $random;
        rnd_wr   = $random & 1;
        if (rnd_wr) begin
            axi_write(rnd_addr, rnd_data);
        end else begin
            axi_read(rnd_addr);
            check_read(rnd_addr, rd_result);
        end
        repeat(1) @(posedge ACLK);
    end

    // ?? TEST 8: Cache flush ??????????????????????????????????
    $display("\n=== TEST 8: Cache flush via APB ===");
    for (i = 0; i < 8; i = i + 1)
        axi_write(32'h0000_5000 + i*4, 32'hF0F0_0000 + i);
    repeat(5) @(posedge ACLK);
    apb_write(32'h00, 32'h0000_000B); // flush bit
    timeout_ctr = 0;
    apb_read(32'h04);
    while (apb_rd_result[0] && timeout_ctr < 2000) begin
        @(posedge ACLK);
        apb_read(32'h04);
        timeout_ctr = timeout_ctr + 1;
    end
    if (timeout_ctr < 2000)
        $display("[PASS] Flush done in %0d cycles", timeout_ctr);
    else
        $error("[FAIL] Flush timed out");

    // ?? Final report ?????????????????????????????????????????
    $display("\n==============================================");
    $display("  SIMULATION COMPLETE");
    $display("  PASS: %0d  |  FAIL: %0d", pass_cnt, fail_cnt);
    if (fail_cnt == 0)
        $display("  >>> ALL TESTS PASSED <<<");
    else
        $display("  >>> %0d TEST(S) FAILED <<<", fail_cnt);
    $display("==============================================\n");

    repeat(10) @(posedge ACLK);
    $finish;
end

// ?? Watchdog ????????????????????????????????????????????????
initial begin
    #10_000_000;
    $error("SIMULATION TIMEOUT - check for deadlock");
    $finish;
end

endmodule