// this is a test bench feeds initial instruction and data
// the processor output is not verified

`timescale 1 ns/10 ps

`define CYCLE 4.8 // You can modify your clock frequency

`define DMEM_INIT "D_mem"
`define SDFFILE   "./CHIP_syn.sdf"	// Modify your SDF file name

// For different condition (I_mem, TestBed)
`ifdef noHazard
    `define IMEM_INIT "I_mem_noHazard"
    `include "./TestBed_noHazard.v"
`endif
`ifdef hasHazard
    `define IMEM_INIT "I_mem_hasHazard"
    `include "./TestBed_hasHazard.v"
`endif
`ifdef BrPred
    `define IMEM_INIT "I_mem_BrPred"
    `include "./TestBed_BrPred.v"
`endif
`ifdef compression
    `define IMEM_INIT "I_mem_compression"
    `include "./TestBed_compression.v"
`endif
`ifdef decompression
    `define IMEM_INIT "I_mem_decompression"
    `include "./TestBed_compression.v"
`endif

module Final_tb;

    reg clk;
    reg rst_n;

    wire mem_read_D;
    wire mem_write_D;
    wire [31:4] mem_addr_D;
    wire [127:0] mem_wdata_D;
    wire [127:0] mem_rdata_D;
    wire mem_ready_D;

    wire mem_read_I;
    wire mem_write_I;
    wire [31:4] mem_addr_I;
    wire [127:0] mem_wdata_I;
    wire [127:0] mem_rdata_I;
    wire mem_ready_I;

    wire [29:0] DCACHE_addr;
    wire [31:0] DCACHE_wdata;
    wire        DCACHE_wen;

    wire [7:0] error_num;
    wire [15:0] duration;
    wire finish;

    // Note the design is connected at testbench, include:
    // 1. CHIP (RISCV + D_cache + I_chache)
    // 2. slow memory for data
    // 3. slow memory for instruction

    CHIP chip0 (clk,
                rst_n,
//----------for slow_memD------------
                mem_read_D,
                mem_write_D,
                mem_addr_D,
                mem_wdata_D,
                mem_rdata_D,
                mem_ready_D,
//----------for slow_memI------------
                mem_read_I,
                mem_write_I,
                mem_addr_I,
                mem_wdata_I,
                mem_rdata_I,
                mem_ready_I,
//----------for TestBed--------------
                DCACHE_addr,
                DCACHE_wdata,
                DCACHE_wen
                );

    slow_memory slow_memD(
        .clk        (clk)           ,
        .mem_read   (mem_read_D)    ,
        .mem_write  (mem_write_D)   ,
        .mem_addr   (mem_addr_D)    ,
        .mem_wdata  (mem_wdata_D)   ,
        .mem_rdata  (mem_rdata_D)   ,
        .mem_ready  (mem_ready_D)
    );

    slow_memory slow_memI(
        .clk        (clk)           ,
        .mem_read   (mem_read_I)    ,
        .mem_write  (mem_write_I)   ,
        .mem_addr   (mem_addr_I)    ,
        .mem_wdata  (mem_wdata_I)   ,
        .mem_rdata  (mem_rdata_I)   ,
        .mem_ready  (mem_ready_I)
    );

    TestBed testbed(
        .clk        (clk)           ,
        .rst        (rst_n)         ,
        .addr       (DCACHE_addr)   ,
        .data       (DCACHE_wdata)  ,
        .wen        (DCACHE_wen)    ,
        .error_num  (error_num)     ,
        .duration   (duration)      ,
        .finish     (finish)
    );

`ifdef SDF
    initial $sdf_annotate(`SDFFILE, chip0);
`endif

// Initialize the data memory
    initial begin
        $display("-----------------------------------------------------\n");
        $display("START!!! Simulation Start .....\n");
        $display("-----------------------------------------------------\n");
        $readmemb (`DMEM_INIT, slow_memD.mem ); // initialize data in DMEM
        $readmemh (`IMEM_INIT, slow_memI.mem ); // initialize data in IMEM

        // waveform dump
        $fsdbDumpfile("Final.fsdb");
        $fsdbDumpvars(0,Final_tb,"+mda");
        $fsdbDumpvars;

        clk = 0;
        rst_n = 1'b1;
        #(`CYCLE*0.2) rst_n = 1'b0;
        #(`CYCLE*8.5) rst_n = 1'b1;
        // https://www.facebook.com/groups/476005556661334/permalink/566160247645864/?comment_id=579558172972738
        $display("Pull up rst_n [Timing violations before can be ignored] (Cycle %0.3f ns)", `CYCLE);

        #(`CYCLE*100000) // calculate clock cycles for all operation (you can modify it)
        $display("============================================================================");
        $display("\n           Error!!! There is something wrong with your code ...!          ");
        $display("\n                       The test result is .....FAIL                     \n");
        $display("============================================================================");
        if (testbed.curstate == 2'b0)
            $display("Possible solution: The first answer may not be correct.\n");
        if (testbed.curstate == 2'b1)
            $display("Possible solution: The clock cycles may be too small. Please modify it.\n");
         $finish;
    end

    always #(`CYCLE*0.5) clk = ~clk;

`ifdef hitRate
integer I1rh, I1wh, I2rh, I2wh, D1rh, D1wh, D2rh, D2wh;
integer I1rs, I1ws, I2rs, I2ws, D1rs, D1ws, D2rs, D2ws;
`endif
    always@(finish)
        if(finish) begin
           #(`CYCLE);
`ifdef hitRate
           $display("Hit");
           $display("     I L1 R      I L2 R      D L1 R      D L2 R");
           $display("%d %d %d %d", I1rh, I2rh, D1rh, D2rh);
           $display("     I L1 W      I L2 W      D L1 W      D L2 W");
           $display("%d %d %d %d", I1wh, I2wh, D1wh, D2wh);
           $display("Miss");
           $display("     I L1 R      I L2 R      D L1 R      D L2 R");
           $display("%d %d %d %d", I1rs, I2rs, D1rs, D2rs);
           $display("     I L1 W      I L2 W      D L1 W      D L2 W");
           $display("%d %d %d %d", I1ws, I2ws, D1ws, D2ws);
`endif
           $finish;
        end

`ifdef hitRate
    initial begin
        {I1rh, I1wh, I2rh, I2wh, D1rh, D1wh, D2rh, D2wh} = 256'd0;
    end
    always@(posedge clk) begin
        if (!rst_n) begin
        end
        else begin
            if (!chip0.I_cache_L1.proc_stall) begin
                if (chip0.I_cache_L1.proc_read) begin
                    $display("I L1 read hit");
                    I1rh <= I1rh + 1;
                end
                if (chip0.I_cache_L1.proc_write) begin
                    $display("I L1 write hit");
                    I1wh <= I1wh + 1;
                end
            end
            if (!chip0.I_cache_L2.proc_stall) begin
                if (chip0.I_cache_L2.proc_read) begin
                    $display("I L2 read hit");
                    I2rh <= I2rh + 1;
                end
                if (chip0.I_cache_L2.proc_write) begin
                    $display("I L2 write hit");
                    I2wh <= I2wh + 1;
                end
            end
            if (!chip0.D_cache_L1.proc_stall) begin
                if (chip0.D_cache_L1.proc_read) begin
                    $display("D L1 read hit");
                    D1rh <= D1rh + 1;
                end
                if (chip0.D_cache_L1.proc_write) begin
                    $display("D L1 write hit");
                    D1wh <= D1wh + 1;
                end
            end
            if (!chip0.D_cache_L2.proc_stall) begin
                if (chip0.D_cache_L2.proc_read) begin
                    $display("D L2 read hit");
                    D2rh <= D2rh + 1;
                end
                if (chip0.D_cache_L2.proc_write) begin
                    $display("D L2 write hit");
                    D2wh <= D2wh + 1;
                end
            end
        end
    end

    initial begin
        {I1rs, I1ws, I2rs, I2ws, D1rs, D1ws, D2rs, D2ws} = 256'd0;
    end
    always@(posedge chip0.I_cache_L1.proc_stall) begin
        $display("I L1 miss");
        if (chip0.I_cache_L1.proc_read)  I1rs <= I1rs + 1;
        if (chip0.I_cache_L1.proc_write) I1ws <= I1ws + 1;
    end
    always@(posedge chip0.I_cache_L2.proc_stall) begin
        $display("I L2 miss");
        if (chip0.I_cache_L2.proc_read)  I2rs <= I2rs + 1;
        if (chip0.I_cache_L2.proc_write) I2ws <= I2ws + 1;
    end
    always@(posedge chip0.D_cache_L1.proc_stall) begin
        $display("D L1 miss");
        if (chip0.D_cache_L1.proc_read)  D1rs <= D1rs + 1;
        if (chip0.D_cache_L1.proc_write) D1ws <= D1ws + 1;
    end
    always@(posedge chip0.D_cache_L2.proc_stall) begin
        $display("D L2 miss");
        if (chip0.D_cache_L2.proc_read)  D2rs <= D2rs + 1;
        if (chip0.D_cache_L2.proc_write) D2ws <= D2ws + 1;
    end
`endif
endmodule
