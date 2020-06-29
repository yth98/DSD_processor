// this is a test bench feeds initial instruction and data
// the processor output is not verified

`timescale 1 ns/10 ps

`define CYCLE 10 // You can modify your clock frequency

`define DMEM_INIT "D_mem"
`define SDFFILE   "./CHIP_syn.sdf"	// Modify your SDF file name

// For different condition (I_mem, TestBed)
`ifdef L2Cache
	`define IMEM_INIT "I_mem_L2Cache"
	`include "./TestBed_L2Cache.v"
`endif
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
	
	wire [29:0]	DCACHE_addr;
	wire [31:0]	DCACHE_wdata;
	wire 		DCACHE_wen;
	
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
	
	integer total_cycle,instruction_cache_stall_cycle,data_cache_stall_cycle,L2_data_cache_stall_cycle; 
	integer	miss_rd_I,total_rd_I,stall_cycle_rd_I,stall_cycle_rd_D,miss_wr_D,total_wr_D,stall_cycle_wr_D,total_rd_D,miss_rd_D,stall;
	initial begin
		total_cycle = 0; 
		instruction_cache_stall_cycle = 0;
		data_cache_stall_cycle=0;
		L2_data_cache_stall_cycle=0;
		stall=0;
		
		miss_rd_I=0;
		miss_wr_D=0;
		miss_rd_D=0;
		total_rd_I=0;
		total_rd_D=0;
		total_wr_D=0;
		stall_cycle_rd_I=0;
		stall_cycle_rd_D=0;
		stall_cycle_wr_D=0;
	end
	
	
// Initialize the data memory
	initial begin
		$display("-----------------------------------------------------\n");
	 	$display("START!!! Simulation Start .....\n");
	 	$display("-----------------------------------------------------\n");
		$readmemb (`DMEM_INIT, slow_memD.mem ); // initialize data in DMEM
		$readmemh (`IMEM_INIT, slow_memI.mem ); // initialize data in IMEM

		// waveform dump
	    // $dumpfile("Final.vcd");
	    // $dumpvars;
	    $fsdbDumpfile("Finalsuccess.fsdb");			
		$fsdbDumpvars(0,Final_tb,"+mda");
		$fsdbDumpvars;
	
		clk = 0;
		rst_n = 1'b1;
		#(`CYCLE*0.2) rst_n = 1'b0;
		#(`CYCLE*8.5) rst_n = 1'b1;
		//#(`CYCLE*0.5 );
		/*
		while(~finish)	begin
		if(chip0.ICACHE_stall) begin
			if(chip0.ICACHE_ren)	begin
				if(stall!=0)	miss_rd_I=miss_rd_I+1;
				total_rd_I = total_rd_I + 1;
				stall = 0;
			end
			else begin
			    stall = 1;
			    stall_cycle_rd_I = stall_cycle_rd_I + 1;
			end
		end
			#(`CYCLE);
		end
		
		while(~finish)	begin
		if(chip0.DCACHE_stall) begin
			if(chip0.DCACHE_ren)	begin
				if(stall!=0)	miss_rd_D=miss_rd_D+1;
				total_rd_D = total_rd_D + 1;
				stall = 0;
			end
			else begin
			    stall = 1;
			    stall_cycle_rd_D = stall_cycle_rd_D + 1;
			end
		end
			#(`CYCLE);
		end
		
		while(~finish)	begin
		if(chip0.DCACHE_stall) begin
			if(chip0.DCACHE_wen)	begin
				if(stall!=0)	miss_wr_D=miss_wr_D+1;
				total_wr_D = total_wr_D + 1;
				stall = 0;
			end
			else begin
			    stall = 1;
			    stall_cycle_wr_D = stall_cycle_wr_D + 1;
			end
		end
			#(`CYCLE);
		end
		*/
		/*
		while(~finish)	begin
			if(chip0.ICACHE_stall)	begin
				instruction_cache_stall_cycle=instruction_cache_stall_cycle+1;
			end
			if(chip0.DCACHE_stall)	begin
				data_cache_stall_cycle=data_cache_stall_cycle+1;
			end
			//if(chip0.L2_stall_D)	begin
				//L2_data_cache_stall_cycle=L2_data_cache_stall_cycle+1;
			//end
			total_cycle = total_cycle + 1;
			#(`CYCLE);
		end	
*/		
		//$display("L2_data_cache_stall_cycle: %d",L2_data_cache_stall_cycle);
		
		/*
		$display("miss_rd_I:",miss_rd_I);
		$display("miss_wr_D:",miss_wr_D);
		$display("miss_rd_D:",miss_rd_D);
		$display("total_rd_I:",total_rd_I);
		$display("total_rd_D:",total_rd_D);
		$display("total_wr_I:",total_wr_D);
		$display("stall_cycle_rd_I:",stall_cycle_rd_I);
		$display("stall_cycle_rd_D:",stall_cycle_rd_D); 
		$display("stall_cycle_wr_D:",stall_cycle_wr_D);
		*/
		$display("total data_cache_stall_cycle: %d",data_cache_stall_cycle);
		$display("total instruction_cache_stall_cycle: %d",instruction_cache_stall_cycle);
		$display( "total cycle: %d", total_cycle );
		#(`CYCLE*10000) // calculate clock cycles for all operation (you can modify it)
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
	
	always@(finish)
	    if(finish)
	       #(`CYCLE) $finish;		   
	
endmodule
