// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
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
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;


//=========================================
    // Note that the overall design of your RISCV includes:
    // 1. pipelined RISCV processor
    // 2. data cache
    // 3. instruction cache


    RISCV_Pipeline i_RISCV(
        // control interface
        .clk            (clk)           , 
        .rst_n          (rst_n)         ,
//----------I cache interface-------		
        .ICACHE_ren     (ICACHE_ren)    ,
        .ICACHE_wen     (ICACHE_wen)    ,
        .ICACHE_addr    (ICACHE_addr)   ,
        .ICACHE_wdata   (ICACHE_wdata)  ,
        .ICACHE_stall   (ICACHE_stall)  ,
        .ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
        .DCACHE_ren     (DCACHE_ren)    ,
        .DCACHE_wen     (DCACHE_wen)    ,
        .DCACHE_addr    (DCACHE_addr)   ,
        .DCACHE_wdata   (DCACHE_wdata)  ,
        .DCACHE_stall   (DCACHE_stall)  ,
        .DCACHE_rdata   (DCACHE_rdata)
    );
    

    cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
    );

    cache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
    );
endmodule



// Pipelined RISC-V processor
module RISCV_Pipeline(
    input           clk,
    input           rst_n,
    output          ICACHE_ren,
    output          ICACHE_wen,
    output  [29:0]  ICACHE_addr,
    output  [31:0]  ICACHE_wdata,
    input           ICACHE_stall,
    input   [31:0]  ICACHE_rdata,
    output          DCACHE_ren,
    output          DCACHE_wen,
    output  [29:0]  DCACHE_addr,
    input   [31:0]  DCACHE_rdata,
    input           DCACHE_stall,
    output  [31:0]  DCACHE_wdata
    );

// internal wire/reg definition
    integer         i;
    reg             wen_PC;
    reg             wen_IF_ID;
    reg             wen_ID_EX;
    reg             wen_EX_MEM;
    wire            wen_MEM_WB;
    assign  wen_MEM_WB = 1'b1;
    wire            MEM_regWrite;
    reg      [1:0]  forward1, forward2;
    reg forward_reg1, forward_reg2;
    reg             stallEX, flush_IF_ID, flush_MEM_WB;

// ================== IF =========================//
    reg     [31:0]  PC;
    reg    [31:0]  PCnxt;
    reg    [31:0] Pcnxt_temp, Pcnxt_temp_nxt;
    reg     problem1;
    parameter IDLE = 2'b00;
    parameter  HAZARD = 2'b01;
    parameter  BUBBLE = 2'b10;
    reg [1:0] state, state_nxt;

/*always@(*)begin
    if(ID_Instr[6:2] == 5'b11011 && ICACHE_stall)
        problem1_nxt = 0;
end*/

always@(posedge clk) begin
    if (!rst_n)
        PC <= 32'd0;
    else begin
        if (wen_PC)     // If wen_PC == 0 , PC do not change
            PC <= PCnxt;
    end
end

always@(posedge clk) begin
    if (!rst_n)
        Pcnxt_temp <= 32'd0;
    else begin
        Pcnxt_temp <= Pcnxt_temp_nxt;
    end
end



    wire    [31:0]  Instr_C, IF_Instr;
    wire     [1:0]  PCoff;
    wire    [31:0]  IF_PCpX;
assign  ICACHE_addr = PC[31:2];
// Read/Write Cycle Arrangement
assign  ICACHE_ren  = 1'b1;
assign  ICACHE_wen  = 1'b0;
assign  Instr_C     = {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]}; //Little endian
/* decomp  decIF(.o_str(IF_Instr),
              .PCoff(PCoff),
              .i_str(Instr_C),
              .PCalign(PC[1])); */
assign  IF_Instr    = Instr_C;
assign  PCoff       = 2'b10;
assign  IF_PCpX     = PC + {29'b0,PCoff,1'b0};

// ================  IF/ID registers =====================//
    reg     [31:0]  ID_PC, ID_Instr, ID_PCpX;
always@(posedge clk) begin
    if (!rst_n | flush_IF_ID) begin
        {ID_PC, ID_PCpX} <= 64'b0;
        ID_Instr <= {25'b0,7'b0010011}; // nop
    end
    else if (wen_IF_ID)
        {ID_PC, ID_Instr, ID_PCpX} <= {PC, IF_Instr, IF_PCpX};
    
end

// ================== ID =========================//
    reg     [10:0]  ctrl;
    wire    [10:0]  ID_ctrl;
    wire            c_regWrite;
    wire     [4:0]  ID_addR1, ID_addR2, ID_addRD;
    wire     [3:0]  ID_InstrALU;
    reg     [31:0]  ID_R1, ID_R2, regist [1:31], ID_imm, ID_R1_temp, ID_R2_temp;
    reg      [4:0]  WB_addRD;
    reg     [31:0]  WB_dataRD;
    wire    [31:0]  ID_PCpi;
assign  ID_addR1    = ID_Instr[19:15]; //Register 1
assign  ID_addR2    = ID_Instr[24:20]; //Register 2
assign  ID_addRD    = ID_Instr[11: 7]; //Register Write
assign  ID_InstrALU = {ID_Instr[30],ID_Instr[14:12]}; //Identify ALU instruction

//  Control Unit
// (ALUsrc ALUop[1:0]) (branch jal jalr memRead memWrite) (regWrite memToReg[1:0])
always@(*) begin // ctrlID TODO more instructions
    case(ID_Instr[6:2]) // opcode
    5'b01100:
        ctrl = 11'b010_00000_100; // R
    5'b00100:
        ctrl = 11'b110_00000_100; // I
    5'b11011:
        ctrl = 11'b000_01000_110; // J jal
    5'b11001:
        ctrl = 11'b100_00100_110; // I jalr
    5'b11000:
        ctrl = 11'b001_10000_000; // B beq ( at MEM stage )
    5'b01000:
        ctrl = 11'b100_00001_000; // S sw
    5'b00000:
        ctrl = 11'b100_00010_101; // I lw
    default:
        ctrl = 11'b000_00000_000;
    endcase
end

assign  ID_ctrl = stallEX ? 11'b0 : ctrl; // muxID

always@(*) begin // regID
    ID_R1_temp = (ID_addR1 != 5'd0) ? regist[ID_addR1] : 32'd0;
    ID_R2_temp = (ID_addR2 != 5'd0) ? regist[ID_addR2] : 32'd0;
end
always@(*) begin // regID
    ID_R1 = forward_reg1 ? WB_dataRD : ID_R1_temp;
    ID_R2 = forward_reg2 ? WB_dataRD : ID_R2_temp;
end


always@(posedge clk) begin
    if (!rst_n)
        for (i=1;i<32;i=i+1)
            regist[i] <= 32'd0;
    else if (c_regWrite && WB_addRD != 5'd0) //Write data in register
        regist[WB_addRD] <= WB_dataRD;
end

always@(*) begin // immID
    case(ID_Instr[6:2])
    5'b00000,
    5'b00100,
    5'b11001:
        ID_imm = {{21{ID_Instr[31]}},ID_Instr[30:21],ID_Instr[20]}; // I jalr
    5'b01000:
        ID_imm = {{21{ID_Instr[31]}},ID_Instr[30:25],ID_Instr[11:8],ID_Instr[7]}; // S
    5'b11000:
        ID_imm = {{20{ID_Instr[31]}},ID_Instr[7],ID_Instr[30:25],ID_Instr[11:8],1'b0}; // B
    5'b11011:
        ID_imm = {{12{ID_Instr[31]}},ID_Instr[19:12],ID_Instr[20],ID_Instr[30:21],1'b0}; // J
    default:
        ID_imm = 32'd0;
    endcase
end
assign  ID_PCpi = ID_PC + ID_imm; // addID jal and branch

//================ ID/EX registers ==================//
    reg     [10:0]  EX_ctrl;
    reg      [4:0]  EX_addR1, EX_addR2, EX_addRD;
    reg      [3:0]  EX_InstrALU;
    reg     [31:0]  EX_PCpX, EX_PCpi, EX_R1, EX_R2, EX_imm, EX_Instr;
always@(posedge clk) begin
    if (!rst_n)
        {EX_ctrl, EX_addR1, EX_addR2, EX_addRD, EX_InstrALU, EX_PCpX, EX_PCpi, EX_R1, EX_R2, EX_imm} <= 90'b0;
    else if (wen_ID_EX)
        {EX_ctrl, EX_addR1, EX_addR2, EX_addRD, EX_InstrALU, EX_PCpX, EX_PCpi, EX_R1, EX_R2, EX_imm,EX_Instr} <=
        {ID_ctrl, ID_addR1, ID_addR2, ID_addRD, ID_InstrALU, ID_PCpX, ID_PCpi, ID_R1, ID_R2, ID_imm,ID_Instr}; 
end
    //It seems that EX_addR1, EX_addR2, EX_PCpi do not need to send to EX stage. 
    //ID_R1, ID_R2 is output of register

//======================== EX ===========================//
    wire            c_ALUsrc;
    wire     [1:0]  c_ALUop;
    wire     [4:0]  shamt;
    reg     [31:0]  ALUin1, ALUin2;
    reg      [3:0]  ALUctrl;
    reg     [31:0]  EX_ALUout;
	wire	[31:0]	EX_noALUout; //modified by turknight
    reg     [31:0]  MEM_ALUout;

assign  {c_ALUsrc, c_ALUop} = EX_ctrl[10:8];
assign shamt = ALUin2[4:0];

//modified by turknight
assign EX_noALUout = EX_R2;

always@(*) begin // mux1EX
    case(forward1)
    2'd0: ALUin1 = EX_R1;
    2'd1: ALUin1 = WB_dataRD;
    2'd2: ALUin1 = MEM_ALUout;
    2'd3: ALUin1 = EX_R1;
    endcase
end
always@(*) begin // mux2EX
	case(c_ALUsrc ? 2'b11 : forward2)
	//case(forward2)
    2'd0: ALUin2 = EX_R2;
    2'd1: ALUin2 = WB_dataRD;
    2'd2: ALUin2 = MEM_ALUout;
    2'd3: ALUin2 = EX_imm;
    endcase
end

always@(*) begin // ALUctrlEX TODO
    case(c_ALUop)
    2'b00:
        ALUctrl = 4'b0010;
    2'b01:
        ALUctrl = 4'b0110;
    2'b10, 2'b11: begin
        case(EX_InstrALU)
        4'b0000:
            ALUctrl = 4'b0010; // add
        4'b1000:
            ALUctrl = 4'b0110; // sub
        4'b0111:
            ALUctrl = 4'b0000; // and
        4'b0110:
            ALUctrl = 4'b0001; // or
        4'b0010:
            ALUctrl = 4'b0111; // slt
	    4'b1100:
	        ALUctrl = 4'b0011; // xori
        4'b0100:
            ALUctrl = 4'b0011; // xori
	    4'b0001:
	        ALUctrl = 4'b0100; // SLL
	    4'b0101:
	        ALUctrl = 4'b0101; // SRL
	    4'b1101:
	        ALUctrl = 4'b1000; // SRA
        default:
            ALUctrl = 4'b0000;
        endcase
      end
    endcase
end

always@(*) begin // ALU_EX TODO
    case(ALUctrl)
    4'b0010: // add
        EX_ALUout = ALUin1 + ALUin2;
    4'b0110: // sub
        EX_ALUout = ALUin1 - ALUin2;
    4'b0000: // and
        EX_ALUout = ALUin1 & ALUin2;
    4'b0001: // or
        EX_ALUout = ALUin1 | ALUin2;
    4'b0111: // slt
	    EX_ALUout = ( $signed( ALUin1 ) < $signed( ALUin2 ) ) ? 32'd1 : 32'd0;
    4'b0011: // xor
        EX_ALUout = ALUin1 ^ ALUin2;
    4'b0100: // SLL
	    EX_ALUout = ALUin1 << shamt;
    4'b0101: //SRL
	    EX_ALUout = ALUin1 >> shamt;
    4'b1000: //SRA
	    EX_ALUout = $signed( ALUin1 ) >>> shamt;
    default:
        EX_ALUout = 32'd0;
    endcase
end

//================== EX/MEM registers =========================//
    reg      [7:0]  MEM_ctrl;
    reg      [4:0]  MEM_addRD;
    reg     [31:0]  MEM_ALUin2, MEM_PCpx, MEM_Instru;
    
always@(posedge clk) begin
    if (!rst_n)
        {MEM_ctrl, MEM_addRD, MEM_ALUin2, MEM_ALUout} <= 77'b0;
    else if (wen_EX_MEM)
        //{MEM_ctrl, MEM_addRD, MEM_ALUin2, MEM_ALUout} <= {EX_ctrl[7:0], EX_addRD, ALUin2, EX_ALUout};
		{MEM_ctrl, MEM_addRD, MEM_ALUin2, MEM_ALUout, MEM_PCpx,MEM_Instru} <= {EX_ctrl[7:0], EX_addRD, EX_noALUout, EX_ALUout, EX_PCpX,EX_Instr};
end

//==================== MEM ==================================//
    wire            c_branch, c_jal, c_jalr, c_memRead, c_memWrite;
    wire            ALUzero;
    wire            beq, PCsrc;
    wire    [31:0]  MEM_D_data;
assign  {c_branch, c_jal, c_jalr, c_memRead, c_memWrite} = MEM_ctrl[7:3];

assign  ALUzero = (MEM_ALUout == 32'd0) ? 1'b1 : 1'b0;
and     andEX(beq, c_branch, ALUzero);
or      or_EX(PCsrc, beq, c_jal);

//assign  PCnxt   = c_jalr ? MEM_ALUout : (PCsrc ? ID_PCpi : IF_PCpX); // PCpi PCpX propagate TODO

assign  DCACHE_ren   = c_memRead;
assign  DCACHE_wen   = c_memWrite;
assign  DCACHE_addr  = MEM_ALUout[31:2];
assign  DCACHE_wdata = {MEM_ALUin2[7:0],MEM_ALUin2[15:8],MEM_ALUin2[23:16],MEM_ALUin2[31:24]};
assign  MEM_D_data   = DCACHE_rdata;

//===================== MEM/WB registers =======================//
    reg      [2:0]  WB_ctrl;
    reg     [31:0]  WB_ALUout, WB_D_data, WB_PCpX, WB_Instru;
always@(posedge clk) begin
    if (!rst_n | flush_MEM_WB)
        {WB_ctrl, WB_addRD, WB_ALUout, WB_D_data} <= 72'b0;
    else if (wen_MEM_WB)
        {WB_ctrl, WB_addRD, WB_ALUout, WB_D_data, WB_PCpX, WB_Instru} <= {MEM_ctrl[2:0], MEM_addRD, MEM_ALUout, MEM_D_data, MEM_PCpx, MEM_Instru};
end

//======================== WB =============================//
    wire     [1:0]  c_memToReg;
assign  {c_regWrite, c_memToReg} = WB_ctrl;
always@(*) begin // muxWB
    case(c_memToReg)
    2'd0: WB_dataRD = WB_ALUout;
    2'd1: WB_dataRD = {WB_D_data[7:0],WB_D_data[15:8],WB_D_data[23:16],WB_D_data[31:24]};
    2'd2: WB_dataRD = WB_PCpX;
    2'd3: WB_dataRD = EX_PCpX; // jal jalr WB_PCpX propagate TODO
    endcase
end

// Data forwarding unit TODO
reg [1:0] forward3, forward4;

assign  MEM_regWrite = MEM_ctrl[2];
always@(*) begin
    if (!rst_n)
        {forward1, forward2} = 4'b0000;
    {forward1, forward2} = 4'b0000;
    {forward3, forward4, forward_reg1, forward_reg2} = 6'b000000;
    // 1. EX Data hazard
    // MEM_regWrite MEM_addRD EX_addR1 EX_addR2
    // forward1 forward2
    if( MEM_regWrite && MEM_addRD!=0 && MEM_addRD == EX_addR1 ) 
        forward1 = 2'b10;
    if( MEM_regWrite && MEM_addRD!=0 && MEM_addRD == EX_addR2 )
        forward2 = 2'b10;
    //if(MEM_Instru==WB_Instru)
    // 2. MEM Data hazard
    // c_regWrite WB_addRD EX_addR1 EX_addR2
    // forward1 forward2
    if( c_regWrite     && WB_addRD!=0   && 
        //!(MEM_regWrite && MEM_addRD!=0) && 
        //MEM_addRD      != EX_addR1      && 
        WB_addRD       == EX_addR1)begin
        forward1 = 2'b01;
    end

    if( c_regWrite     && WB_addRD!=0   && 
       // !(MEM_regWrite && MEM_addRD!=0) && 
       // MEM_addRD      != EX_addR2      && 
        WB_addRD       == EX_addR2)begin
        forward2 = 2'b01;
    end

    if( ID_Instr[6:2] == 5'b11001 && MEM_addRD == ID_addR1) //jalr
        forward3 = 2'b01;

    if(c_regWrite && ID_Instr[6:2] == 5'b11001 && WB_addRD==ID_addR1) //jalr vs jal
        forward3 = 2'b10;

    if( ID_Instr[6:2] == 5'b11000 && EX_addRD== ID_addR1 ) //beq or bne
        forward4 = 2'b01;

    if(c_regWrite && WB_addRD!=0 && WB_addRD == ID_addR1 )
        forward_reg1 = 1;
    if(c_regWrite && WB_addRD!=0 && WB_addRD == ID_addR2 )
        forward_reg2 = 1;
end
always@(*)begin
    case(state)
        IDLE:begin
            if(ID_Instr[6:2] == 5'b11011 && ( ICACHE_stall||DCACHE_stall))begin
                state_nxt <= HAZARD;
            end
            else if(ID_Instr[6:2] == 5'b11001 && ( ICACHE_stall||DCACHE_stall))
                state_nxt <= HAZARD;
            else if(ID_Instr[6:2] == 5'b11000 && ( ICACHE_stall||DCACHE_stall))
                state_nxt <= HAZARD;
            else
                state_nxt <= IDLE;
        end
        HAZARD:begin
            if(!ICACHE_stall && !DCACHE_stall)
                state_nxt <= /*BUBBLE*/IDLE;
            else
                state_nxt <= HAZARD;
        end
        BUBBLE:begin
            state_nxt <= IDLE;
        end
        default:state_nxt <= IDLE;
    endcase
end

always@(*)begin
    if(ID_Instr[6:2] == 5'b11011 && ( ICACHE_stall||DCACHE_stall))begin
        Pcnxt_temp_nxt = ID_PCpi;
    end
    else if(ID_Instr[6:2] == 5'b11001 && ( ICACHE_stall||DCACHE_stall))
        Pcnxt_temp_nxt = ID_R1 + ID_imm;
    else if(ID_Instr[6:2] == 5'b11000 && ( ICACHE_stall||DCACHE_stall))
        Pcnxt_temp_nxt = ID_PCpi;
    else
        Pcnxt_temp_nxt = Pcnxt_temp;
end

always@(posedge clk) begin
    if (!rst_n)
        state <= IDLE;
    else begin
        state <= state_nxt;
    end
end
wire [31:0] jalr_rs;
wire [31:0] branch_r1;
reg hi;
// Hazard detection / Pipeline stall unit TODO
assign  EX_memRead = EX_ctrl[4];
assign jalr_rs = (forward3==2'b01) ? MEM_ALUout : 
                 (forward3==2'b10) ? WB_dataRD  : ID_R1;
assign branch_r1 = (forward4==2'b01) ? EX_ALUout : ID_R1;
always@(*) begin
    {wen_PC, wen_IF_ID, wen_ID_EX, wen_EX_MEM} = 4'b1111;
    {flush_IF_ID, stallEX, flush_MEM_WB} = 3'b000;
    PCnxt = IF_PCpX;
    // 1. Load-Use Data Hazard
    // EX_memRead EX_addR1 EX_addR2 ID_addR1 ID_addR2
    // wen_PC wen_IF_ID stallEX
    if( EX_memRead && ( EX_addR1==ID_addR1 || EX_addR2==ID_addR2 ) ) begin
        wen_PC = 0;
        wen_IF_ID = 0;
        stallEX = 1;
    end
    /*if(ID_Instr[6:2]==5'b00000&&( IF_Instr[19:15]==ID_addR1 || IF_Instr[24:20]==ID_addR2 ) )begin
        wen_IF_ID = 0;
        wen_PC = 0;
    end*/
    // 2. Branch Hazard
    // ID_Instr
    // flush_IF_ID

    /*if(EX_ctrl[5])begin //jalr
        PCnxt = EX_ALUout;
    end*/
    
    if( ID_Instr[6:2] == 5'b11011) begin //jal
        PCnxt = ID_PCpi;
        flush_IF_ID = 1'b1;
        //PCnxt = ID_PCpi;
    end 
   
    if( ID_Instr[6:2] == 5'b11001 )begin //jalr
        flush_IF_ID = 1'b1;
        //if(forward3==2'b10)
          //  PCnxt
        PCnxt = jalr_rs + ID_imm;
    end

    if(state==HAZARD || state==BUBBLE)begin
        PCnxt = Pcnxt_temp;
        flush_IF_ID = 1'b1;
    end

    if( ID_Instr[6:2] == 5'b11000 && ID_Instr[14:12] == 3'b001 )begin //bne
       // flush_IF_ID = 1'b1;
      
        if(state==HAZARD || state==BUBBLE) begin
            if( branch_r1!=ID_R2 )begin
                flush_IF_ID = 1'b1;
                PCnxt = Pcnxt_temp;
            end
               // PCnxt = Pcnxt_temp;
            else begin
                PCnxt = IF_PCpX;
               // if(IF_Instr[6:2] == 5'b11001)
                    flush_IF_ID = 1'b0;
            end
                
        end       
         
        else begin
            if( branch_r1!=ID_R2 )begin
                flush_IF_ID = 1'b1;
                PCnxt = ID_PCpi;
            end
                //PCnxt = ID_PCpi;
            else begin
                PCnxt = IF_PCpX;
               // if(IF_Instr[6:2] == 5'b11001)
                    flush_IF_ID = 1'b0;
            end
        end
           // PCnxt = ID_PCpi;    
        /*if(IF_Instr[6:2]==5'b00100)
            flush_IF_ID = 1'b0;*/
        
    end

    if( ID_Instr[6:2] == 5'b11000 && ID_Instr[14:12] == 3'b000 )begin //beq
        //flush_IF_ID = 1'b1;
      
        if(state==HAZARD || state==BUBBLE) begin
            if( branch_r1==ID_R2 ) begin
                flush_IF_ID = 1'b1;
                PCnxt = Pcnxt_temp;
                hi=0;
            end
                //PCnxt = Pcnxt_temp;
            else begin
                PCnxt = IF_PCpX;
              //  if(IF_Instr[6:2] == 5'b11001)
                    flush_IF_ID = 1'b0;
            end
        end        
        else begin
            if( branch_r1==ID_R2 )begin
                PCnxt = ID_PCpi;
                flush_IF_ID = 1'b1;
                hi=1;
            end
               // PCnxt = ID_PCpi;
            else begin
                PCnxt = IF_PCpX;
              //  if(IF_Instr[6:2] == 5'b11001)
                    flush_IF_ID = 1'b0;
            end
        end 

        /* if(IF_Instr[6:2]==5'b00100)
            flush_IF_ID = 1'b0;*/
    end

   
    // 3. ICACHE stall
    if (ICACHE_stall) begin
        wen_PC = 1'b0;
        flush_IF_ID = 1'b1;
       // wen_IF_ID = 1'b0;
      //wen_ID_EX = 1'b0; 
        //    PCnxt = Pcnxt_temp;
        /*if(ID_Instr[6:2]==5'b00100)
            flush_IF_ID = 1'b0;*/
    end
    // 4. DCACHE stall
    // DCACHE_stall
    // wen_PC wen_IF_ID wen_ID_EX wen_EX_MEM flush_MEM_WB
    if(DCACHE_stall) begin
        wen_PC = 1'b0;
        wen_IF_ID = 1'b0;
        wen_ID_EX = 1'b0;
        wen_EX_MEM = 1'b0;
        flush_MEM_WB= 1'b1;
    end
    /*if(ID_Instr[6:2]==5'b00000)
        flush_IF_ID = 1'b1;*/
    if(ICACHE_stall&&DCACHE_stall)
        flush_IF_ID = 1'b0;

   
end
endmodule

module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
//==== parameter ==========================================
    parameter idle=3'b000;
    parameter read=3'b001;
    parameter writec =3'b010;
    parameter writemiss=3'b011;
    parameter writem=3'b100;
//==== wire/reg definition ================================
    reg [2:0]state, n_state;
    reg [31:0] proc_rdata;
    reg proc_stall;
    reg mem_read,n_mem_read;
    reg mem_write,n_mem_write;
    reg [27:0] mem_addr,n_mem_addr;
    reg [127:0] mem_wdata, n_mem_wdata;

    reg [153:0] cache[0:7];//153-valid bit 152:128-tag 127:96-4th word 95-64-third word 63:32-second word  
    reg [153:0] n_cache[0:7];
    reg hit;
    reg [2:0]index;
    reg [24:0]tag;
    reg [1:0]block_offset;
    integer i;
//==== combinational circuit ==============================
always@(*) begin
    //default value
    n_state=idle;
    case (state) //how states changes
        idle: begin
            if (proc_read==1&&proc_write==0)begin 
                if(hit==1) begin n_state=idle; end
                else begin n_state=read; end
            end
            else if (proc_write==1&&proc_read==0)begin 
                if(hit==1) begin n_state=writec; end
                else begin n_state=writemiss; end
            end
            else begin n_state=idle; end
        end
        read: begin
            if(mem_ready==1) begin n_state=idle;end
            else begin n_state=read;end
        end
        writemiss:begin
           if(mem_ready==1) begin n_state=writec;end
            else begin n_state=writemiss;end
        end
        writec:begin n_state=writem; end
        writem:begin
            if(mem_ready==1)begin n_state=idle; end
            else begin n_state=writem; end
        end
    endcase
end

always @(*) begin
//breakdown address
    block_offset=proc_addr[1:0];
    index=proc_addr[4:2];
    tag=proc_addr[29:5];
//default value
    hit=0;
    proc_rdata=32'd0;
    proc_stall=1;
    n_mem_read=mem_read;
    n_mem_write=mem_write;
    n_mem_addr=mem_addr;
    n_mem_wdata=mem_wdata;
    n_cache[0]=cache[0];
    n_cache[1]=cache[1];
    n_cache[2]=cache[2];
    n_cache[3]=cache[3];
    n_cache[4]=cache[4];
    n_cache[5]=cache[5];
    n_cache[6]=cache[6];
    n_cache[7]=cache[7];
    case (state)
        idle:begin
            n_mem_write=0;
            if(proc_read==1&&proc_write==0)begin
                if(tag==cache[index][152:128]&&cache[index][153]==1)begin 
                    proc_stall=0;
                    hit=1;
                    if(block_offset==2'b00)begin proc_rdata= cache[index][31:0]; end
                    else if(block_offset==2'b01)begin proc_rdata= cache[index][63:32]; end
                    else if(block_offset==2'b10)begin proc_rdata= cache[index][95:64]; end
                    else begin proc_rdata= cache[index][127:96]; end 
                    end
                else begin 
                    hit=0;
                    proc_stall=1;
                    n_mem_read=1;
                    n_mem_addr={tag,index};
                    end
            end
            else if(proc_write==1&&proc_read==0)begin//write
                if(tag==cache[index][152:128]&&cache[index][153]==1)begin //write hit
                    hit=1;
                    proc_stall=1;
                    end
                else begin //write miss
                    hit=0;
                    proc_stall=1;
                    n_mem_read=1;
                    n_mem_addr={tag,index};
                end
            end
            else
            begin
                proc_stall=0;
            end
        end
        read:begin//load data from memory
            proc_stall=1; 
            if(mem_ready==1)begin 
            n_cache[index][127:0]=mem_rdata;//modify data(4 words)
            n_cache[index][152:128]=tag;//modify tag
            n_cache[index][153]=1;
            n_mem_read=0;
            end
        end
        writemiss:begin//load data from memory,identical to read state
            proc_stall=1;
            if(mem_ready==1)begin 
            n_cache[index][127:0]=mem_rdata;//modify data(4 words)
            n_cache[index][152:128]=tag;//modify tag
            n_cache[index][153]=1;
            n_mem_read=0;
            end
        end
        writec:begin
            proc_stall=1;
            if(block_offset==2'b00)begin n_cache[index][153:0]={cache[index][153:32],proc_wdata}; end
            else if(block_offset==2'b01)begin n_cache[index][153:0]={cache[index][153:64],proc_wdata,cache[index][31:0]}; end
            else if(block_offset==2'b10)begin n_cache[index][153:0]={cache[index][153:96],proc_wdata,cache[index][63:0]}; end
            else if(block_offset==2'b11)begin n_cache[index][153:0]={cache[index][153:128],proc_wdata,cache[index][95:0]}; end 
        end
        writem:begin
            if(mem_ready==1)begin
                proc_stall=0;
                n_mem_write=0;
            end
            else begin
            proc_stall=1;
            n_mem_write=1;
            n_mem_addr={tag,index};
            n_mem_wdata=cache[index][127:0];
            end
        end
    endcase
end
//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        state<=idle;
        mem_read<=0;
        mem_write<=0;
        mem_addr<=28'd0;
        mem_wdata<=128'd0;
        for (i =0 ;i<8 ;i=i+1 ) begin
        cache[i]<=154'd0;
        end
    end
    else begin //sequential pass
        state<=n_state;
        mem_read<=n_mem_read;
        mem_write<=n_mem_write;
        mem_addr<=n_mem_addr;
        mem_wdata<=n_mem_wdata;
        for (i =0 ;i<8 ;i=i+1 ) begin
            cache[i]<=n_cache[i];
        end
    end
end

endmodule
