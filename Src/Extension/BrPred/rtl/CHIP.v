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
    

    cache_D D_cache(
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

    cache_I I_cache(
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
    wire            EX_memRead;
    wire            MEM_regWrite;
    reg      [1:0]  forward1, forward2;
    reg             forward_reg1, forward_reg2;
    reg             flush_IF_ID, stallEX, flush_MEM_WB;

//========================== IF ==========================//
    reg     [31:0]  PC;
    wire     [31:0]  PCnxt;

always@(posedge clk) begin
    if (!rst_n)
        PC <= 32'd0;
    else begin
        if (wen_PC)     // If wen_PC == 0 , PC do not change
            PC <= PCnxt;
    end
end

    wire    [31:0]  IF_Instr;
    wire    [31:0]  IF_PCpX;
assign  ICACHE_ren  = 1'b1;
assign  ICACHE_wen  = 1'b0;
assign  ICACHE_addr = PC[31:2];
assign  IF_Instr    = {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]}; //Little endian
assign  IF_PCpX     = PC + 32'd4;

//=================== IF/ID registers ====================//
    reg     [31:0]  ID_PC, ID_Instr, ID_PCpX;
always@(posedge clk) begin
    if (!rst_n | flush_IF_ID) begin
        {ID_PC, ID_PCpX} <= 64'b0;
        ID_Instr <= {25'b0,7'b0010011}; // nop
    end
    else if (wen_IF_ID)
        {ID_PC, ID_Instr, ID_PCpX} <= {PC, IF_Instr, IF_PCpX};
end

//========================== ID ==========================//
    reg     [10:0]  ctrl;
    wire    [10:0]  ID_ctrl;
    wire            c_regWrite;
    wire     [4:0]  ID_addR1, ID_addR2, ID_addRD;
    wire     [3:0]  ID_InstrALU;
    reg     [31:0]  ID_R1_temp, ID_R2_temp, ID_R1, ID_R2, regist [1:31], ID_imm;
    reg      [4:0]  WB_addRD;
    reg     [31:0]  WB_dataRD;
    wire    [31:0]  ID_PCpi;
    wire    [31:0]  branch_jump_address;

assign  ID_addR1    = ID_Instr[19:15]; //Register 1
assign  ID_addR2    = ID_Instr[24:20]; //Register 2
assign  ID_addRD    = ID_Instr[11: 7]; //Register Write
assign  ID_InstrALU = {ID_Instr[30],ID_Instr[14:12]}; //Identify ALU instruction

//  Control Unit
// (ALUsrc ALUop[1:0]) (branch jal jalr memRead memWrite) (regWrite memToReg[1:0])
always@(*) begin // ctrlID
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
        ctrl = 11'b001_10000_000; // B beq 
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
always@(*) begin // regID but have to forwarding due to 3 cycles
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
assign  branch_jump_address = ctrl[5] ? (ID_R1 + ID_imm) : ID_PCpi;

//=================== ID/EX registers ====================//
    reg     [10:0]  EX_ctrl;
    reg      [4:0]  EX_addR1, EX_addR2, EX_addRD;
    reg      [3:0]  EX_InstrALU, EX_Instru_temp;
    reg     [31:0]  EX_PCpX, EX_PCpi, EX_R1, EX_R2, EX_imm;
always@(posedge clk) begin
    if (!rst_n)
        {EX_ctrl, EX_addR1, EX_addR2, EX_addRD, EX_Instru_temp, EX_PCpX, EX_PCpi, EX_R1, EX_R2, EX_imm} <= 90'b0;
    else if (wen_ID_EX)
        {EX_ctrl, EX_addR1, EX_addR2, EX_addRD, EX_Instru_temp, EX_PCpX, EX_PCpi, EX_R1, EX_R2, EX_imm} <=
        {ID_ctrl, ID_addR1, ID_addR2, ID_addRD, ID_InstrALU, ID_PCpX, ID_PCpi, ID_R1, ID_R2, ID_imm};
end
    //ID_R1, ID_R2 is output of register

//========================== EX ==========================//
    wire            c_ALUsrc;
    wire     [1:0]  c_ALUop;
    wire     [4:0]  shamt;
    reg     [31:0]  ALUin1, ALUin2;
    reg      [3:0]  ALUctrl;
    reg     [31:0]  EX_ALUout;
    reg     [31:0]  MEM_ALUout;

assign  {c_ALUsrc, c_ALUop} = EX_ctrl[10:8];
assign  shamt = ALUin2[4:0];

always@(*)begin
    if( EX_ctrl[10:9] == 3'b11 ) // I-type isn't determined by Instr[30] , but SRAI.
        if( EX_Instru_temp == 4'b1101 ) //SRAI
            EX_InstrALU = EX_Instru_temp;
        else
            EX_InstrALU = { 1'b0,EX_Instru_temp[2:0] };
    else
        EX_InstrALU = EX_Instru_temp;
end

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
    2'd0: ALUin2 = EX_R2;
    2'd1: ALUin2 = WB_dataRD;
    2'd2: ALUin2 = MEM_ALUout;
    2'd3: ALUin2 = EX_imm;
    endcase
end

always@(*) begin // ALUctrlEX
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

always@(*) begin // ALU_EX
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

//=================== EX/MEM registers ===================//
    reg      [7:0]  MEM_ctrl;
    reg      [4:0]  MEM_addRD;
    reg     [31:0]  MEM_R2, MEM_PCpX;
always@(posedge clk) begin
    if (!rst_n)
        {MEM_ctrl, MEM_addRD, MEM_R2, MEM_ALUout} <= 77'b0;
    else if (wen_EX_MEM)
        {MEM_ctrl, MEM_addRD, MEM_R2, MEM_ALUout, MEM_PCpX} <= {EX_ctrl[7:0], EX_addRD, EX_R2, EX_ALUout, EX_PCpX};
end

//========================= MEM ==========================//
    wire            c_branch, c_jal, c_jalr, c_memRead, c_memWrite;
    wire            ALUzero;
    wire            beq, PCsrc;
    wire    [31:0]  MEM_D_data;
assign  {c_branch, c_jal, c_jalr, c_memRead, c_memWrite} = MEM_ctrl[7:3];

assign  ALUzero = (MEM_ALUout == 32'd0) ? 1'b1 : 1'b0;
and     andEX(beq, c_branch, ALUzero);
or      or_EX(PCsrc, beq, c_jal);

assign  DCACHE_ren   = c_memRead;
assign  DCACHE_wen   = c_memWrite;
assign  DCACHE_addr  = MEM_ALUout[31:2];
assign  DCACHE_wdata = {MEM_R2[7:0],MEM_R2[15:8],MEM_R2[23:16],MEM_R2[31:24]};
assign  MEM_D_data   = DCACHE_rdata;

//=================== MEM/WB registers ===================//
    reg      [2:0]  WB_ctrl;
    reg     [31:0]  WB_ALUout, WB_D_data, WB_PCpX, WB_Instru;
always@(posedge clk) begin
    if (!rst_n | flush_MEM_WB)
        {WB_ctrl, WB_addRD, WB_ALUout, WB_D_data} <= 72'b0;
    else
        {WB_ctrl, WB_addRD, WB_ALUout, WB_D_data, WB_PCpX} <= {MEM_ctrl[2:0], MEM_addRD, MEM_ALUout, MEM_D_data, MEM_PCpX};
end

//========================== WB ==========================//
    wire     [1:0]  c_memToReg;
assign  {c_regWrite, c_memToReg} = WB_ctrl;

always@(*) begin // muxWB
    case(c_memToReg)
    2'd0: WB_dataRD = WB_ALUout;
    2'd1: WB_dataRD = {WB_D_data[7:0],WB_D_data[15:8],WB_D_data[23:16],WB_D_data[31:24]};
    2'd3,
    2'd2: WB_dataRD = WB_PCpX;
    endcase
end

// Data forwarding unit
wire    [31:0]  jalr_rs;
wire    [31:0]  branch_r1, branch_r2;
assign  jalr_rs     = (MEM_addRD == ID_addR1) ? MEM_ALUout : ID_R1;
assign  branch_r1   = ( EX_addRD == ID_addR1) ?  EX_ALUout : ID_R1;
assign  branch_r2   = ( EX_addRD == ID_addR2) ?  EX_ALUout : ID_R2;
assign  MEM_regWrite = MEM_ctrl[2];
/*always@(*) begin
    {forward1, forward2, forward_reg1, forward_reg2} = 6'b000000;
    // 1. EX Data hazard
    if( MEM_regWrite && MEM_addRD!=0 && MEM_addRD == EX_addR1 )
        forward1 = 2'b10;
    if( MEM_regWrite && MEM_addRD!=0 && MEM_addRD == EX_addR2 )
        forward2 = 2'b10;

    // 2. MEM Data hazard
    if(c_regWrite && WB_addRD!=0 && WB_addRD == EX_addR1 )
        forward1 = 2'b01;
    if(c_regWrite && WB_addRD!=0 && WB_addRD == EX_addR2 )
        forward2 = 2'b01;

    // data write in register need one cycle, need forwarding
    if(c_regWrite && WB_addRD!=0 && WB_addRD == ID_addR1 )
        forward_reg1 = 1'b1;
    if(c_regWrite && WB_addRD!=0 && WB_addRD == ID_addR2 )
        forward_reg2 = 1'b1;
end*/
always@(*)begin
    forward1 = 2'b00;
    if(c_regWrite && WB_addRD!=0 && WB_addRD == EX_addR1 )
        forward1 = 2'b01;
    if( MEM_regWrite && MEM_addRD!=0 && MEM_addRD == EX_addR1 )
        forward1 = 2'b10;
end
always@(*)begin
    forward2 = 2'b00;
    if(c_regWrite && WB_addRD!=0 && WB_addRD == EX_addR2 )
        forward2 = 2'b01;
    if( MEM_regWrite && MEM_addRD!=0 && MEM_addRD == EX_addR2 )
        forward2 = 2'b10;
end
always@(*)begin
    forward_reg1 = 0;
    if(c_regWrite && WB_addRD!=0 && WB_addRD == ID_addR1 )
        forward_reg1 = 1'b1;
end
always@(*)begin
    forward_reg2 = 0;
    if(c_regWrite && WB_addRD!=0 && WB_addRD == ID_addR2 )
        forward_reg2 = 1'b1;
end
//========================= FSM ==========================//
    parameter  IDLE   = 1'b0;
    parameter  HAZARD = 1'b1;
    reg     state, state_nxt;

    wire    problem1;
    wire    Cache_stall;
    wire    branch_jump;

    assign  problem1    = state;
    assign  Cache_stall = ICACHE_stall | DCACHE_stall; // There will be some problem when each cache is stalled.
    assign  branch_jump = ctrl[6] | ctrl[5] | ctrl[7]; // ctrl[6] is jal, ctrl[5] is jalr, ctrl[7] is branch

always@(*)begin
    case(state)
        IDLE:begin
            if( branch_jump && Cache_stall )
                state_nxt = HAZARD;
            else
                state_nxt = IDLE;
        end
        HAZARD:begin
            if( !Cache_stall )
                state_nxt = IDLE;
            else
                state_nxt = HAZARD;
        end
    endcase
end

always@(posedge clk) begin
    if (!rst_n)
        state <= IDLE;
    else
        state <= state_nxt;
end

//================== Store PCnxt Value ===================//
reg     [31:0]  Pcnxt_temp, Pcnxt_temp_nxt;

always@(*)begin
    if( branch_jump && Cache_stall )
        Pcnxt_temp_nxt = branch_jump_address;
    else
        Pcnxt_temp_nxt = Pcnxt_temp;
end

always@(posedge clk) begin
    if (!rst_n)
        Pcnxt_temp <= 32'd0;
    else
        Pcnxt_temp <= Pcnxt_temp_nxt;
end
//========================================================//
reg [31:0] PC_normal;
reg predict_miss;
// Hazard detection / Pipeline stall unit
assign  EX_memRead  = EX_ctrl[4];
always@(*) begin
    {wen_PC, wen_IF_ID, wen_ID_EX, wen_EX_MEM} = 4'b1111;
    {flush_IF_ID, stallEX, flush_MEM_WB} = 3'b000;
   // PC_normal = IF_PCpX;
    // 1. Load-Use Data Hazard
    if( EX_memRead && ( EX_addRD==ID_addR1 || EX_addRD==ID_addR2 ) ) begin
        wen_PC = 1'b0;
        wen_IF_ID = 1'b0;
        stallEX = 1'b1;
    end

    // 2. Branch Hazard

    if( ctrl[6] ) begin // jal
       // PC_normal = ID_PCpi;
        flush_IF_ID = 1'b1;
    end

    if( ctrl[5] )begin // jalr
      //  PC_normal = jalr_rs + ID_imm;
        flush_IF_ID = 1'b1;
    end

    if( problem1 )begin
        //PC_normal = Pcnxt_temp;
        flush_IF_ID = 1'b1;
    end

    /*if( ctrl[7] )begin // beq or bne
        if( (branch_r1==ID_R2) ^ ID_Instr[12] ) begin
            PC_normal = problem1 ? Pcnxt_temp : ID_PCpi;
            flush_IF_ID = 1'b1;
        end
        else begin
            PC_normal = IF_PCpX;
            flush_IF_ID = 1'b0;
        end
    end*/
    if( ctrl[7] ) begin
        flush_IF_ID = predict_miss;
    end

    if(predict_miss)
        flush_IF_ID = 1'b1;

    // 3. ICACHE stall
    if (ICACHE_stall) begin
        wen_PC = 1'b0;
        flush_IF_ID = 1'b1;
    end
    // 4. DCACHE stall
    if (DCACHE_stall) begin
        wen_PC = 1'b0;
        wen_IF_ID = 1'b0;
        wen_ID_EX = 1'b0;
        wen_EX_MEM = 1'b0;
        flush_MEM_WB = 1'b1;
    end
end

always@(*)begin
    PC_normal = IF_PCpX;
    if( ctrl[6] ) begin // jal
        PC_normal = ID_PCpi;
       // flush_IF_ID = 1'b1;
    end
    else if( ctrl[5] )begin // jalr
        PC_normal = jalr_rs + ID_imm;
      //  flush_IF_ID = 1'b1;
    end
    else if( problem1 )begin
        PC_normal = Pcnxt_temp;
        //flush_IF_ID = 1'b1;
    end
end
//================ BPU ======================//
    reg [1:0] BPU_state, BPU_state_nxt;
    reg [31:0] ID_normal;
    reg  BrPre, ID_BrPre;  // 1 = taken, 0 = not taken
    wire branch; // branch = 1 when IF stage is bne or bqe 
    wire [31:0] branch_immdiate, branch_address, PC_after_branch, really_address, really_address2; 
    reg  [31:0] ID_branch_address;

    parameter STRONG_TAKEN = 2'b00;
    parameter WEAK_TAKEN = 2'b01;
    parameter STRONG_NOT_TAKEN = 2'b10;
    parameter WEAK_NOT_TAKEN = 2'b11;
   
    assign  branch = ( IF_Instr[6:2] == 5'b11000 ) ? 1'b1 : 1'b0; 
    assign  branch_immdiate = {{20{IF_Instr[31]}},IF_Instr[7],IF_Instr[30:25],IF_Instr[11:8],1'b0};
    assign  branch_address  = PC + branch_immdiate;
    assign  PC_after_branch = ( BrPre & branch ) ? branch_address : PC_normal;
    assign  really_address  = ctrl[7] ? ID_branch_address : ID_normal;
    assign  really_address2 = ID_BrPre ? ID_PCpX :  really_address;
    assign  PCnxt = predict_miss ? really_address2 : PC_after_branch;

always@(*)begin
    case( BPU_state )
        STRONG_TAKEN:begin
            if( predict_miss )
                BPU_state_nxt = WEAK_TAKEN;
            else
                BPU_state_nxt = STRONG_TAKEN;
        end
        WEAK_TAKEN:begin
            if( predict_miss )
                BPU_state_nxt = STRONG_NOT_TAKEN;
            else
                BPU_state_nxt = STRONG_TAKEN;
       end
        WEAK_NOT_TAKEN:begin
            if( predict_miss )
                BPU_state_nxt = STRONG_TAKEN;
            else
                BPU_state_nxt = STRONG_NOT_TAKEN;
       end
        STRONG_NOT_TAKEN:begin
            if( predict_miss )
                BPU_state_nxt = WEAK_NOT_TAKEN;
            else
                BPU_state_nxt = STRONG_NOT_TAKEN;
        end
        default: BPU_state_nxt = STRONG_NOT_TAKEN;
    endcase
end

always@(*)begin
    case( BPU_state )
        STRONG_TAKEN:begin
           BrPre = 1;
        end
        WEAK_TAKEN:begin
           BrPre = 1;
        end
        WEAK_NOT_TAKEN:begin
          BrPre = 0;
        end
        STRONG_NOT_TAKEN:begin
           BrPre = 0;
        end
        default: BrPre = 1;
    endcase
end

always@(*)begin
    predict_miss = 0;
    if( ctrl[7] ) begin
        if( ID_BrPre && ID_Instr[14:12]==3'b000 )begin //beq
            if(branch_r1==branch_r2)
                predict_miss = 0;
            else
                predict_miss = 1;
        end 
        else if( !ID_BrPre && ID_Instr[14:12]==3'b000)begin
            if(branch_r1!=branch_r2)
                predict_miss = 0;
            else
                predict_miss = 1;
        end
        else if( ID_BrPre && ID_Instr[14:12]==3'b001)begin //bne
            if(branch_r1!=branch_r2)
                predict_miss = 0;
            else
                predict_miss = 1;
        end
        else if( !ID_BrPre && ID_Instr[14:12]==3'b001)begin
            if(branch_r1==branch_r2)
                predict_miss = 0;
            else
                predict_miss = 1;
        end
    end
end

always@( posedge clk )begin
    if(!rst_n)
        BPU_state <= STRONG_TAKEN;
    else if( ctrl[7] ) // state only change when bne/bqe in ID stage
        BPU_state <= BPU_state_nxt;
end

always@( posedge clk )begin
    if(!rst_n) begin
        ID_branch_address <= 32'd0;
        ID_BrPre <= 0;
    end
    else begin
        ID_BrPre <= BrPre;
        ID_normal <= PC_normal;
        ID_branch_address <= branch_address;
    end
end

endmodule




module cache_D(
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

//==== wire/reg definition ================================
    reg            mem_read, mem_write;
    reg     [27:0] mem_addr;
    reg    [127:0] mem_wdata;

    reg      [1:0] state_w;
    reg      [1:0] state;
    reg      [3:0] valid1, valid2;
    reg      [3:0] dirty1, dirty2;
    reg            be_dirty;
    reg      [3:0] lru;             // 0:data1 1:data2
    reg     [29:4] tag1    [0:3], tag2    [0:3];
    reg    [127:0] data1   [0:3], data2   [0:3];
    wire           hit1, hit2;
    wire     [3:2] set = proc_addr[3:2];
    wire    [6:0] temp;
//==== combinational circuit ==============================
    localparam S_IDLE = 2'd0;
    localparam S_WB   = 2'd3;
    localparam S_RD   = 2'd2;
assign temp = (proc_addr << 5);
assign proc_stall = ~(hit1 | hit2) && (proc_read | proc_write);
assign proc_rdata = hit1
                  ? data1[set][temp+:32]
                  : data2[set][temp+:32];
assign hit1 = valid1[set] & (tag1[set] == proc_addr[29:4]);
assign hit2 = valid2[set] & (tag2[set] == proc_addr[29:4]);

always@(*) begin
    case (state)
    S_IDLE:
        if (proc_stall)
            state_w = (~lru[set] & dirty1[set] | lru[set] & dirty2[set]) ? S_WB : S_RD;
        else
            state_w = S_IDLE;
    S_WB:
        state_w = mem_ready ? S_RD : S_WB;
    S_RD:
        state_w = mem_ready ? S_IDLE : S_RD;
    default:
        state_w = S_IDLE;
    endcase
end
//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        mem_read   <= 0;
        mem_write  <= 0;
        state   <= S_IDLE;
        valid1  <= 4'b0;
        valid2  <= 4'b0;
        dirty1  <= 4'b0;
        dirty2  <= 4'b0;
        lru     <= 4'b0;
    end
    else begin
        state   <= state_w;
        be_dirty<= (proc_read) ? 1'b0 : (proc_write) ? 1'b1 : be_dirty;
        case (state)
        S_IDLE: begin
            if (proc_read && ~(hit1 | hit2)) begin
                if (~lru[set] & dirty1[set]) begin
                    mem_write  <= 1;
                    mem_addr   <= {tag1[set],set};
                    mem_wdata  <= data1[set];
                end else if (lru[set] & dirty2[set]) begin
                    mem_write  <= 1;
                    mem_addr   <= {tag2[set],set};
                    mem_wdata  <= data2[set];
                end else begin
                    mem_read   <= 1;
                    mem_addr   <= proc_addr[29:2];
                end
            end
            else if (proc_write) begin
                if (hit1) begin
                    dirty1[set] <= 1;
                    data1[set][temp+:32] <= proc_wdata;
                end else if (hit2) begin
                    dirty2[set] <= 1;
                    data2[set][temp+:32] <= proc_wdata;
                end else if (~lru[set] & dirty1[set]) begin
                    mem_write  <= 1;
                    mem_addr   <= {tag1[set],set};
                    mem_wdata  <= data1[set];
                end else if (lru[set] & dirty2[set]) begin
                    mem_write  <= 1;
                    mem_addr   <= {tag2[set],set};
                    mem_wdata  <= data2[set];
                end else begin
                    mem_read   <= 1;
                    mem_addr   <= proc_addr[29:2];
                end
            end
            if ((proc_read | proc_write) & (hit1 | hit2))
                lru[set] <= hit1;
        end
        S_WB:
            if (mem_ready) begin
                mem_read   <= 1;
                mem_write  <= 0;
                mem_addr   <= proc_addr[29:2];
            end
        S_RD:
            if (mem_ready) begin
                mem_read   <= 0;
                if (~lru[set]) begin
                    valid1[set] <= 1;
                    dirty1[set] <= be_dirty;
                    tag1  [set] <= proc_addr[29:4];
                    data1 [set] <= mem_rdata;
                end else begin
                    valid2[set] <= 1;
                    dirty2[set] <= be_dirty;
                    tag2  [set] <= proc_addr[29:4];
                    data2 [set] <= mem_rdata;
                end
            end
        endcase
    end
end
endmodule

module cache_I(
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

//==== wire/reg definition ================================
    reg            mem_read;
    reg     [27:0] mem_addr;
    reg    [127:0] mem_wdata;

    reg            state_w;
    reg            state;
    reg      [3:0] valid1, valid2;
    reg      [3:0] lru;             // 0:data1 1:data2
    reg     [29:4] tag1    [0:3], tag2    [0:3];
    reg    [127:0] data1   [0:3], data2   [0:3];
    wire           hit1, hit2;
    wire  [6:0] temp;
    wire     [3:2] set = proc_addr[3:2];
//==== combinational circuit ==============================
    localparam S_IDLE = 1'b0;
    localparam S_RD   = 1'b1;
assign temp = (proc_addr[1:0] << 5);
assign proc_stall = ~(hit1 | hit2) && proc_read;
assign proc_rdata = hit1
                  ? data1[set][temp+:32]
                  : data2[set][temp+:32];
assign mem_write = 1'b0;
assign hit1 = valid1[set] & (tag1[set] == proc_addr[29:4]);
assign hit2 = valid2[set] & (tag2[set] == proc_addr[29:4]);

always@(*) begin
    case (state)
    S_IDLE:
        state_w = proc_stall;
    S_RD:
        state_w = ~mem_ready;
    endcase
end
//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        mem_read   <= 0;
        state   <= S_IDLE;
        valid1  <= 4'b0;
        valid2  <= 4'b0;
        lru     <= 4'b0;
    end
    else begin
        state   <= state_w;
        case (state)
        S_IDLE: begin
            if (proc_stall) begin
                mem_read   <= 1;
                mem_addr   <= proc_addr[29:2];
            end
            if (proc_read & (hit1 | hit2))
                lru[set] <= hit1;
        end
        S_RD:
            if (mem_ready) begin
                mem_read   <= 0;
                if (~lru[set]) begin
                    valid1[set] <= 1;
                    tag1  [set] <= proc_addr[29:4];
                    data1 [set] <= mem_rdata;
                end else begin
                    valid2[set] <= 1;
                    tag2  [set] <= proc_addr[29:4];
                    data2 [set] <= mem_rdata;
                end
            end
        endcase
    end
end
endmodule