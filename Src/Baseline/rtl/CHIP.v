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
    reg             stallEX, flush_IF_ID, flush_MEM_WB;

// IF
    reg     [31:0]  PC;
    wire    [31:0]  PCnxt;
always@(posedge clk) begin
    if (!rst_n)
        PC <= 32'd0;
    else begin
        if (wen_PC)
            PC <= PCnxt;
    end
end

    wire    [31:0]  Instr_C, IF_Instr;
    wire     [1:0]  PCoff;
    wire    [31:0]  IF_PCpX;
assign  ICACHE_addr = PC[31:2];
// Read/Write Cycle Arrangement
assign  ICACHE_ren  = 1'b1;
assign  ICACHE_wen  = 1'b0;
assign  Instr_C     = {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
/* decomp  decIF(.o_str(IF_Instr),
              .PCoff(PCoff),
              .i_str(Instr_C),
              .PCalign(PC[1])); */
assign  IF_Instr    = Instr_C;
assign  PCoff       = 2'b10;
assign  IF_PCpX     = PC + {29'b0,PCoff,1'b0};

// IF/ID registers
    reg     [31:0]  ID_PC, ID_Instr, ID_PCpX;
always@(posedge clk) begin
    if (!rst_n | flush_IF_ID) begin
        {ID_PC, ID_PCpX} <= 64'b0;
        ID_Instr <= {25'b0,7'b0010011}; // nop
    end
    else if (wen_IF_ID)
        {ID_PC, ID_Instr, ID_PCpX} <= {PC, IF_Instr, IF_PCpX};
end

// ID
    reg     [10:0]  ctrl;
    wire    [10:0]  ID_ctrl;
    wire            c_regWrite;
    wire     [4:0]  ID_addR1, ID_addR2, ID_addRD;
    wire     [3:0]  ID_InstrALU;
    reg     [31:0]  ID_R1, ID_R2, regist [1:31], ID_imm;
    reg      [4:0]  WB_addRD;
    reg     [31:0]  WB_dataRD;
    wire    [31:0]  ID_PCpi;
assign  ID_addR1    = ID_Instr[19:15];
assign  ID_addR2    = ID_Instr[24:20];
assign  ID_addRD    = ID_Instr[11: 7];
assign  ID_InstrALU = {ID_Instr[30],ID_Instr[14:12]};

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
    ID_R1 = (ID_addR1 != 5'd0) ? regist[ID_addR1] : 32'd0;
    ID_R2 = (ID_addR2 != 5'd0) ? regist[ID_addR2] : 32'd0;
end
always@(posedge clk) begin
    if (!rst_n)
        for (i=1;i<32;i=i+1)
            regist[i] <= 32'd0;
    else if (c_regWrite && WB_addRD != 5'd0)
        regist[WB_addRD] <= WB_dataRD;
end

always@(*) begin // immID
    case(ID_Instr[6:2])
    5'b00000,
    5'b00100,
    5'b11001:
        ID_imm = {{21{ID_Instr[31]}},ID_Instr[30:21],ID_Instr[20]}; // I
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
assign  ID_PCpi = ID_PC + ID_imm; // addID

// ID/EX registers
    reg     [10:0]  EX_ctrl;
    reg      [4:0]  EX_addR1, EX_addR2, EX_addRD;
    reg      [3:0]  EX_InstrALU;
    reg     [31:0]  EX_PCpX, EX_PCpi, EX_R1, EX_R2, EX_imm;
always@(posedge clk) begin
    if (!rst_n)
        {EX_ctrl, EX_addR1, EX_addR2, EX_addRD, EX_InstrALU, EX_PCpX, EX_PCpi, EX_R1, EX_R2, EX_imm} <= 90'b0;
    else if (wen_ID_EX)
        {EX_ctrl, EX_addR1, EX_addR2, EX_addRD, EX_InstrALU, EX_PCpX, EX_PCpi, EX_R1, EX_R2, EX_imm} <=
        {ID_ctrl, ID_addR1, ID_addR2, ID_addRD, ID_InstrALU, ID_PCpX, ID_PCpi, ID_R1, ID_R2, ID_imm};
end

// EX
    wire            c_ALUsrc;
    wire     [1:0]  c_ALUop;
    wire     [4:0]  shamt;
    reg     [31:0]  ALUin1, ALUin2;
    reg      [3:0]  ALUctrl;
    reg     [31:0]  EX_ALUout;
    reg     [31:0]  MEM_ALUout;
assign  {c_ALUsrc, c_ALUop} = EX_ctrl[10:8];
assign shamt = ALUin2[4:0];

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
        EX_ALUout = (ALUin1 < ALUin2) ? 32'd1 : 32'd0;
    4'b0011: // xor
        EX_ALUout = ALUin1 ^ ALUin2;
    4'b0100: // SLL
	    EX_ALUout = ALUin1 << shamt;
    4'b0101: //SRL
	    EX_ALUout = ALUin1 >> shamt;
    4'b1000: //SRA
	    EX_ALUout = $signed(ALUin1) >>> shamt;
    default:
        EX_ALUout = 32'd0;
    endcase
end

// EX/MEM registers
    reg      [7:0]  MEM_ctrl;
    reg      [4:0]  MEM_addRD;
    reg     [31:0]  MEM_ALUin2;
always@(posedge clk) begin
    if (!rst_n)
        {MEM_ctrl, MEM_addRD, MEM_ALUin2, MEM_ALUout} <= 77'b0;
    else if (wen_EX_MEM)
        {MEM_ctrl, MEM_addRD, MEM_ALUin2, MEM_ALUout} <= {EX_ctrl[7:0], EX_addRD, ALUin2, EX_ALUout};
end

// MEM
    wire            c_branch, c_jal, c_jalr, c_memRead, c_memWrite;
    wire            ALUzero;
    wire            beq, PCsrc;
    wire    [31:0]  MEM_D_data;
assign  {c_branch, c_jal, c_jalr, c_memRead, c_memWrite} = MEM_ctrl[7:3];

assign  ALUzero = (MEM_ALUout == 32'd0) ? 1'b1 : 1'b0;
and     andEX(beq, c_branch, ALUzero);
or      or_EX(PCsrc, beq, c_jal);
assign  PCnxt   = c_jalr ? MEM_ALUout : (PCsrc ? ID_PCpi : IF_PCpX); // PCpi PCpX propagate TODO

assign  DCACHE_ren   = c_memRead;
assign  DCACHE_wen   = c_memWrite;
assign  DCACHE_addr  = MEM_ALUout[31:2];
assign  DCACHE_wdata = {MEM_ALUin2[7:0],MEM_ALUin2[15:8],MEM_ALUin2[23:16],MEM_ALUin2[31:24]};
assign  MEM_D_data   = DCACHE_rdata;

// MEM/WB registers
    reg      [2:0]  WB_ctrl;
    reg     [31:0]  WB_ALUout, WB_D_data;
always@(posedge clk) begin
    if (!rst_n | flush_MEM_WB)
        {WB_ctrl, WB_addRD, WB_ALUout, WB_D_data} <= 72'b0;
    else if (wen_MEM_WB)
        {WB_ctrl, WB_addRD, WB_ALUout, WB_D_data} <= {MEM_ctrl[2:0], MEM_addRD, MEM_ALUout, MEM_D_data};
end

// WB
    wire     [1:0]  c_memToReg;
assign  {c_regWrite, c_memToReg} = WB_ctrl;
always@(*) begin // muxWB
    case(c_memToReg)
    2'd0: WB_dataRD = WB_ALUout;
    2'd1: WB_dataRD = {WB_D_data[7:0],WB_D_data[15:8],WB_D_data[23:16],WB_D_data[31:24]};
    2'd2, 2'd3: WB_dataRD = EX_PCpX; // jal jalr WB_PCpX propagate TODO
    endcase
end

// Data forwarding unit TODO
assign  MEM_regWrite = MEM_ctrl[2];
always@(*) begin
    if (!rst_n)
        {forward1, forward2} = 4'b0000;
    {forward1, forward2} = 4'b0000;
    // 1. EX Data hazard
    // MEM_regWrite MEM_addRD EX_addR1 EX_addR2
    // forward1 forward2
    // 2. MEM Data hazard
    // c_regWrite WB_addRD EX_addR1 EX_addR2
    // forward1 forward2
end

// Hazard detection / Pipeline stall unit TODO
assign  EX_memRead = EX_ctrl[4];
always@(*) begin
    {wen_PC, wen_IF_ID, wen_ID_EX, wen_EX_MEM} = 4'b1111;
    {flush_IF_ID, stallEX, flush_MEM_WB} = 3'b000;
    // 1. Load-Use Data Hazard
    // EX_memRead EX_addR1 EX_addR2 ID_addR1 ID_addR2
    // wen_PC wen_IF_ID stallEX
    // 2. Branch Hazard
    // ID_Instr
    // flush_IF_ID
    // 3. ICACHE stall
    if (ICACHE_stall) begin
        wen_PC = 1'b0;
        flush_IF_ID = 1'b1;
    end
    // 4. DCACHE stall
    // DCACHE_stall
    // wen_PC wen_IF_ID wen_ID_EX wen_EX_MEM flush_MEM_WB
end
endmodule

// Extension: Compressed Instructions
// TODO Instruction Alignment

module decomp(
    output   [31:0] o_str,
    output    [1:0] PCoff,
    input    [31:0] i_str,
    input           PCalign);

    reg   [1:0] PCoff;
    reg  [31:0] o_str;
    wire        [15:0] C = (PCalign) ? i_str[31:16] : i_str[15:0];

always@(*) begin
    if (C[1:0] == 2'b11) begin
        PCoff = 2'b10; // PC + 4
        o_str = i_str;
    end else begin
        PCoff = 2'b01; // PC + 2
        case({C[15:13],C[1:0]})
            5'b01000:
                o_str ={5'b00000,C[5],C[12:10],C[6],2'b0,
                        2'b01,C[9:7],
                        3'b010,
                        2'b01,C[4:2],
                        7'b0000011}; // C.lw
            5'b11000:
                o_str ={5'b00000,C[5],C[12],
                        2'b01,C[4:2],
                        2'b01,C[9:7],
                        3'b010,
                        C[11:10],C[6],2'b0,
                        7'b0100011}; // C.sw
            5'b00101:
                o_str ={C[12],C[8],C[10:9],C[6],C[7],C[2],C[11],C[5:3],{9{C[12]}},
                        5'b00001,
                        7'b1101111}; // C.jal
            5'b10001: begin
                case({C[6:5]})
                2'b00:
                    o_str ={7'b0100000,
                            2'b01,C[4:2],
                            2'b01,C[9:7],
                            3'b000,
                            2'b01,C[9:7],
                            7'b0110011}; // C.sub
                2'b10:
                    o_str ={7'b0000000,
                            2'b01,C[4:2],
                            2'b01,C[9:7],
                            3'b110,
                            2'b01,C[9:7],
                            7'b0110011}; // C.or
                2'b11:
                    o_str ={7'b0000000,
                            2'b01,C[4:2],
                            2'b01,C[9:7],
                            3'b111,
                            2'b01,C[9:7],
                            7'b0110011}; // C.and
                default:
                    o_str = 32'd0;
                endcase
            end
            5'b11001:
                o_str ={{4{C[12]}},C[6:5],C[2],
                        5'b00000,
                        2'b01,C[9:7],
                        3'b000,
                        C[11:10],C[4:3],C[12],
                        7'b1100011}; // C.beqz
            5'b10010: begin
                if (C[6:2] == 5'd0)
                    o_str ={12'b0,
                            C[11:7],
                            3'b000,
                            5'b00001,
                            7'b1100111}; // C.jalr
                else
                    o_str ={7'b0000000,
                            C[6:2],
                            C[11:7],
                            3'b000,
                            C[11:7],
                            7'b0110011}; // C.add
            end
            default:
                o_str = 32'd0;
        endcase
    end
end
endmodule



// L1 Cache by B06901031
// 2-way associative, write back
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
    
//==== wire/reg definition ================================
    reg            mem_read, mem_write;
    reg     [27:0] mem_addr;
    reg    [127:0] mem_wdata;

    reg      [2:0] state_w;
    reg      [2:0] state;
    reg      [3:0] valid1, valid2;
    reg      [3:0] dirty1, dirty2;
    reg      [3:0] lru;             // 0:data1 1:data2
    reg     [29:4] tag1    [0:3], tag2    [0:3];
    reg    [127:0] data1   [0:3], data2   [0:3];
    wire           hit1, hit2;
    wire     [3:2] set;
//==== combinational circuit ==============================
    localparam S_IDLE = 3'd0;
    localparam S_WBRD = 3'd1;
    localparam S_RD   = 3'd2;
    localparam S_WB   = 3'd3;
    localparam S_RDWB = 3'd4;

assign set = proc_addr[3:2];
assign proc_stall = ~(hit1 | hit2);
assign proc_rdata = proc_read & hit1
                  ? data1[set][proc_addr[1:0]*32+:32]
                  : proc_read & hit2
                  ? data2[set][proc_addr[1:0]*32+:32]
                  : 32'd0;
assign hit1 = valid1[set] & (tag1[set] == proc_addr[29:4]);
assign hit2 = valid2[set] & (tag2[set] == proc_addr[29:4]);

always@(*) begin
    case(state)
    S_IDLE: begin
        if (hit1 | hit2)
            state_w = S_IDLE;
        else if (proc_read)
            state_w = (~lru[set] & dirty1[set] | lru[set] & dirty2[set]) ? S_WBRD : S_RD;
        else if (proc_write)
            state_w = (~lru[set] & dirty1[set] | lru[set] & dirty2[set]) ? S_WB : S_RDWB;
        else
            state_w = S_IDLE;
    end
    S_WBRD:
        state_w = mem_ready ? S_RD : S_WBRD;
    S_RD:
        state_w = mem_ready ? S_IDLE : S_RD;
    S_WB:
        state_w = mem_ready ? S_RDWB : S_WB;
    S_RDWB:
        state_w = mem_ready ? S_IDLE : S_RDWB;
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
        case(state)
        S_IDLE: begin
            if (proc_read) begin
                if (proc_stall) begin
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
            end
            else if (proc_write) begin
                if (hit1) begin
                    dirty1[set] <= 1;
                    data1[set][proc_addr[1:0]*32+:32] <= proc_wdata;
                end else if (hit2) begin
                    dirty2[set] <= 1;
                    data2[set][proc_addr[1:0]*32+:32] <= proc_wdata;
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
        S_WBRD:
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
                    dirty1[set] <= 0;
                    tag1  [set] <= proc_addr[29:4];
                    data1 [set] <= mem_rdata;
                end else begin
                    valid2[set] <= 1;
                    dirty2[set] <= 0;
                    tag2  [set] <= proc_addr[29:4];
                    data2 [set] <= mem_rdata;
                end
            end
        S_WB:
            if (mem_ready) begin
                mem_read   <= 1;
                mem_write  <= 0;
            end
        S_RDWB:
            if (mem_ready) begin
                mem_read   <= 0;
                if (~lru[set]) begin
                    valid1[set] <= 1;
                    dirty1[set] <= 1;
                    tag1  [set] <= proc_addr[29:4];
                    data1 [set] <= mem_rdata;
                end else begin
                    valid2[set] <= 1;
                    dirty2[set] <= 1;
                    tag2  [set] <= proc_addr[29:4];
                    data2 [set] <= mem_rdata;
                end
            end
        endcase
    end
end
endmodule
