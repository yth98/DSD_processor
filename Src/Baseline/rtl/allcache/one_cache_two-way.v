module cache(//4-set 32-word finished
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
    assign proc_stall = (~(hit1 | hit2))&&(proc_read|proc_write);
assign proc_rdata = proc_read & hit1
                  ? data1[set][proc_addr[1:0]*32+:32]
                  : proc_read & hit2
                  ? data2[set][proc_addr[1:0]*32+:32]
                  : 32'd0;
assign hit1 = valid1[set] & (tag1[set] == proc_addr[29:4]);
assign hit2 = valid2[set] & (tag2[set] == proc_addr[29:4]);

always@(*) begin
    case (state)
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
        case (state)
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
