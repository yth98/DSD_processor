module L2cache(//64-block 256words
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
    reg      [63:0] valid;
    reg      [63:0] dirty;
    reg     [29:8] tag     [0:63];
    reg    [127:0] data    [0:63];
    wire           hit;
//==== combinational circuit ==============================
    localparam S_IDLE = 3'd0;
    localparam S_WBRD = 3'd1;
    localparam S_RD   = 3'd2;
    localparam S_WB   = 3'd3;
    localparam S_RDWB = 3'd4;

assign proc_stall = (~hit)&&(proc_read|proc_write);
assign proc_rdata = proc_read & hit
                  ? data[proc_addr[7:2]][proc_addr[1:0]*32+:32]
                  : 32'd0;
assign hit = valid[proc_addr[7:2]] & (tag[proc_addr[7:2]] == proc_addr[29:8]);

always@(*) begin
    case (state)
    S_IDLE: begin
        if (hit)
            state_w = S_IDLE;
        else if (proc_read)
            state_w = dirty[proc_addr[7:2]] ? S_WBRD : S_RD;
        else if (proc_write)
            state_w = dirty[proc_addr[7:2]] ? S_WB : S_RDWB;
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
        valid   <= 64'b0;
        dirty   <= 64'b0;
    end
    else begin
        state   <= state_w;
        case (state)
        S_IDLE: begin
            if (proc_read) begin
                if (proc_stall) begin
                    if (dirty[proc_addr[7:2]]) begin
                        mem_write  <= 1;
                        mem_addr   <= {tag[proc_addr[7:2]],proc_addr[7:2]};
                        mem_wdata  <= data[proc_addr[7:2]];
                    end else begin
                        mem_read   <= 1;
                        mem_addr   <= proc_addr[29:2];
                    end
                end
            end
            else if (proc_write) begin
                if (hit) begin
                    dirty[proc_addr[7:2]] <= 1;
                    data[proc_addr[7:2]][proc_addr[1:0]*32+:32] <= proc_wdata;
                end else begin
                    if (dirty[proc_addr[7:2]]) begin
                        mem_write  <= 1;
                        mem_addr   <= {tag[proc_addr[7:2]],proc_addr[7:2]};
                        mem_wdata  <= data[proc_addr[7:2]];
                    end else begin
                        mem_read   <= 1;
                        mem_addr   <= proc_addr[29:2];
                    end
                end
            end
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
                valid[proc_addr[7:2]] <= 1;
                dirty[proc_addr[7:2]] <= 0;
                tag[proc_addr[7:2]] <= proc_addr[29:8];
                data[proc_addr[7:2]] <= mem_rdata;
            end
        S_WB:
            if (mem_ready) begin
                mem_read   <= 1;
                mem_write  <= 0;
            end
        S_RDWB:
            if (mem_ready) begin
                mem_read   <= 0;
                valid[proc_addr[7:2]] <= 1;
                dirty[proc_addr[7:2]] <= 1;
                tag[proc_addr[7:2]] <= proc_addr[29:8];
                data[proc_addr[7:2]] <= mem_rdata;
            end
        endcase
    end
end

endmodule