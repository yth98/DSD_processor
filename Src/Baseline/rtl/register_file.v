/*
    Module:         Register File
    Description:
        32 x 32b register file for RISCV
*/

module Register_File ( Clk, rst_n, Read_in1, Read_in2, Write_reg, Write_data, WEN, Read_out1, Read_out2 );


    input [4:0] Read_in1, Read_in2, Write_reg;
    input [31:0] Write_data;
    input WEN, Clk, rst_n;

    output [31:0] Read_out1, Read_out2;

    reg [31:0] Read_out1, Read_out2;
    reg [31:0] register_read [0:31] ;
    reg [31:0] register_write [0:31] ;

    integer i, z;
    //===================Conbinential=======================//

   

    always@(*) begin
        register_read[0]  = 0;
        register_write[0] = 0;
        for( i=1; i<32; i=i+1 ) begin
            if( WEN == 1 && Write_reg == i )
                register_write[i] = Write_data;
            else
                register_write[i] = register_read[i];
        end
    end

    always@(*) begin
        Read_out1 = register_read[Read_in1];
        Read_out2 = register_read[Read_in2];
    end

    

    //===================Sequential======================//

    always@( posedge Clk ) begin
        for( z=1;z<32;z=z+1) begin
            if( rst_n == 0 )
                register_read[z] <= { 32{1'b0} };
            else
                register_read[z] <= register_write[z] ;
        end
    end

endmodule