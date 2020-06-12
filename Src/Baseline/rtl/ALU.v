/*
    Module:         ALU
    Description:
        An arithmetic logic unit that supports:
            1.  Addition
            2.  Subtraction
            3.  AND
            4.  OR
            5.  XOR
            6.  NOR
            7.  SLL
            8.  SRL
            9.  SRA
            10. SLT
*/

module ALU( ALUop, in1, in2, out );
    input   [3:0] ALUop;
    input  [31:0] in1;
    input  [31:0] in2;
    output [31:0] out;

    reg  [31:0] out;
    wire [31:0] add_sub;
    wire [31:0] in2_neg;
    wire  [4:0] shamt;

    assign in2_neg = 1 + ~in2;
    assign add_sub = in1 + ((ALUop==4'b0) ? in2 : in2_neg);
    assign shamt = in2[4:0];

    always @ (*) begin
        case (ALUop)
          4'd0: out = add_sub;                  // ADD
          4'd1: out = add_sub;                  // SUB
          4'd2: out = in1 & in2;                // AND
          4'd3: out = in1 | in2;                // OR
          4'd4: out = in1 ^ in2;                // XOR
          4'd5: out = ~(in1 | in2);             // NOR
          4'd6: out = in1 << shamt;             // SLL
          4'd7: out = in1 >> shamt;             // SRL
          4'd8: out = $signed(in1) >>> shamt;   // SRA 
          4'd9: out = {31'b0, add_sub[31]};     // SLT
          default: out = 32'b0;
        endcase
    end
endmodule