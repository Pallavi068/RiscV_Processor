`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 09.11.2021 21:15:02
// Design Name:
// Module Name: alu
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


//`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 15.10.2021 17:31:08
// Design Name:
// Module Name: alu
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

//
//`include "riscv_define_all.v"
//
module alu(
    input reset,
    input [31:0]    rs1,
    input [31:0]    rs2,
    input [31:0]    rs3,
    input [3:0]     alu_operation,
    output reg[31:0]    rd  
    );
    //reg [31:0] rd;
    reg [4:0] rs2_5b;
   
    always @(*)
    begin
        if(reset == 1) begin
            rd = 0;
        end
        else begin
            case(alu_operation)
                4'd1:
                    begin
                        rd = rs1 & rs2;
                    end
                4'd2:
                    begin
                        rd = rs1 | rs2;
                    end
                4'd3:
                    begin
                        rd = rs1 + rs2;
                    end
                4'd4:
                    begin
                        rd = rs1 - rs2;
                    end
                4'd5:                        //SLL
                    begin
                        rs2_5b = rs2[4:0];
                        rd = rs1 << rs2_5b;
                    end
                4'd6:                          //SRA
                    begin
                        rs2_5b= rs2[4:0];
                        rd= (rs1[31]==1) ? ({5{rs1[31]}} | (rs1>>rs2_5b)): (rs1>> rs2_5b);
                    end
                4'd7:                        //SRL
                    begin
                        rs2_5b = rs2[4:0];
                        rd = rs1 >> rs2_5b;
                    end
                4'd8:                         //slt
                    begin
                       rd= (rs1 < rs2)? 1:0;
                    end    
                4'd9:                      //xor
                     begin
                        rd = rs1 ^ rs2;
                     end
                default:
                    begin
                        rd = 32'bx;
                    end
            endcase
        end
    end
   
   // assign rd = rd;
endmodule