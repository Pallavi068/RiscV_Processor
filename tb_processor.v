`timescale 1ns/1ps 
//`include "top_module.v"
`include "top_module_with_pipeline.v"
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.11.2021 11:04:11
// Design Name: 
// Module Name: 
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

module tb_processor();
    reg clk;
    reg start;
    reg reset;
        
    wire[31:0]  inst_o_IF; //instruction out from IF
    wire[31:0]  inst_o_ID;
    wire[31:0]  inst_o_EX;
    wire[31:0]  inst_o_ME;
    wire[31:0]  inst_o_WB;
    
    RV32I processor(
        .clk(clk),
        .reset(reset),
        .start(start),
        
        .inst_o_IF(inst_o_IF),
        .inst_o_ID(inst_o_ID),
        .inst_o_EX(inst_o_EX),
        .inst_o_ME(inst_o_ME),
        .inst_o_WB(inst_o_WB)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        start = 0;
        reset = 0;
    end
    
    initial begin 
        
        start = 1;
        #5 reset = 1;
        #5 reset = 0;
    end    
    initial begin
        $dumpvars(0,tb_processor);
        $dumpfile("output.vcd");
        
        #1000 $finish;   
     end
endmodule
