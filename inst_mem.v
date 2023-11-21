//`include "pc_register.v"
`timescale 1ns/1ps
`define MAX_SIZE 1024   // 1024x32 memory
`define BINARY_FILE "Test_Binary.txt"
module inst_mem (
    
    input  en,
    input[31:0] pc_addr, 
    output reg[31:0] instruction 
);

reg[31:0] inst_mem[0:`MAX_SIZE-1] ;
//wire[31:0] pc;
initial begin
    $readmemb(`BINARY_FILE,inst_mem); ///load the memory array
end
always @(*) begin
   if ((pc_addr >> 2) > `MAX_SIZE-1) begin
    instruction <= 32'b00000000011000001000010001111111;  // GETBACK TO FIRST INSTRUCTION
   // $display("-------At the end of ROM------\n"); 
   end
    else if (en) begin
    instruction <= inst_mem[pc_addr>>2];
   
end
 end

 //just to check if memory is loaded or not
// always @(*) begin
//     for (integer i=0;i< 10 ;i=i+1 ) begin
//         $display(" %b \n",inst_mem[i]);
//     end
// end
// always @(*) begin
//     pc = pc_addr;
// end
// reg go,branch,reset,stall;
// reg[31:0] branch_address ;
// pc_register p(.pc(pc),.go(go),.branch(branch),.branch_address(branch_address)
// ,.clk(clk),.reset(reset),.stall(stall));


endmodule //inst_mem
