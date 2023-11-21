//`include "IFstage.v"

module pc_register(
    output reg [31:0] pc,
    input start, branch,
    input[31:0] branch_address,
    input clk,
    input reset,
    input stall
   // input[4:0] stall_code 
    );
    //reg[31:0] pc_final_out;
    /*always@(*) begin
        pc_final_out = pc;
    end*/
    //reg reset;
   //   reg[31:0] instr;
   //   wire[31:0] inst_to_decode, pc_to_ID,addr_to_ROM,rd_en;
     

   //   IFstage IF(.reset(reset),.inst_from_ROM(instr),.pc_into_IF(pc), .branch(branch),
   //  ,.inst_to_decode(inst_to_decode),.pc_to_ID(pc_to_ID),.addr_to_ROM(addr_to_ROM));
   //  initial 
   //  begin
   //    instr = 32'hABCD;
   //   end
   //  initial begin
   //    pc = $random;  // for testing purpose
   //  end
    always@(posedge clk) begin
      if (reset) begin
         pc<=-4;
      end 
      else
         begin
          if(start) begin
             if(~branch) 
                pc <= pc + 4;
             else if(branch) 
                pc <= branch_address;   
            /* else if (stall_code[2] == 1)
                pc <= pc; */
             end
         end    
       end
    
endmodule