module pipe_IF_ID (
    input  reset, clk,
    input[31:0] inst_from_IF,
    input[31:0] pc_from_IF,
                        //boolean type
    input if_branch,  // to check branch is coming from decode stage or not
    /*********Changed**********/
   // input[4:0] stall_code, 
    /**************************/
    output reg[31:0] pc_to_ID,
    output reg [31:0] inst_to_ID 
);

    always @(posedge clk ) begin
        if (reset  /*||(stall_code[2]==0 && stall_code[1]==1) */) begin
            pc_to_ID <= 0;
            inst_to_ID <= 32'hx;
        end
        else begin
            pc_to_ID <= pc_from_IF;
            inst_to_ID <= inst_from_IF;
        end
    end
    
endmodule