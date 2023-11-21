module pipe_ID_EX (
    input clk, reset,
    input[31:0] inst_from_ID, pc_from_ID,
    input [31:0] op1_from_ID, // rs1_data
    input [31:0] op2_from_ID, // rs2_data
    input[31:0] memory_offset_from_ID, 
    input[31:0] U_type_imm_from_ID,
    input U_type_flag_from_ID,

   // input[31:0] rd_data_from_ID, // rd_data to WB
    input [4:0] rd_addr_from_ID,  
    input rd_write_enable_from_ID,
    input [3:0] alu_control_ID,  // alu control signal towards alu
    
    // all outgoing signal towards EX stage

    output reg [31:0] inst_to_EX, pc_to_EX,
    output reg  [31:0] op1_to_EX, // rs1_data
    output reg  [31:0] op2_to_EX, // rs2_data
    output reg [31:0] memory_offset_to_EX, 
    output reg [31:0] U_type_imm_to_EX,
    output reg U_type_flag_to_EX,
   // output reg [31:0] rd_data_to_EX, // rd_data to WB
    output reg  [4:0] rd_addr_to_EX,  
    output reg  rd_write_enable_to_EX,
    output reg  [3:0] alu_control_to_EX,

    /***********CHANGES***************/
    input mmr_write_enable_from_ID,
    output reg  mmr_write_enable_to_EX
    /*********************************/
    /************Changes to Hazard*************/
   // input[4:0] stall_code
    /******************************************/
);   



    always @(posedge clk) begin
        if (reset) begin
            inst_to_EX <= 32'hx;
            op1_to_EX <= 0;
            op2_to_EX <= 0;
            memory_offset_to_EX <= 0;
            U_type_imm_to_EX <=0;
            rd_write_enable_to_EX <= 0;
            alu_control_to_EX <= 0;
        end
    /**********Changes********************/    

      /***************************************/  
        else begin
            inst_to_EX <=  inst_from_ID;
            pc_to_EX   <= pc_from_ID;
            op1_to_EX <= op1_from_ID;
            op2_to_EX <= op2_from_ID;
            memory_offset_to_EX <= memory_offset_from_ID;
            U_type_imm_to_EX <= U_type_imm_from_ID;
           // rd_data_to_EX <= rd_data_from_ID;
            rd_addr_to_EX <=  rd_addr_from_ID;
            rd_write_enable_to_EX <= rd_write_enable_from_ID;
            alu_control_to_EX <= alu_control_ID; 
            U_type_flag_to_EX <= U_type_flag_from_ID;
            /******Changes*********************/
             mmr_write_enable_to_EX <=mmr_write_enable_from_ID;
            /*********************************/
         
        end
    end
    
endmodule