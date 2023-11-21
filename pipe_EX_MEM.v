module pipe_EX_MEM (
    input clk, reset,
    input [31:0] inst_from_EX, 
    input[4:0] rd_addr_from_EX,
    input  rd_we_from_EX,
    input[31:0] rd_data_from_EX,
    input[31:0] mem_location_EX,  // calculated memory loaction from EX stage
    input[2:0] mem_flag_EX,
    input[31:0] data_to_memory_EX, // sw data coming from EX stage

    output reg[4:0] rd_addr_to_MEM,
    output reg[31:0] rd_data_to_MEM,
    output reg  rd_we_to_MEM,
    output reg[31:0] mem_location_MEM,  //calculated memory loaction in MEM stage

    output reg[2:0] mem_flag_MEM,
    output reg [31:0] data_to_memory_MEM,  // sw data
    output reg [31:0]inst_to_MEM,

    /*********changes******************/
    input mmr_we_from_EX,
    output reg mmr_we_to_MEM
    /**********************************/
);
    
    always @(posedge clk) begin
        if (reset) begin
            rd_we_to_MEM <=0;  // it stops writing anything in WB stage
            rd_data_to_MEM <=0;
            mem_location_MEM <=0;
            mem_flag_MEM <= 3'hx;  // matches to neither SW/LW
            data_to_memory_MEM <=0;
            inst_to_MEM <= 32'hx;
        end
        else begin
            rd_we_to_MEM <=rd_we_from_EX;  
            rd_data_to_MEM <= rd_data_from_EX ;
            rd_addr_to_MEM <= rd_addr_from_EX;
            mem_location_MEM <=mem_location_EX;
            mem_flag_MEM <= mem_flag_EX ;  
            data_to_memory_MEM <=data_to_memory_EX;  // sw data from EX is passed to MEM side
            inst_to_MEM <=  inst_from_EX;
            /***************Changes***********/
            mmr_we_to_MEM <= mmr_we_from_EX;
            /*********************************/
        end
    end


endmodule