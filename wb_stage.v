module wb_stage (
input reset,
input we,

input[31:0] rd_data_from_mem,
input[4:0] rd_addr_from_mem,
input [31:0]inst_from_MEM,
output reg[31:0]  rd_data_out_to_register,
output reg [4:0] rd_addr_to_register,
output reg[31:0] inst_from_WB,
/**********Relevant to MMR************/
input[31:0] loadnoc_data,
input[31:0] mmr_location,
input mmr_we_wb,
output reg mmr_we_wb_out,
output reg[31:0] loadnoc_data_out_to_MMR,
output reg[31:0] mmr_location_out  
/**********************************/
);

always @(*) begin
    if (reset) begin
        rd_data_out_to_register = 0;
        rd_addr_to_register = 0;
        /***********changes**************/
        mmr_location_out = 0;
        loadnoc_data_out_to_MMR = 0;
        /********************************/
    end
    else begin
        rd_data_out_to_register = we ? rd_data_from_mem : 0;
        rd_addr_to_register = rd_addr_from_mem; 
        inst_from_WB = inst_from_MEM;
        /*****************changes**********/
        if (mmr_we_wb) begin
            mmr_location_out = mmr_location;  // send the address to MMR
            loadnoc_data_out_to_MMR = loadnoc_data;           
        end
        mmr_we_wb_out = mmr_we_wb;
        /**********************************/
    end
end



endmodule //wb_stage