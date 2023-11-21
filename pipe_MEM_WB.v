module pipe_MEM_WB (
    input clk, reset,
    input[31:0] rd_data_from_MEM,  // rd_data passing from MEM side
    input[4:0] rd_addr_from_MEM,
    input rd_we_out_from_MEM,
    input[31:0] inst_out_from_MEM,
    
    output reg rd_we_to_WB ,
    output reg[31:0] rd_data_to_WB, 
    output reg[4:0] rd_addr_to_WB ,
    output reg[31:0] inst_out_to_WB ,
    /***********Changes***************/
    input[31:0] mmr_location_from_MEM,
    output reg [31:0] mmr_location_to_WB,
    input mmr_we_from_MEM,
    output reg mmr_we_to_WB,
    input[31:0] loadnoc_data_from_MEM,
    output reg[31:0] loadnoc_data_to_WB
    /*********************************/
);
    always @(posedge clk) begin
        if (reset) begin
            rd_we_to_WB <= 0;
            rd_data_to_WB <= 0;
            rd_addr_to_WB <=0;
            inst_out_to_WB <= 32'hx;
        end
        else begin
            rd_we_to_WB <= rd_we_out_from_MEM;
            rd_data_to_WB <= rd_data_from_MEM;
            rd_addr_to_WB <= rd_addr_from_MEM;
            inst_out_to_WB <= inst_out_from_MEM;
            /***********Changes*****************/
            loadnoc_data_to_WB <= loadnoc_data_from_MEM;
            mmr_we_to_WB <= mmr_we_from_MEM;
            mmr_location_to_WB <= mmr_location_from_MEM;
            /***********************************/
        end
    end    


endmodule