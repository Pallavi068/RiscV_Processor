`define LW_flag 3'b001
`define LB_flag 3'b111

`define SW_flag 3'b010
`define SB_flag  3'b100

`define loadnoc 3'b011
module mem_stage (
    input reset,
    input [4:0]rd_addr,
    input [31:0]rd_data,
    input rd_we,
    input [31:0]mem_location,
    input [2:0]mem_flag,
   
    input [31:0]inst_from_ex,
    input [31:0]data_from_RAM, //for LW
    input [31:0] data_to_memory_from_ex, //for sw or loadnoc
    output reg [31:0] rd_data_out,
    output reg [4:0] rd_addr_out,
    output reg rd_we_out,
    output reg [31:0] ram_location, //to RAM
    output reg [31:0]data_to_store, // for SW or loadnoc
    /*****changes**************/
   
    input mmr_we_wb, // control signal to write in MMR
    output reg [31:0] mmr_location,  //address for loadnoc or storenoc to MMR**************
    output reg mmr_we_wb_out,
    output reg [31:0] loadnoc_data,  
   // output reg[2:0] mem_flag_out, 
    /**************************/
    output reg RAM_re, RAM_we,
    output [31:0] inst_out_mem
);


assign inst_out_mem = inst_from_ex;

always @(*) begin
    if (reset) begin
        ram_location=0;
        data_to_store=0;
        rd_data_out=0;
        rd_addr_out=0;
        rd_we_out=0;
        {RAM_we, RAM_re}=0;
        /******changes***********/
        mmr_we_wb_out = 0;
        mmr_location = 32'hx;
        loadnoc_data = 32'hx;
        /***********************/
    end
    else begin
        case (mem_flag)
            `LW_flag:begin
                RAM_re = (mem_location >=0 && mem_location<=32'h00003fff) ? 1:0;  //read RAM if address lies in this range//
                RAM_we = 0;
                data_to_store = 0;
                ram_location = mem_location; // calculated address from execute stage to memory stage and then to RAM
                rd_data_out = data_from_RAM; // loaded data from RAM to output(wb) stage
                rd_addr_out = rd_addr;
                rd_we_out = rd_we;
                /******changes***********/
                mmr_we_wb_out = 0;
                mmr_location = 32'hx;
                loadnoc_data = 32'hx;
                /***********************/
         
            end 
            /************Changes*********************/
            `LB_flag:begin
            RAM_re = (mem_location >=0 && mem_location<=32'h00003fff) ? 1:0;  //read RAM if address lies in this range//
            RAM_we = 0;
            data_to_store = 0;
            ram_location = mem_location; // calculated address from execute stage to memory stage and then to RAM
            rd_data_out = { {24{data_from_RAM[7]}} ,data_from_RAM[7:0]}; // loaded data from RAM to output(wb) stage
            rd_addr_out = rd_addr;
            rd_we_out = rd_we;
            /******changes***********/
            mmr_we_wb_out = 0;
            mmr_location = 32'hx;
            loadnoc_data = 32'hx;
            /***********************/
     
           end 
            /****************************************/


            `SW_flag : begin
                RAM_re = 0;
                RAM_we = (mem_location >=0 && mem_location<=32'h00003fff) ? 1:0;
                ram_location = mem_location;     // calculated address "<rs2+imm>" from execute stage to memory stage and then to RAM
                data_to_store = data_to_memory_from_ex ; //stored to RAM
               /* rd_addr_out= rd_addr; */
                /******changes***********/
                mmr_we_wb_out = 0;
                mmr_location = 32'hx;
                loadnoc_data = 32'hx;
                /***********************/
                rd_we_out = 0;
                rd_data_out = 0;
            end
            /**************CHANGES******************/
            `SB_flag : begin
            RAM_re = 0;
            RAM_we = (mem_location >=0 && mem_location<=32'h00003fff) ? 1:0;
            ram_location = mem_location;     // calculated address "<rs2+imm>" from execute stage to memory stage and then to RAM
            data_to_store[7:0] = data_to_memory_from_ex ; //stored to RAM
           /* rd_addr_out= rd_addr; */
            /******changes***********/
            mmr_we_wb_out = 0;
            mmr_location = 32'hx;
            loadnoc_data = 32'hx;
            /***********************/
            rd_we_out = 0;
            rd_data_out = 0;
            end
            /****************************************/



            `loadnoc : begin
                RAM_re =  0  ;
                RAM_we = 0 ;
                rd_we_out = 0;
                ram_location = mem_location;   
                // /******changes***********/  bypass the control signal
                if (ram_location>=32'h00004000 && ram_location<=32'h0000400f) begin
                    mmr_we_wb_out = mmr_we_wb;
                    mmr_location = mem_location;    // calculated address "<rs1+imm>" from execute stage to MMR
                    loadnoc_data = data_to_memory_from_ex;  // "rs2" to store in MMR<rs1+imm>
                    /***********************/
                end    
                /************Changes****************/
                    //STORENOC
                if (ram_location >= 32'h00004010 && ram_location<=32'h00004013) begin
                    mmr_we_wb_out = mmr_we_wb;
                    mmr_location = 32'h00004010;    // calculated address "<rs1+imm>" from execute stage to MMR
                    loadnoc_data = 32'd1;  // "1" to store in MMR<rs1+imm>
                end    
                   /*******************************/ 
                rd_data_out = 0;    
                end
             

                
            
            default: begin  // not memory instructions // so pass rd to next stage (WB)
                rd_we_out = rd_we;
                rd_addr_out = rd_addr;
                rd_data_out = rd_data;
               // mem_flag_out = 0;
                /******changes***********/
                RAM_re =  0  ;
                RAM_we = 0 ;
                mmr_we_wb_out = 0;
                mmr_location = 32'hx;
                loadnoc_data = 32'hx;
                /***********************/
            end
        endcase
    end
end

    
endmodule