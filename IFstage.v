
module IFstage (
    input reset,
    input[31:0] inst_from_ROM, 
    input[31:0] pc_into_IF,
    input branch,
    output reg [31:0] inst_to_decode,
    output reg[31:0] pc_to_ID,
    output reg [31:0] addr_to_ROM,  
    output reg rd_en    //read enable
);





     always @(*) begin
        if(reset==1 ) begin
            pc_to_ID=0;
            inst_to_decode=0;
            rd_en=0;
        end
     


     else begin
        addr_to_ROM = pc_into_IF;
        pc_to_ID = pc_into_IF;
        inst_to_decode= inst_from_ROM;
        rd_en=1;

     end
    end
endmodule