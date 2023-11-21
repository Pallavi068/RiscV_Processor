
`define MEM_BINARY "datamemory.txt"
//`define MEM_MAPPED_REG "memory_regfile.txt"
module main_memory #(parameter MAXSIZE=1024) (
    input[31:0] ram_location, // whose only 12 bits are of use
   // input[31:0] data_into_RAM,
    /**CHANGES***/
    input[31:0] data_to_store,
    /*************/
    input wr_en,
    input rd_en,
    input clk,reset,
    output reg[31:0] data_from_RAM
   
);
    reg[31:0] RAM[0 : MAXSIZE-1];

    initial begin                       // normal memory is loaded
        $readmemb(`MEM_BINARY, RAM);
    end
    

    always @(posedge clk) begin
     
         // write
            if (wr_en) begin
              /****changes*****/  
                RAM[(ram_location[11:0]) >> 2] <= data_to_store;
                 //   RAM[ram_location >> 2] <= data_to_store;             //Store op // normal memory operation                             
            end
            else
                    $display("not normal memory operation!!");    
            
        end

    always @(*) begin
       
            data_from_RAM = (rd_en==1 && wr_en==0) ?RAM[(ram_location[11:0])>>2] : 32'hx;  // normal memory operations    
            
    end
    integer mem1,mem2;
initial begin
    mem2 = $fopen("datamemory_dump_inital.txt","w");
    for (integer j = 0; j <32 ; j=j+1 ) begin
        $fdisplay(mem2,"mem[%d] = %h",j,RAM[j]);
    end
    $fclose(mem2);
end
always @(posedge wr_en) begin
    for (integer i = 0;i<32 ;i=i+1 ) begin
        $display("mem[%d] = %h",i,RAM[i]);
    end
    mem1 = $fopen("datamemory_dump_final.txt","a");
    for (integer j = 0; j <32 ; j=j+1 ) begin
       
        $fdisplay(mem1,"mem[%d] = %h",j,RAM[j]);
    end
    $fclose(mem1);
end
endmodule

