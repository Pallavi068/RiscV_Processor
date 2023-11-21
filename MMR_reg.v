`define MMR_binary "memory_regfile.txt"
module MMR (
    input [31:0] loadnoc_data_from_wb,
    input  mmr_we,
    input [31:0] mmr_location,
    input clk   
);
    reg[31:0] MMR[0:4] ; //4 for loadnoc
                        // 1 for storenoc
    
    initial begin
        $readmemb(`MMR_binary,MMR);
    end
    always @(posedge clk) begin
        // write from 0x4000 to 0x400f // loadnoc
        if (mmr_we) begin
            MMR[mmr_location[7:0] >> 2] <= loadnoc_data_from_wb; 
        end
        else
            $display("");
    end
//    always @(*) begin
//        for (integer i = 0;i<5 ;i=i+1 ) begin
//            $display("MMR[%d]-->%b",i,MMR[i]);
//        end
       
//    end
    integer fp1,fp2;
initial begin
    fp2 = $fopen("../dump_files/mmr_dump_inital.txt","w");
    for (integer j = 0; j <5 ; j=j+1 ) begin
        $fdisplay(fp2,"MMR[%d] = %h",j,MMR[j]);
    end
    $fclose(fp2);
end
always @(*) begin
    for (integer i = 0;i<5 ;i=i+1 ) begin
        $display("MMR[%d] = %h",i,MMR[i]);
    end
    fp1 = $fopen("../dump_files/mmr_dump_final.txt","a");
    for (integer j = 0; j <5 ; j=j+1 ) begin
       
        $fdisplay(fp1,"MMR[%d] = %h",j,MMR[j]);
    end
    $fclose(fp1);
end

endmodule