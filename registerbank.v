`define REG_BINARY "Register_source.txt"
module registerbank (
    input[4:0] rs1_addr, rs2_addr,rd_addr,  
    input rs1_re, rs2_re, rd_we,
    input clk, reset,
    input[31:0] rd_data_from_wb, 
    output reg[31:0] rs1_data, rs2_data
);

// create the 32x32 register array

reg[31:0] regbank [0:31] ; 
//initializing
initial begin
    $readmemb(`REG_BINARY,regbank);
end

// always @(posedge clk, posedge reset) begin
//     if (reset) begin
//         for ( integer i=0 ; i < 32; i=i+1 ) begin
//             regbank[i] <= 0;
//         end
//     end
// end

//reading rs1 , rs2
always @(*) begin
    // if(reset)
    //     {rs1_data, rs2_data} = 32'd0;
    //else begin
        rs1_data = rs1_re ? regbank[rs1_addr] : 0;
        rs2_data = rs2_re ? regbank[rs2_addr] : 0;
    //end    
end
// writing to rd

always @(*) begin
    if (reset) begin
        regbank[rd_addr] = 32'd0;
    end
    else begin
        regbank[rd_addr] = rd_we ? rd_data_from_wb : 0 ;
    end
    
end
integer regmem1,regmem2;
initial begin
    regmem2 = $fopen("register_dump_inital.txt","w");
    for (integer j = 0; j <32 ; j=j+1 ) begin
        $fdisplay(regmem2,"reg[%d] = %h",j,regbank[j]);
    end
    $fclose(regmem2);
end
always @(posedge rd_we) begin
    for (integer i = 0;i<32 ;i=i+1 ) begin
        $display("reg[%d] = %h",i,regbank[i]);
    end
    regmem1 = $fopen("register_dump_final.txt","a");
    for (integer j = 0; j <32 ; j=j+1 ) begin
       
        $fdisplay(regmem1,"reg[%d] = %h",j,regbank[j]);
    end
    $fclose(regmem1);
end

endmodule //registerbank