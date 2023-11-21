module stall(
    input reset,
    input is_stall,
    output reg [4:0] stall_code
    );

    always @(*) begin
        if(reset == 1) begin
            stall_code = 5'b00000;
        end
        if(is_stall == 1) begin
            stall_code = 5'b00111;
        end
        else 
            stall_code = 5'b00000;
        
    end

endmodule
