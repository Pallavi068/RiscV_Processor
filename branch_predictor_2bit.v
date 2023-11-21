`include "opcode.v"
module TwoBitSaturatingCounter (
 // input  clk,
  input[31:0]  instruction,
  input  reset,
  input  actually_taken,  // Increment the prediction_state
  output reg [1:0] prediction_state
);
reg isBranch;
wire[6:0] branch_opcode = instruction[6:0];
parameter SNT = 2'b00;
parameter WNT = 2'b01;
parameter WT = 2'b10;
parameter ST = 2'b11;
  // Initial state
  initial begin
    prediction_state = SNT;  // Initial state is strongly not taken
  end

  always @(*) begin
    if(branch_opcode == `opcode_branch_sbtype)
        isBranch = 1; 
    else
        isBranch = 0;     
    end


  // prediction_state logic
  always @(posedge isBranch or posedge reset) begin
    if (reset) begin
      prediction_state <= SNT;  // Reset to initial state
    end else begin
      // Increment and decrement logic
      if (actually_taken) begin
        if (prediction_state != ST )  // If not in the strongly taken state
          prediction_state <= prediction_state + 1;    // Move forward IF ACTUAL OUTCOME is taken
        else if (prediction_state == ST)
          prediction_state <= prediction_state;   
        end 

      else if (~actually_taken) begin
        if (prediction_state != SNT)  // If not in the strongly not taken state
          prediction_state <= prediction_state - 1;     // Move forward IF ACTUAL OUTCOME is not taken
        else if (prediction_state == SNT)
          prediction_state <= prediction_state;  
      end
    end
  end
  always @(*) begin
    $monitor("prediction is %b", prediction_state);
  end

endmodule