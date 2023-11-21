`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 09.11.2021 21:14:35
// Design Name:
// Module Name: EX_stage
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
//`include "riscv_define_all.v"
//`include "alu.v"
`define LW_flag  3'b001
`define LB_flag  3'b111

`define SW_flag  3'b010
`define SB_flag  3'b100

`define loadnoc  3'b011
module EX_stage(
    input reset,
   /**********************/ 
    input[31:0] U_type_imm,
    input U_type_flag,
/*************************/
    input [31:0] op1, // From ID input from Reg1
    input [31:0] op2, // From ID input from Reg1
  //  input [31:0] op3, // From ID input from Reg1
    input [3:0] alu_control, // Operating to do

    input [4:0] rd_addr, // Location to write the data gotten from ID  
    input rd_we, // write enable 1 or 0
    input [31:0] mem_offset, // RAM location to write
   
    input [31:0] inst_from_Decode,
   
    output reg [4:0] rd_addr_wb, // output = input for WB
    output reg rd_we_wb, // rd write enable signal towards WB
    /******************CHANGES***************/
    input mmr_write_enable,   
    output reg mmr_we_wb,  
    /*****************************************/
    output reg [31:0] rd_data, // Calculated Data for WB
    output reg [31:0] mem_addr_mem, // Calculated addr to write to or load from

    output reg[2:0] mem_flag,    // Used to check if LW or SW is done in the MEM stage
    output reg [31:0] data_to_memory_from_ex, // This is used to send to the mem stage, as we have to STORE r2 in memory for SW
    output reg [31:0] inst_to_mem
    );
/**************CHANGES****************************/
    always @(*) begin
        if (mem_flag == `SW_flag) begin
            data_to_memory_from_ex = op1; 
            // sending rs1 data for store "SW" operation
        end
        else if (mem_flag == `loadnoc) begin
            data_to_memory_from_ex = op2;
            // sending rs2 data for "loadnoc" operation
        end
        else
            data_to_memory_from_ex = 32'hx;   // store nothing
    end
 /************************************************/   
    
    
    reg [31:0] rs1_alu;
    reg [31:0] rs2_alu;
    reg [31:0] rs3_alu;
    reg [3:0] op_alu;
    wire[31:0] result;
    alu alu_ex(
        .reset(reset),
        .rs1(rs1_alu),
        .rs2(rs2_alu),
        .rs3(rs3_alu),
        .alu_operation(op_alu),
        //.rd(alu_register) // Connect ALU REGISTER To ThE rd_data or mem_addr_mem
        .rd(result)
    );
    always@(*) begin
        rd_data = result;
    end 
    always @(U_type_flag) begin
        if (U_type_flag==1) begin
            rd_data = U_type_imm;
            
        end
    end

    always @(*) begin
        if(reset == 1) begin
            rs1_alu = 0;
            rs2_alu = 0;
            rs3_alu = 0;
            op_alu = 0;      
            rd_addr_wb = 0;
            rd_data = 0; 
            /************changes**********/
            mmr_we_wb = 0;
            /***************************/
        end
        else begin // Perform the ALU operations
            case (alu_control)
            4'd1: begin            //AND
                rs1_alu = op1;
                rs2_alu = op2;
                op_alu =  4'd1;
                mem_flag =3'b0;
            end
            4'd2: begin           //OR
                rs1_alu = op1;
                rs2_alu = op2;
                op_alu = 4'd2;
                mem_flag =3'b0;
            end
                4'd3: begin             //ADD
                    rs1_alu = op1;
                    rs2_alu = op2;
                    op_alu = 4'd3;
                    mem_flag = 3'b0;
                end
                4'd4: begin             //SUB
                    rs1_alu = op1;
                    rs2_alu = op2;
                    op_alu = 4'd4;
                    mem_flag =3'b0;
                end

                4'd5: begin          //SLL
                    rs1_alu = op1;
                    rs2_alu = op2;
                    op_alu = 4'd5;
                    mem_flag = 3'b0;
                end
                4'd6: begin           //SRA
                    rs1_alu = op1;
                    rs2_alu = op2;
                    op_alu = 4'd6;
                    mem_flag =3'b0;
                end
                4'd8: begin                //slt
                    rs1_alu = op1;
                    rs2_alu = op2;
                    op_alu = 4'd8;
                    mem_flag =3'b0;
                end
                4'd7: begin          //   srl
                    rs1_alu = op1;
                    rs2_alu = op2;
                    op_alu = 4'd7;
                    mem_flag =3'b0;
                end
                4'd9: begin             ///xor
                    rs1_alu = op1;
                    rs2_alu = op2;
                    op_alu = 4'd9;
                    mem_flag =3'b0;
                end 
                4'd10: begin                    //sw
                    rs1_alu = op2;           
                    rs2_alu = mem_offset;       // rs2 + imm
                    op_alu = 4'd3;
                    mem_flag = `SW_flag;
                end
                4'd11: begin                 //lw
                    rs1_alu = op1;
                    rs2_alu = mem_offset;
                    op_alu = 4'd3;
                    mem_flag = `LW_flag;
                end
                /***********Changes************/
                4'd13: begin                 //lb
                    rs1_alu = op1;
                    rs2_alu = mem_offset;
                    op_alu = 4'd3;
                    mem_flag = `LB_flag;
                end

                4'd14: begin                 //sb
                    rs1_alu = op1;
                    rs2_alu = mem_offset;
                    op_alu = 4'd3;
                    mem_flag = `SB_flag;
                end

                /***************************/
             


                
                //***********changes below******************//
                default: begin
                    rs1_alu = op1;           
                    rs2_alu = mem_offset;       
                    op_alu = 4'd3;              // rs1 + imm
                    mem_flag = `loadnoc;       // rs2 --> MMR[rs1+imm]
                end
                /******************************/
            endcase
        end
    end

    always @(*) begin
        rd_addr_wb = rd_addr;
        rd_we_wb = rd_we;
        /*********changes*************/
        mmr_we_wb = mmr_write_enable;
        /***************************/
        mem_addr_mem = 0; // Initialize mem_addr
        
        case (mem_flag)
          `LW_flag: begin
            mem_addr_mem = rd_data;  // calculated address from ALU
          end
          `SW_flag: begin
            mem_addr_mem  = rd_data;  // calculated address from ALU
          end  
          `loadnoc : begin
            mem_addr_mem = rd_data;
          end
        endcase
    end
   
    always @(*) begin
        inst_to_mem = inst_from_Decode;
    end
endmodule