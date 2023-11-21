`include "func3_7.v"
`include "opcode.v"
`include "pc_register.v"
`include "IFstage.v"
`include "inst_mem.v"
`include "ID.v"
`include "registerbank.v"
`include "EX_stage.v"
`include "alu.v"
`include "mem_stage.v"
`include "main_memory.v"
`include "wb_stage.v"
module RV32I (
    input clk, reset, start,
    
    output[31:0]  inst_o_IF,  //instruction out from IF
    output[31:0]  inst_o_ID,
    output[31:0]  inst_o_EX,
    output[31:0]  inst_o_ME,
    output[31:0]  inst_o_WB
);
    
    wire[31:0] pc;
    wire branch;
    wire[31:0] branch_address;
/***************PC MODULE**********************/
    pc_register p(.pc(pc),.start(start),.branch_address(branch_address),.stall(1'b0),.clk(clk),.reset(reset),.branch(branch));
/************IF Module************************/
    wire [31:0] inst_from_ROM; 
    wire[31:0] pc_into_IF;
    wire[31:0] pc_to_ID;
    wire[31:0] addr_to_ROM;
    wire rd_en;

    IFstage iff(.reset(reset),.pc_into_IF(pc),.inst_from_ROM(inst_o_ROM),
    .branch(branch),.inst_to_decode(inst_o_IF),.pc_to_ID(pc_to_ID),.addr_to_ROM(addr_to_ROM),.rd_en(rd_en));
 /*   always @(*) begin
         pc_into_IF = pc;
    end */
/***********Instr Mem***********************/
    //wire rd_en;
    wire[31:0] inst_o_ROM;  // instruction fetched from ROM
    //wire [31:0] addr_in_ROM ;
//    always @(*) begin
//     addr_in_ROM = addr_to_ROM;
//    end
    inst_mem mem(.en(rd_en),.pc_addr(addr_to_ROM),.instruction(inst_o_ROM));    


/*********** register bank***********************/
    wire[4:0] rs1_addr, rs2_addr,rd_addr;
    wire rs1_re, rs2_re, rd_we;
    wire[31:0]rs1_data, rs2_data;
                                                                    // output from WB stage//
    registerbank rb(.rs1_addr(rs1_addr),.rs2_addr(rs2_addr),.rd_addr(rd_addr_to_register),
    .rs1_re(rs1_re),.rs2_re(rs2_re),.rd_we(rd_we),.rs1_data(rs1_data),.rs2_data(rs2_data),
    .rd_data_from_wb(rd_data_out_to_register));

/******************Decode Stage*******************/
    wire[31:0] op1, op2 ;
    wire[31:0] memory_offset;
    wire[3:0] alu_control ;
    wire[31:0] pc_to_EX;
ID_stage id(
    .instruction(inst_o_IF),
    .pc(pc_to_ID), 
    .reset(reset),
    .rs1_data(rs1_data),
    .rs2_data(rs2_data),
    .rs1_addr(rs1_addr),
    .rs2_addr(rs2_addr),
    .rd_addr(rd_addr),
    .op1(op1),.op2(op2),
    .rs1_read_enable(rs1_re),.rs2_read_enable(rs2_re),.rd_write_enable(rd_we),
    .memory_offset(memory_offset),
    .br_location(branch_address),
    .br(branch),

    .alu_control(alu_control),
    .pc_to_nextstage(pc_to_EX),
    .instr_to_nextstage(inst_o_ID)
    );
/********************EX Stage & ALU************************/
   
    wire[4:0]rd_addr_wb;
    wire rd_we_wb;
    wire[31:0] rd_data;
    wire[31:0]mem_addr_mem;
    wire[2:0] mem_flag;
    wire[31:0] data_to_memory_from_ex ;
EX_stage ex (
.reset(reset),
.op1(op1), .op2(op2),
.alu_control(alu_control),
.rd_addr(rd_addr),
.rd_we(rd_we),
.mem_offset(memory_offset),
.inst_from_Decode(inst_o_ID),
.rd_addr_wb(rd_addr_wb),
.rd_we_wb(rd_we_wb),
.rd_data(rd_data),
.mem_addr_mem(mem_addr_mem),
.mem_flag(mem_flag),
.inst_to_mem(inst_o_EX),
.data_to_memory_from_ex(data_to_memory_from_ex)  // sw data for outgoing
);

/**************Mem Stage*******************/
wire [31:0]  data_to_RAM;
wire [31:0] ram_location ;
wire[31:0] rd_data_out;
wire[4:0] rd_addr_out;
wire rd_we_out;
wire RAM_re, RAM_we;

mem_stage MM(
    .reset(reset),
    .rd_addr(rd_addr_wb),
    .rd_data(rd_data),
    .rd_we(rd_we),
    .mem_location(mem_addr_mem),
    .mem_flag(mem_flag),
    .data_to_memory_from_ex(data_to_memory_from_ex),  // sw data incoming
    .inst_from_ex(inst_o_EX),
    .data_from_RAM(data_from_RAM), //lw

    .rd_data_out(rd_data_out),
    .rd_addr_out(rd_addr_out),
    .rd_we_out(rd_we_out),
    .ram_location(ram_location),
    .data_to_RAM(data_to_RAM),
    .RAM_re(RAM_re), .RAM_we(RAM_we),
    .inst_out_mem(inst_o_ME)
);
wire[31:0] data_from_RAM ;
/**************ram*******************/
main_memory mm(        
    .address(ram_location),
    .data_into_RAM(data_to_RAM), // input to RAM
    .wr_en(RAM_we), 
    .rd_en(RAM_re),
    .clk(clk),
    .reset(reset),
    .data_from_RAM(data_from_RAM)
);

/**************wb stage************/
wire[4:0]rd_addr_to_register;
wire[31:0]  rd_data_out_to_register;
wb_stage wb(
    .reset(reset),
    .we(rd_we_out),
    .rd_data_from_mem(rd_data_out),
    .rd_addr_from_mem(rd_addr_out),
    .rd_data_out_to_register(rd_data_out_to_register),
    .rd_addr_to_register(rd_addr_to_register)

);







endmodule