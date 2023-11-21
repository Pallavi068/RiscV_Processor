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
`include "pipe_IF_ID.v"
`include "pipe_ID_EX.v"
`include "pipe_EX_MEM.v"
`include "pipe_MEM_WB.v"
/*************Changes***************/
`include "MMR_reg.v"
`include "stall.v"
`include "branch_predictor.v"
`include "branch_predictor_2bit.v"
/**********************************/
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
    pc_register p(.pc(pc),.start(start),.branch_address(branch_address),.clk(clk),.reset(reset),.branch(branch));
/************IF Module************************/
    wire [31:0] inst_from_ROM; 
    wire[31:0] pc_into_IF;
    wire[31:0] pc_from_IF;
    wire[31:0] addr_to_ROM;
    wire rd_en;

    IFstage iff(.reset(reset),.pc_into_IF(pc),.inst_from_ROM(inst_o_ROM),
    .branch(branch),.inst_to_decode(inst_o_IF),.pc_to_ID(pc_from_IF),.addr_to_ROM(addr_to_ROM),.rd_en(rd_en));

/***********Instr Mem***********************/
    
    wire[31:0] inst_o_ROM;  // instruction fetched from ROM

    inst_mem mem(.en(rd_en),.pc_addr(addr_to_ROM),.instruction(inst_o_ROM));    


/*********** register bank***********************/
    wire[4:0] rs1_addr, rs2_addr,rd_addr;
    wire rs1_re, rs2_re, rd_we;
    wire[31:0]rs1_data, rs2_data;
                                                                    // output from WB stage//
    registerbank rb(.rs1_addr(rs1_addr),
    .rs2_addr(rs2_addr),
    .rd_addr(rd_addr_to_register),
    .rs1_re(rs1_re),
    .rs2_re(rs2_re),
    .rd_we(rd_we),
    .reset(reset),
    .clk(clk),
    .rs1_data(rs1_data),
    .rs2_data(rs2_data),
    .rd_data_from_wb(rd_data_out_to_register));

/*************** IF_ID Pipeline*******************/
wire [31:0] inst_to_ID;
wire [31:0] pc_to_ID;

pipe_IF_ID  IFID(.clk(clk),
.reset(reset),
.inst_from_IF(inst_o_IF),
.pc_from_IF(pc_from_IF),
.if_branch(branch), //from decode
.pc_to_ID(pc_to_ID),
.inst_to_ID(inst_to_ID)
// .stall_code(stall_code)
 );


/******************Decode Stage*******************/
    wire[31:0] op1, op2 ;
    wire[31:0] memory_offset;
    wire[3:0] alu_control ;
    wire[31:0] pc_to_EX;
/****************************/
    wire [31:0] U_type_imm;
    wire U_type_flag;
/****************************/   
/*********Changes*************/
    wire mmr_write_enable;
/*****************************/

ID_stage id(
    .instruction(inst_to_ID),
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
   
    .U_type_imm(U_type_imm),
    .U_type_flag(U_type_flag),
 
    /****changes**************/
    .mmr_write_enable(mmr_write_enable),
    /*************************/ 
    /******Changes to Hazards********/
    .prev_des_addr(rd_addr_wb), // output from EX stage
    .prev_result(rd_data),  
    .prev_rd_we(d_we_wb),
     .mem_flag(mem_flag),
    .data_from_mem(rd_data_out),         // output from MEM stage
    //.if_stall(is_stall),
    /*******************************/ 
    .br_location(branch_address),
    .br(branch),
    .alu_control(alu_control),
    .pc_to_nextstage(pc_to_EX),
    .instr_to_nextstage(inst_o_ID)
    );


    /*********************ID_EX_PIPELINE******************/
wire[31:0] inst_to_EX;
wire[31:0] op1_to_EX;
wire[31:0] op2_to_EX;
wire[31:0] memory_offset_to_EX;

wire [31:0] U_type_imm_to_EX;
wire U_type_flag_to_EX;

wire [3:0] alu_control_to_EX;
wire[4:0] rd_addr_to_EX;

wire rd_write_enable_to_EX;
/*********Changes****************/
wire mmr_write_enable_to_EX;
/********************************/

pipe_ID_EX IDEX(
    .clk(clk),
    .reset(reset),
    .inst_from_ID(inst_o_ID),
   //.pc_from_ID(pc_to_EX),
    .op1_from_ID(op1),
    .op2_from_ID(op2),
    .memory_offset_from_ID(memory_offset),
    .U_type_imm_from_ID(U_type_imm),
    .rd_addr_from_ID(rd_addr),
    .rd_write_enable_from_ID(rd_we),
    .alu_control_ID(alu_control),
    .U_type_flag_from_ID(U_type_flag),

    .inst_to_EX(inst_to_EX),
    //.pc_to_EX(),
    .op1_to_EX(op1_to_EX),
    .op2_to_EX(op2_to_EX),
    .memory_offset_to_EX(memory_offset_to_EX),
    .U_type_imm_to_EX(U_type_imm_to_EX),
    //.rd_data_to_EX(rd_data_to_EX),
    .rd_addr_to_EX(rd_addr_to_EX),
    .rd_write_enable_to_EX(rd_write_enable_to_EX),
    .alu_control_to_EX(alu_control_to_EX),
    .U_type_flag_to_EX(U_type_flag_to_EX),

    /***********changes******************/
    .mmr_write_enable_from_ID(mmr_write_enable),
    .mmr_write_enable_to_EX(mmr_write_enable_to_EX)
    /************************************/
    //.stall_code(stall_code)
);
/********************EX Stage & ALU************************/
   
    wire[4:0]rd_addr_wb;
    wire rd_we_wb;
    wire[31:0] rd_data;
    wire[31:0]mem_addr_mem;
    wire[2:0] mem_flag;
    wire[31:0] data_to_memory_from_ex ;
    /******Changes******************/
    wire mmr_we_wb;
    /******************************/
EX_stage ex (
.reset(reset),

.U_type_imm(U_type_imm_to_EX),
.U_type_flag(U_type_flag_to_EX),

.op1(op1_to_EX), .op2(op2_to_EX),
.alu_control(alu_control_to_EX),
.rd_addr(rd_addr_to_EX),
.rd_we(rd_write_enable_to_EX),
.mem_offset(memory_offset_to_EX),
.inst_from_Decode(inst_to_EX),
.rd_addr_wb(rd_addr_wb),
.rd_we_wb(rd_we_wb),
.rd_data(rd_data),
.mem_addr_mem(mem_addr_mem),
.mem_flag(mem_flag),
.inst_to_mem(inst_o_EX),
.data_to_memory_from_ex(data_to_memory_from_ex),  // "sw or loadnoc" data for outgoing
/*********Changes***********************/
.mmr_write_enable(mmr_write_enable_to_EX),
.mmr_we_wb(mmr_we_wb)
/************************************/

);
/************M*****EX_MEM PIELINE*************************/



wire[4:0] rd_addr_to_MEM;
wire[31:0] rd_data_to_MEM;
wire rd_we_to_MEM;
wire[31:0] mem_location_MEM;
wire[2:0] mem_flag_MEM;
wire [31:0] data_to_memory_MEM;
wire[31:0]  inst_to_MEM;
/*************Changes************/
wire mmr_we_to_MEM;
/*******************************/  

pipe_EX_MEM EXMEM(
    .clk(clk),
    .reset(reset),
    .inst_from_EX(inst_o_EX),
    .rd_addr_from_EX(rd_addr_wb),
    .rd_we_from_EX(rd_we_wb),
    .rd_data_from_EX(rd_data),
    .mem_location_EX(mem_addr_mem),
    .mem_flag_EX(mem_flag),
    .data_to_memory_EX(data_to_memory_from_ex),

    .rd_addr_to_MEM(rd_addr_to_MEM),
    .rd_data_to_MEM(rd_data_to_MEM),
    .rd_we_to_MEM(rd_we_to_MEM),
    .mem_location_MEM(mem_location_MEM),
    .mem_flag_MEM(mem_flag_MEM),
    .data_to_memory_MEM(data_to_memory_MEM),
    .inst_to_MEM(inst_to_MEM),
    /***********Changes*****************/
    .mmr_we_from_EX(mmr_we_wb),
    .mmr_we_to_MEM(mmr_we_to_MEM)
    /*********************************/

);

/**************Mem Stage*******************/
wire [31:0]  data_to_store;
wire [31:0] ram_location ;
wire[31:0] rd_data_out;
wire[4:0] rd_addr_out;
wire rd_we_out;
wire RAM_re, RAM_we;
/*************Changes*******************/
wire[31:0] mmr_location ;
wire mmr_we_wb_out;
wire[31:0] loadnoc_data;
/***************************************/

mem_stage MM(
    .reset(reset),
    .rd_addr(rd_addr_to_MEM),
    .rd_data(rd_data_to_MEM),
    .rd_we(rd_we_to_MEM),
    .mem_location(mem_location_MEM),
    .mem_flag(mem_flag_MEM),
    .data_to_memory_from_ex(data_to_memory_MEM),  // sw data incoming
    .inst_from_ex(inst_to_MEM),
    .data_from_RAM(data_from_RAM), //lw

    .rd_data_out(rd_data_out),
    .rd_addr_out(rd_addr_out),
    .rd_we_out(rd_we_out),
    .ram_location(ram_location),
    .data_to_store(data_to_store),
    .RAM_re(RAM_re), .RAM_we(RAM_we),
    .inst_out_mem(inst_o_ME),
    /**********Changes******************/
    .mmr_we_wb(mmr_we_to_MEM),
    .mmr_location(mmr_location),
    .mmr_we_wb_out(mmr_we_wb_out),
    .loadnoc_data(loadnoc_data)
 
    /***********************************/
);
wire[31:0] data_from_RAM ;
/**************ram*******************/
main_memory mm(        
    .ram_location(ram_location),
    .data_to_store(data_to_store), // input to RAM
    .wr_en(RAM_we), 
    .rd_en(RAM_re),
    .clk(clk),
    .reset(reset),
    .data_from_RAM(data_from_RAM)
);



/*************mem_wb**************/
wire [31:0] rd_data_to_WB;
wire[4:0] rd_addr_to_WB;
wire rd_we_to_WB;
wire [31:0] inst_out_to_WB;

/***********Changes******************/
wire[31:0] mmr_location_to_WB ;
wire mmr_we_to_WB;
wire[31:0] loadnoc_data_to_WB ;
/************************************/

pipe_MEM_WB MEMWB(
    .clk(clk),
    .reset(reset),
    .rd_data_from_MEM(rd_data_out),
    .rd_addr_from_MEM(rd_addr_out),
    .rd_we_out_from_MEM(rd_we_out),
    .inst_out_from_MEM(inst_o_ME),

    .rd_we_to_WB(rd_we_to_WB),
    .rd_data_to_WB(rd_data_to_WB),
    .rd_addr_to_WB(rd_addr_to_WB),
    .inst_out_to_WB(inst_out_to_WB),

    /********Changes*****************/
    .mmr_location_from_MEM(mmr_location),
    .mmr_location_to_WB(mmr_location_to_WB),
    .mmr_we_from_MEM(mmr_we_wb_out),
    .mmr_we_to_WB(mmr_we_to_WB),
    .loadnoc_data_from_MEM(loadnoc_data),
    .loadnoc_data_to_WB(loadnoc_data_to_WB)
    /*********************************/
);





/**************wb stage************/
wire[4:0]rd_addr_to_register;
wire[31:0]  rd_data_out_to_register;

/************Changes*********/
wire[31:0] loadnoc_data_out_to_MMR;
wire[31:0] mmr_location_out;
wire mmr_we_wb_out_to_MMR;
/**************************/
wb_stage wb(
    .reset(reset),
    .we(rd_we_to_WB),
    .rd_data_from_mem(rd_data_to_WB),
    .rd_addr_from_mem(rd_addr_to_WB),
    .rd_data_out_to_register(rd_data_out_to_register),
    .rd_addr_to_register(rd_addr_to_register),
    .inst_from_MEM(inst_out_to_WB),
    .inst_from_WB(inst_o_WB),

    /***********Changes************/
    .loadnoc_data(loadnoc_data_to_WB),
    .mmr_location(mmr_location_to_WB),
    .mmr_we_wb(mmr_we_to_WB),
    .mmr_we_wb_out(mmr_we_wb_out_to_MMR),
    .loadnoc_data_out_to_MMR(loadnoc_data_out_to_MMR),
    .mmr_location_out(mmr_location_out)
    /************************************/
);

MMR mmr(
    .loadnoc_data_from_wb(loadnoc_data_out_to_MMR),
    .mmr_we(mmr_we_wb_out_to_MMR),
    .mmr_location(mmr_location_out),
    .clk(clk)
);
//wire is_stall;
//wire[4:0] stall_code;
// stall s(
//     .reset(reset),
//     .is_stall(is_stall),
//     .stall_code(stall_code)
// );
/*************Changes*****************/

/*************************************/
wire predictedBranchOutcome;
/*OneBitBranchPredictor bp(
    .instruction(inst_o_ID),
    .actualBranchOutcome(branch),
    .predictedBranchOutcome(predictedBranchOutcome)
);
*/
wire[1:0] prediction_state ;

TwoBitSaturatingCounter bp (
   // .clk(clk),
    .instruction(inst_o_ID),
    .reset(reset),
    .actually_taken(branch),
    .prediction_state(prediction_state)
);


endmodule