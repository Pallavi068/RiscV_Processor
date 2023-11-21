`include "opcode.v"
`include "func3_7.v"
`define LW_flag  3'b001
`define SW_flag  3'b010
module ID_stage (
    input[31:0] instruction,  
    input[31:0] pc,
    input reset,      // all coming from IF STAGE
    /************Changes to Hazards****************/
    input[4:0] prev_des_addr,    // prev rd address
    input[31:0] prev_result,      // prev rd data
    input prev_rd_we,
    input[2:0] mem_flag,   // to check whether memory or execute
    input[31:0] data_from_mem,   // LW data from mem stage is bypassed here 
   
    /**********************************************/
    input[31:0] rs1_data, 
    input[31:0] rs2_data,
    input[31:0] rs3_data, // all coming from register bank
    output reg[4:0] rs1_addr, rs2_addr, rs3_addr,rd_addr,
    output reg[31:0] op1, op2,op3, // going to EX stage as operands for ALU

    // control signals
    output reg rs1_read_enable, rs2_read_enable, rs3_read_enable, rd_write_enable,
    /******CHANGES********************/
    output reg mmr_write_enable,
    /*******************************/
    output reg[31:0] memory_offset,
    output reg[31:0] U_type_imm ,
    //branch logic
    output reg[31:0] br_location,
    output reg br, // if branch comes br will be 1
    
    output reg U_type_flag, 
    // to decode which type of ALU operation has to be done
    output reg[3:0] alu_control,  
    output reg[31:0] pc_to_nextstage,
    output reg[31:0] instr_to_nextstage
   
);
    // r type
wire [6:0] opcode = instruction[6:0];
wire [2:0] funct3 = instruction[14:12];
wire [6:0] funct7 = instruction[31:25];

/*******************/
wire [4:0] shamt = instruction[24:20] ;
reg shamt_flag;
/******************/
wire [4:0] rs1 = instruction[19:15];
wire [4:0] rs2 = instruction[24:20];
wire [4:0] rd = instruction[11:7];


// I type
wire[11:0] I_imm = instruction[31:20];
// S type
wire [11:0] S_imm = {instruction[31:25], instruction[11:7]};
wire [19:0] U_imm = {instruction[31:12]};
// data for immediate type
reg[31:0] imm_data;
reg [31:0] SB_type_addr;

reg [31:0] cal_branch_addr ;

  

reg[9:0] funct7_3;


/***Changes***/
//reg[2:0] mem_status;
/***************/


always @(*) begin
    if (reset) begin
        {rs1_read_enable,rs2_read_enable,rs3_read_enable} = 0;
        {rs1_addr,rs2_addr,rs3_addr,rd_addr} = 5'b0;
        alu_control = 0;
        memory_offset = 0;
        br = 0;
        br_location = 32'b0;
        rd_write_enable = 0;
        /*********changes*************/
        mmr_write_enable=0;
        /***************************/
    end

  else begin
    funct7_3 = {funct7,funct3};
    case (opcode)
       `opcode_rtype : begin  
             br=0;  // all R Type has same opcodes///
             rs1_read_enable = 1;
             rs2_read_enable = 1;
            
             rs1_addr = rs1;
             rs2_addr = rs2;
                                        
             rd_addr = rd;
             rd_write_enable = 1;
             /********changes*********/
             mmr_write_enable=0;
          //   mem_flag = 0;
             /***********************/
             imm_data = 0;

             memory_offset = 0;
          case (funct7_3) 
        `funct7_3and  : begin 
                alu_control = 4'd1 ;  				
            end 
            `funct7_3or  : begin 
            alu_control = 4'd2 ;
            end
            `funct7_3add  : begin 
            alu_control = 4'd3 ;
            end  
            `funct7_3sub  : begin 
            alu_control = 4'd4 ;   
            end
            `funct7_3sll  : begin 
            alu_control = 4'd5 ;   
            end  
            `funct7_3sra  : begin 
            alu_control = 4'd6 ;    
            end 
            `funct7_3srl  : begin 
            alu_control = 4'd7 ;  
            end 
            `funct7_3slt  : begin 
            alu_control = 4'd8 ;  
            end 
            `funct7_3xor  : begin 
            alu_control = 4'd9 ;  
            end 
            default: begin $display("**at the default***"); end
          endcase
       end
        `opcode_itype : begin
          br=0;
          rs1_read_enable = 1;
          rs2_read_enable = 0;
         
          rs1_addr = rs1;
          rs2_addr = 0;
                                     
          rd_addr = rd;
          rd_write_enable = 1;
          /***Changes*****/
          mmr_write_enable=0;
        //  mem_flag = 0;
          /*************/  
          imm_data = {{20{I_imm[11]}},I_imm};   // extending to 32 bits as imm_data is 32 bits and I_imm is 12 bits. So extend MSB 20 times

           
          memory_offset = 0;
            case (funct3)                         /// include func3 file
            `funct3_addi  : begin
                alu_control = 4'd3;
                shamt_flag = 0;
            end
            `funct3_slti  : begin
                alu_control = 4'd8;
                shamt_flag = 0;
            end
            `funct3_xori  : begin
                alu_control = 4'd9;
                shamt_flag = 0;
            end
            `funct3_ori  : begin
                alu_control = 4'd2;
                shamt_flag = 0;
            end 
            `funct3_andi  : begin
                alu_control = 4'd1;
                shamt_flag = 0;
            end
                default: begin $display("**at the default***"); end
            endcase

            case(funct7_3)
                `funct7_3slli : begin
                    alu_control = 4'd5;
                     shamt_flag = 1;
                end
                `funct7_3srli : begin
                    alu_control = 4'd7;
                    shamt_flag = 1;
            end
                `funct7_3srai : begin
                alu_control = 4'd6;
                shamt_flag = 1;
            end
            default: begin shamt_flag = 0; end
            endcase

        end
        `opcode_sw : begin  // Implemented sw only  SW rs1, rs2, offset  // store into memory
             br=0;
             rs1_read_enable = 1;                           // rs1 ----> mem[rs2+ offset]
             rs2_read_enable = 1;     // access rs1 and rs2 both
       
             rs1_addr = rs1;
             rs2_addr = rs2;
                                   
             rd_addr = 0;
             rd_write_enable = 0;
             /*********changes*************/
             mmr_write_enable=0;
           //  mem_flag = `SW_flag;
             /***************************/   
             imm_data = 0;
             memory_offset = {{20{S_imm[11]}},S_imm};  
        /**********Changes*********************/     
             case (funct3)
                `funct3sw : begin
                    alu_control = 4'd10;
                end
                `funct3sb : begin
                    alu_control = 4'd14;        
                end
             endcase
        /**************************************/     
        end
        `opcode_loadnoc : begin  // Implemented   loadnoc rs2, rs1, offset  // store into memory mapped register
            br=0;
            rs1_read_enable = 1;      // rs2 ----> mem[rs1+ offset]
            rs2_read_enable = 1;     // access rs1 and rs2 both
  
            rs1_addr = rs1;
            rs2_addr = rs2;
                              
            rd_addr = 0;
            rd_write_enable = 0;
            /*********changes*************/
            mmr_write_enable = 1;
         //   mem_flag = 0;
            /***************************/
            imm_data = 0;
            memory_offset = {{20{S_imm[11]}},S_imm};  
            alu_control = 4'd12;      //default case in ex stage
                                      // to calculate rs1+offset
        end 
        
                                                                // load from memory
        `opcode_lw : begin 
            br=0;
             // Implemented sw only  LW rd <----mem[rs1+ offset]
             rs1_read_enable = 1;   
             rs2_read_enable = 0;
  
             rs1_addr = rs1;
             rs2_addr = 0;
                              
             rd_addr = rd;
             rd_write_enable = 1;
            /*********changes*************/
        mmr_write_enable=0;
       // mem_flag = `LW_flag;
        /***************************/
             imm_data = 0;
             memory_offset = {{20{I_imm[11]}},I_imm};  
             /********Changes*******************/
             case (funct3)
                `funct3lw : begin
                    alu_control = 4'd11;
                end
                `funct3lb : begin
                    alu_control = 4'd13;
                end
             endcase
             /**********************************/
        end
        
        
           `opcode_branch_sbtype : begin
             rs1_read_enable = 1;
             rs2_read_enable = 1;
       
             rs1_addr = rs1;
             rs2_addr = rs2;
                                   
             rd_addr = 0;
             rd_write_enable = 0;
                /*********changes*************/
        mmr_write_enable=0;
       // mem_flag = 0;
        /***************************/
             imm_data = 0;
             memory_offset = 0;
             SB_type_addr = {{20{instruction[31]}},instruction[7],instruction[30:25],instruction[11:8],1'b0};
             cal_branch_addr = pc + SB_type_addr;
             br_location = cal_branch_addr;  // branch address
               case (funct3)
               `funct3_beq : begin
                  br = (rs1_data == rs2_data) ? 1 : 0; 
               end
               `funct3_bne : begin
                  br = (rs1_data != rs2_data) ? 1 : 0;
               end
               `funct3_blt : begin
                  br = (rs1_data  < rs2_data)  ? 1 : 0;
               end 
                default: br = 0; 
               endcase
              
           end  
           `opcode_LUI : begin
                br = 0;
                rs1_read_enable = 0;
                rs2_read_enable = 0;
                rs1_addr = 0;
                rs2_addr = 0;
                
                rd_addr = rd;
                rd_write_enable = 1; 
                 /*********changes*************/
        mmr_write_enable=0;
      //  mem_flag = 0;
        /***************************/
                imm_data = 0 ;

                memory_offset = 0;

                U_type_imm = {U_imm,{12{1'b0}}};
                U_type_flag = 1;
                alu_control = 0;
           end
           default: begin
            $display("");
           end
        // end of opcode
    endcase
 end
end
  always @(*) begin
    if (reset || rs1_read_enable==0) 
        op1 = 0;
    /***************Changes to HAZARD********************/
    else if(mem_flag==0 && (rs1_read_enable==1 && (prev_des_addr==rs1)))  begin 
        op1 = prev_result;                          // forward the prev result as preset operand
    end
    else  if (mem_flag == `LW_flag) begin
        if (rs2_read_enable==1 && (prev_des_addr==rs2)) begin
            op2 = data_from_mem;
        end
        else if (rs1_read_enable==1 && (prev_des_addr==rs1)) begin
            op1 = data_from_mem;
        end        
    end
    /*****************************************************/   
             
    else if ( rs1_read_enable)
        op1 = rs1_data;
   
  end
  always @( *) begin
    if (reset) begin
        op2 = 0;
    end
     /***************Changes to HAZARD********************/
    else if(mem_flag==0 && (rs2_read_enable==1 && (prev_des_addr==rs2))) begin
        op2 = prev_result;                              // forward the prev result as preset operand
    end    
    
    /***************************************************/ 
    else if(rs2_read_enable) begin
        op2 = rs2_data;
    end    
    else if (shamt_flag == 1)   begin// slli, srli, srai
        op2 = {{27{1'b0}},shamt};
    end    
    else if (rs2_read_enable == 0) begin   // immediate type
        op2 = imm_data;
    end 
   
        
    else
        op2 = 0;   
  end

  always @(*) begin
    pc_to_nextstage = pc;
    instr_to_nextstage = instruction;
  end
endmodule


