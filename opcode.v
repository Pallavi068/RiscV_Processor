/*
`define opcode_and          7'b0110011
`define opcode_or           7'b0110011
`define opcode_add          7'b0110011
`define opcode_sub          7'b0110011
`define opcode_sll          7'b0110011
`define opcode_sra          7'b0110011 */

`define opcode_rtype        7'b0110011  // includes from ADD to AND

`define opcode_itype        7'b0010011  // includes all immediate type instruction
                                        // except load immediate

//`define opcode_addi         7'b0010011

`define opcode_sw           7'b0100011

// store memory type
//`define opcode_stype         7'b0100011



//`define opcode_beq          7'b1100011

`define opcode_branch_sbtype       7'b1100011
`define opcode_LUI                 7'b0110111
// load memory operations
`define opcode_lw           7'b0000011
`define opcde_lb            7'b0000011
// loadnoc
`define opcode_loadnoc      7'b1111111