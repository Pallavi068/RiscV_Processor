#loadnoc x2, x1, 8   <x2> --> MMR<x1+8>
#storenoc x2, x1, 8   1 --> MMR<x1+8> same opcode
addi x1, x0, 4
addi x2, x0, 4
#beq x2, x1, 16 # if t0 == t1 then target
slti x1, x0, 16
slli x3, x2, 5
andi x1, x0, 8 
xori x1, x0, 4  #jumps from 3
srai x2, x0, 2
#bne x2, x1, 16 # taken
sw x1,8(x0)
sub x10,x6,x4
sll x9,x4,x1
and x7,x6,x4
or x8,x6,x11
lw x3, 0(x1)
#beq x2, x1, 16
add x9,x9,x10
sra x5, x4,x1
sw x1,8(x0)
lw x3, 0(x1)
sub x6,x3,x0
and x7,x6,x4
addi x1, x0, 4
addi x2, x1, 6
lw x3, 0(x2)
add x4, x5, x6 # x4 =x51 x62
sub x10,x4,x5
#lui rd,FFFFF