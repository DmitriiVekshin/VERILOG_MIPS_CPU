#define t0 $8
#define t1 $9

#define s0 $16
#define s1 $17
#define s2 $18
#define s3 $19
#define s4 $20
.globl start
.set noat
.ent start

start:
addi s0, $0, 0x0000 
addi s3, $0, 0x0017 
lw s1, 0x00000008(s0) // number IEEE-754
addi s2, $0, 0x0 // floor(logX)
srlv s1, s1, s3
addi s3, $0, 0x0000007F // 127
addi t0, $0, 0x00000001 // 1

slt s4, s1, s3 // exp < 127 ? neg : pos
beq t0, s4, neg //
sub s2, s1, s3
jal done // jump done
neg:
nop
sub s2, s3, s1
sub s2, $0, s2
done:
nop
sw s2,0x00000004(s0)
nop
.end start
