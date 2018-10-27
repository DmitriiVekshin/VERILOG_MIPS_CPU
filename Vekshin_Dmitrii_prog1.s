#define v0 $2
#define a0 $4
#define a1 $5


#define t0 $8
#define t1 $9

.globl max
.set noat
.ent max

max:
	lw v0, 0x000C($0) //maximum
	lw a0, 0x0008($0) //amount of array's items
	addi t0, $0, 0x0000

for:
	nop
	beq a0, $0, done // if a0 == 0, break the cycle
	lw a1, 0x000C(t0) // iinitialization current arrays item adres
	addi a0, $0, ffff

	slt t1, v0, a1 //  maximium < current array item  ?   max = cur. arr. item : continue finding  
	beq t1, $0, inc // 
	addi v0, a1, 0x0000

inc:
	nop
	addi t0, t0, 0x00000004 // &arr[i] = &arr[i+1]
	jr for // jump to for//
done:
	nop
	sw v0,0x0004($0)
	jr main // navrat z podprogramu
.end max
