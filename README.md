# 32-bit-Single-cycle-MIPS-Processor
A 32-bit single-cycle processor with MIPS architecture, implemented by Verilog HDL using Quartus II

## Components
Register, main controller, PC, ALU, ALU controller, datapath, multiplexer and memory

## Functions
Implemented functions and instructions including load (LW), store (SW), add, substract (SUB), jump (J), BEQ and OR

## Demonstration
The value assigned to processor component is X, Y, Z, T=6, 7, 1, 2. The data bus size is 20 and the design is an Ori.  
### Test 1
**Load the data stored in the X and Y locations of the data memory into the X and Y registers.**  
The value of X and Y is 6 and 7. The machine code used to load data from memory location to registers follows I-format. The data stored in memory location #6 and #7 is 0x00550000 and 0x00004444. The opcode of load instruction for the I-format machine codes is 100011. RS remains 000000, destination register Rt is register #6 or #7 in this case, and offset address is the memory location #6 or #7.  
The machine code loading data in memory location X(#6) to register X(#6) is  
100011 00000 00110 0000000000000110  
MIPS code: lw $6, 6($0).  
The machine code loading data in memory location Y(#7) to register Y(#7) is  
100011 00000 00111 0000000000000111  
Opcode | RS | Rt | Immediate (offset address)  
MIPS code: lw $7, 7($0)  
The simulated waveform is demonstrated below.  
![Image]()

### Test 2
**Add the X and Y registers and store the result in the Z register.**  
The machine code used to add data from two registers and store in another register follows R-format.  
The opcode of add instruction for the R-format machine codes is 000000. RS is register #6, destination register Rt is register #7 in this case, shift amount remains 000000, and the function code for add instruction is 100000.  
The machine code adding data in register X(#6) and Y(#7) and storing the result to register Z(#1) is  
000000 00110 00111 00001 00000 100000  
Opcode | RS | Rt | Rd | Shift amount | Function code  
MIPS code: add $1, $6, $7  
The simulated waveform is demonstrated below.  
![Image]()  

### Test 3
**Store the data from the Z register into the Z memory location.**  
The machine code used to store data from register to memory location follows I-format.  
The opcode of store instruction for the I-format machine codes is 101011. RS remains 000000, destination register Rt is memory location #1 in this case, and offset address is the register #1.  
The machine code storing data to register Z(#1) from memory location Z(#1) is  
101011 00000 00001 0000000000000001  
Opcode | RS | Rt | Immediate (offset address)  
MIPS code: sw $1, 1($0)  
The simulated waveform is demonstrated below.  
![Image]()  

### Test 4
**Load the data in the Z memory location into the T register.**  
The machine code used to load data from memory location to registers follows I-format.  
The opcode of load instruction for the I-format machine codes is 100011. RS remains 000000, destination register Rt is register #2 in this case, and offset address is the memory location #1.  
The machine code loading data in memory location Z(#1) to register T(#2) is  
100011 00000 00010 0000000000000001  
Opcode | RS | Rt | Immediate (offset address)  
MIPS code: lw $2, 1($0)  
The simulated waveform is demonstrated below.  
![Image]()

### Test 5
**Simulate the operation of the BEQ instruction.**  
BEQ operation requires two identical data to draw a result and conduct consequent operations. Since the equality between the data in register Z(#1) and T(#2), where the data loaded is 0x00554444, the BEQ operation is valid. The machine code used to load data from memory location to registers follows I-format.  
Suppose that the result is stored in register #4, according to the logic below,  
PC+1+offset address = destination register  
PC is +5, which is indicated by the position of machine code in .mif file in this case. Destination register is assigned as register #4. Therefore, the offset address value is equal to -2. By 2’s complement, the signed binary expression of decimal 2 can be obtained, which is used as offset address.  
The opcode of BEQ instruction for the I-format machine codes is 000100. RS is register #1, destination register Rt is register #2, and offset address is signed number -2 in this case.  
Therefore, the machine code of BEQ instruction is  
000100 00001 00010 1111111111111110  
Opcode | RS | Rt | Immediate (offset address)  
MIPS code: beq $1, $4, $2  
The simulated waveform is demonstrated below.   
![Image]()

### Test 6
**Simulate the operation of the Jump instruction.**  
New variables should be added. The code representing main controller and datapath need to be improved. The code below representing new parameter JMP needs to be added to the part of main controller.  
```markdown
module Control (opcode,RegDst,Branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Jump);

input 	[5:0] 	opcode;
output	[1:0] 	ALUOp;
output	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jump;
reg		[1:0]	ALUOp;
reg 	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jump;

parameter R_Format = 6'b000000, LW = 6'b100011, SW = 6'b101011, BEQ=6'b000100, JMP=6'b000010;
```

Introduce the new variable Jump, the number of variables in the beginning of main controller becomes 10. Therefore, the main part of the main controller code has become
```markdown
R_Format:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 1001000100;
LW:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 0111100000; 
SW:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b x1x0010000;	
BEQ:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b x0x0001010;
default:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b xxxxxxxxxx;
```
In addition, the variable JMP needs to be assigned.
```markdown
JMP:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b xxxxxxxxx1;
```
In the part of datapath, the variable Jump and PC are required to be added at the beginning of the variable definition.
```markdown
module DataPath(RegDst, Branch, MemRead, MemtoReg, ALUOp, MemWrite,
ALUSrc, RegWrite, clock, reset, opcode, ALUResultOut ,DReadData, X, Y, Z, T, PC, Jump);
output [31:0] ALUResultOut ,DReadData, X, Y, Z, T, PC;
```

Meanwhile, additional assignments need to be added to the next instructions either.
Introduce two new variables PCadd and PCOffseradd, which represent the subsequent variable of PC and PCOffset in the next instruction.
```markdown
// Acquire the fields of the R_Format Instruction for clarity	
assign {opcode, rs, rt, rd, shamt, funct} = Instruction;
// Acquire the immediate field of the I_Format instructions
assign offset = Instruction[15:0];
//sign-extend lower 16 bits
assign SignExtendOffset = { {16{offset[15]}} , offset[15:0]};
// Multiply by 4 the PC offset
assign PCOffset = SignExtendOffset << 2;
// Write the address of the next instruction into the program counter
assign	PCadd = (Jump)	? PCOffsetadd : PC+4;//Additional PC value
assign  PCOffsetadd = {PC[31:28],offset[15:0],2'b 00};
```

Another part of code demonstrating the logic of Jump operation are also added to the code.
```markdown
always @ (posedge clock)
begin
if (reset)
	PC<=32'h00000000;
    else if (Jump==1)
	PC<=PCadd;
	else
	PC<=PCValue;
end
endmodule
```

The I-format machine code for Jump instruction consists of a Jump opcode and an address.   
The opcode of Jump instruction for the I-format machine codes is 000100. The address is signed as +5 in this case.   
Therefore, the machine code of Jump instruction is   
000010 00000000000000000000000101   
Opcode | Immediate (offset address)   
MIPS code: j 00101   
The simulated waveform is demonstrated below.   

### Test 7
**Make the processor correctly execute the Ori instruction.**  
To enable the additional Ori instruction the code in .v file requires modification.  
The code representing ALU control and main controller need to be improved. The code below needs to be added to the part of ALU control.  
```markdown
always@( ALUOp, FuncCode)
	begin
	case(ALUOp)
	2'b00:	ALUCtl = 4'b0010;
	2'b01:	ALUCtl = 4'b0110;
	2'b11:  ALUCtl = 4'b0001;
	2'b10:	case(FuncCode)
```
Another part of additional code should be added to main controller.
```markdown
parameter R_Format = 6'b000000, LW = 6'b100011, SW = 6'b101011, BEQ=6'b000100, JMP=6'b000010, Ori=b’001101;

R_Format:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 1001000100;
LW:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 0111100000; 
SW:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b x1x0010000;	
BEQ:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b x0x0001010;
default:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b xxxxxxxxxx;
Ori:{RegDst,ALUSrc,MemtoReg,RegWrite,Memread,MemWrite,Branch,ALUOp}=10’b 0101000110; 
```

### Test 8
**Simulate the operation of the Ori instruction.**  
To operate Ori instruction, the machine code in .mif file is modified.  
Ori operation requires two data from register and external locations to draw a result by operating OR calculation on each binary digit. The register #6 is chosen as the register which provides data. After performing OR operation on each binary digit of data from register #6 and an external data 0x0, the result is stored in the register #7. The machine code used to read data from registers and external locations, and conduct OR operation follows I-format.  
#### Case 1
The opcode of Ori instruction for the I-format machine codes is 001101. If we want to execute OR operation on data in register #6 and 0x0, and store the result in register #7, suppose that RS is register #6, destination register Rt is register #7, and offset address is the external data 0x0 in this case.  
Therefore, the machine code of BEQ instruction is  
001101 00110 00111 0000000000000000  
Opcode | RS | Rt | Immediate (offset address)  
MIPS code: ori $6, $7, 0000000000000000  
The simulated waveform for Case 1 is demonstrated below.  
![Image]()

As shown in the waveform, the data 0x00550000 in register #7 is the result of OR operation between 0x00550000 and external data 0x00000000. The additional Ori instruction is valid.

#### Case 2
The opcode of Ori instruction for the I-format machine codes is 001101. If we want to execute OR operation on data in register #6 and 0x1, and store the result in register #7, suppose that RS is register #6, destination register Rt is register #7, and offset address is the external data 0x1 in this case.  
Therefore, the machine code of BEQ instruction is  
001101 00110 00111 1111111111111111  
Opcode | RS | Rt | Immediate (offset address)  
MIPS code: ori $6, $7, 1111111111111111  
The simulated waveform for Case 2 is demonstrated below.  
![Image]()

As shown in the waveform, the data 0x11111111 in register #7 is the result of OR operation between 0x00550000 and external data 0x11111111. The additional Ori instruction is valid.

### Test 9
**Assign the processor’s data bus width from 32 to 20. Repeat the simulation of Test 4 using the new processor.**  
To modify the data bus width, the code in .v file is modified.  
In original code, the data bus size is defined as [31:0]. To meet the requirement of data bus size of 20, the definition [31:0] should be modified as [19:0].  
The detailed modified code is shown below.  
```markdown
module RegisterFile (Read1,Read2,Writereg,WriteData,RegWrite, Data1, Data2,clock,reset, X, Y, Z, T);

	input 	[4:0] Read1,Read2,Writereg; // the registers numbers to read or write
	input 	[19:0] WriteData; 			// data to write
	input 	RegWrite; 					// The write control
	input 	clock, reset; 				// The clock to trigger writes
	output 	[19:0] Data1, Data2, X, Y, Z, T; 		// the register values read;
	reg 	[31:0] RF[31:0]; 			// 32 registers each 32 bits long
	integer	k;

//ALU
module MIPSALU (ALUctl, A, B, ALUOut, Zero);
	input	[3:0] 	ALUctl;
	input	[19:0] 	A,B;
	output	[19:0] 	ALUOut;
	output 	Zero;
	reg		[19:0] ALUOut;

// Datapath
module DataPath(RegDst, Branch, MemRead, MemtoReg, ALUOp, MemWrite,
ALUSrc, RegWrite, clock, reset, opcode,/* RF1,  RF2, RF3,*/ALUResultOut ,DReadData, X, Y, Z, T, PC, Jump);

input 	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,clock,reset,Jump;
input 	[1:0] 	ALUOp;
output 	[5:0] 	opcode;
output	[19:0]	/*RF1,  RF2, RF3,*/ ALUResultOut ,DReadData, X, Y, Z, T, PC;

reg 	[31:0] PC, IMemory[0:31];
wire 	[31:0] SignExtendOffset, PCOffset, PCOffsetadd, PCValue, PCadd;
wire    [19:0] ALUResultOut,
		IAddress, DAddress, IMemOut, DmemOut, DWriteData, Instruction,
		RWriteData, DReadData, ALUAin, ALUBin, X, Y, Z, T;
wire 	[3:0] ALUctl;
wire 	Zero;
wire 	[4:0] WriteReg;

module MIPS1CYCLE(clock, reset,opcode, ALUResultOut ,DReadData, X, Y, Z, T, PC);
	input 	clock, 	reset;
	output	[5:0] 	opcode;
	output	[31:0]	ALUResultOut ,DReadData, X, Y, Z, T, PC; // For simulation purposes
	
	wire [1:0] ALUOp;
	wire [5:0] opcode;
	wire [19:0] ALUResultOut ,DReadData;
	wire [31:0] SignExtend, X, Y, Z, T, PC;
	wire RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;
```
By simulating with the machine code below, the waveform can be obtained.  
MIPS code: lw $6, 6($0)  
100011 00000 00111 0000000000000111  
Opcode | RS | Rt | Immediate (offset address)  
