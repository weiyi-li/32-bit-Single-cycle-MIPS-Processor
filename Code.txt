// MIPS single Cycle processor originaly developed for simulation by Patterson and Hennesy
// Modified for synthesis using the QuartusII package by Dr. S. Ami-Nejad. Feb. 2009 

// Register File
module IMemory (
	address,
	q);

	input	[4:0]  address;
	output	[31:0]  q;

	wire [31:0] sub_wire0;
	wire [31:0] q = sub_wire0[31:0];

	lpm_rom	lpm_rom_component (
				.address (address),
				.q (sub_wire0),
				.inclock (1'b1),
				.memenab (1'b1),
				.outclock (1'b1));
	defparam
		lpm_rom_component.intended_device_family = "APEX20KE",
		lpm_rom_component.lpm_address_control = "UNREGISTERED",
		lpm_rom_component.lpm_file = "WL_Processor.mif",
		lpm_rom_component.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
		lpm_rom_component.lpm_outdata = "UNREGISTERED",
		lpm_rom_component.lpm_type = "LPM_ROM",
		lpm_rom_component.lpm_width = 32,
		lpm_rom_component.lpm_widthad = 5;


endmodule

module RegisterFile (Read1,Read2,Writereg,WriteData,RegWrite, Data1, Data2,clock,reset, X, Y, Z, T);

	input 	[4:0] Read1,Read2,Writereg; // the registers numbers to read or write
	input 	[31:0] WriteData; 			// data to write
	input 	RegWrite; 					// The write control
	input 	clock, reset; 				// The clock to trigger writes
	output 	[31:0] Data1, Data2, X, Y, Z, T; 		// the register values read;
	reg 	[31:0] RF[31:0]; 			// 32 registers each 32 bits long
	integer	k;
	
	// Read from registers independent of clock	
	assign 	Data1 = RF[Read1];
	assign 	Data2 = RF[Read2]; 
	assign  X = RF[6];
	assign  Y = RF[7];
	assign  Z = RF[1];
	assign  T = RF[2];
	// write the register with new value on the falling edge of the clock if RegWrite is high
	always @(posedge clock or posedge reset)
		if (reset) for(k=0;k<32;k=k+1) RF[k]<=32'h00000000;
		// Register 0 is a read only register with the content of 0
		else	if (RegWrite & (Writereg!=0)) RF[Writereg] <= WriteData;
endmodule

//ALU Control 
module ALUControl (ALUOp, FuncCode, ALUCtl);

	input 	[1:0] 	ALUOp;
	input 	[5:0] 	FuncCode;
	output	[3:0]	ALUCtl;
	reg		[3:0]	ALUCtl;
	
	always@( ALUOp, FuncCode)
	begin
	case(ALUOp)
	2'b00:	ALUCtl = 4'b0010;
	2'b01:	ALUCtl = 4'b0110;
	2'b11:  ALUCtl = 4'b0001;
	2'b10:	case(FuncCode)
			6'b 100000: ALUCtl = 4'b 0010;
			6'b 100010: ALUCtl = 4'b 0110;
			6'b 100100: ALUCtl = 4'b 0000;
			6'b 100101: ALUCtl = 4'b 0001;
			6'b 101010: ALUCtl = 4'b 0111;
			default:	ALUCtl = 4'b xxxx;
			endcase	
	default:	ALUCtl = 4'b xxxx;
	endcase
	end
endmodule

//ALU
module MIPSALU (ALUctl, A, B, ALUOut, Zero);
	input	[3:0] 	ALUctl;
	input	[31:0] 	A,B;
	output	[31:0] 	ALUOut;
	output 	Zero;
	reg		[31:0] ALUOut;
	
	assign Zero = (ALUOut==0); //Zero is true if ALUOut is 0
	always @(ALUctl, A, B) begin //reevaluate if these change
	case (ALUctl)
		0: ALUOut <= A & B;
		1: ALUOut <= A | B;
		2: ALUOut <= A + B;
		6: ALUOut <= A - B;
		7: ALUOut <= A < B ? 1:0;
		// .... Add more ALU operations here
		default: ALUOut <= A; 
		endcase
	end
endmodule

// Data Memory
module DataMemory(Address, DWriteData, MemRead, MemWrite, clock, reset, DReadData);
input 	[31:0] 	Address, DWriteData;
input			MemRead, MemWrite, clock, reset;
output 	[31:0]	DReadData;
reg		[31:0] 	DMem[7:0];	

assign  DReadData = DMem[Address[2:0]];
always @(posedge clock or posedge reset)begin
		if (reset) begin
			DMem[0]=32'h00000005;
			DMem[1]=32'h0000000A;
			DMem[2]=32'h00000055;
			DMem[3]=32'h000000AA;
			DMem[4]=32'h00005555;
			DMem[5]=32'h00008888;
			DMem[6]=32'h00550000;
			DMem[7]=32'h00004444;
			end else
			if (MemWrite) DMem[Address[2:0]] <= DWriteData;
		end
endmodule

// Main Controller
module Control (opcode,RegDst,Branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Jump);

input 	[5:0] 	opcode;
output	[1:0] 	ALUOp;
output	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jump;
reg		[1:0]	ALUOp;
reg 	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jump;

parameter R_Format = 6'b000000, LW = 6'b100011, SW = 6'b101011, BEQ=6'b000100, JMP=6'b000010, Ori=6'b001101;
always @(opcode)begin
	case(opcode)
		R_Format:{RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 1001000100;
		LW: 	 {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 0111100000; 
		SW: 	 {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b x1x0010000;	
		BEQ:	 {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b x0x0001010;
		JMP:     {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b xxxxxxxxx1;
	    Ori:     {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b 0101000110;
		// .... Add more instructions here
		default: {RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp,Jump}= 10'b xxxxxxxxxx;
		endcase
	end
endmodule 

// Datapath
module DataPath(RegDst, Branch, MemRead, MemtoReg, ALUOp, MemWrite,
ALUSrc, RegWrite, clock, reset, opcode,/* RF1,  RF2, RF3,*/ALUResultOut ,DReadData, X, Y, Z, T, PC, Jump);

input 	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,clock,reset,Jump;
input 	[1:0] 	ALUOp;
output 	[5:0] 	opcode;
output	[31:0]	/*RF1,  RF2, RF3,*/ ALUResultOut ,DReadData, X, Y, Z, T, PC;

reg 	[31:0] PC, IMemory[0:31];
wire 	[31:0] SignExtendOffset, PCOffset, PCOffsetadd, PCValue, PCadd, ALUResultOut,
		IAddress, DAddress, IMemOut, DmemOut, DWriteData, Instruction,
		RWriteData, DReadData, ALUAin, ALUBin, X, Y, Z, T;
wire 	[3:0] ALUctl;
wire 	Zero;
wire 	[4:0] WriteReg;

//Instruction fields, to improve code readability
wire [5:0] 	funct;
wire [4:0] 	rs, rt, rd, shamt;
wire [15:0] offset;

//Instantiate local ALU controller
ALUControl alucontroller(ALUOp,funct,ALUctl);

// Instantiate ALU
MIPSALU ALU(ALUctl, ALUAin, ALUBin, ALUResultOut, Zero);

// Instantiate Register File
RegisterFile REG(rs, rt, WriteReg, RWriteData, RegWrite, ALUAin, DWriteData,clock,reset, X, Y, Z, T);

// Instantiate Data Memory
DataMemory datamemory(ALUResultOut, DWriteData, MemRead, MemWrite, clock, reset, DReadData);

// Instantiate Instruction Memory
IMemory	IMemory_inst (
	.address ( PC[6:2] ),
	.q ( Instruction )
	);
	  
// Synthesize multiplexers
assign 	WriteReg	= (RegDst)			? rd 				: rt;
assign	ALUBin		= (ALUSrc) 			? SignExtendOffset 	: DWriteData;
assign	PCValue		= (Branch & Zero)	? PC+4+PCOffset 	: PC+4;
assign	RWriteData 	= (MemtoReg)		? DReadData			: ALUResultOut;	

// Acquire the fields of the R_Format Instruction for clarity	
assign {opcode, rs, rt, rd, shamt, funct} = Instruction;
// Acquire the immediate field of the I_Format instructions
assign offset = Instruction[15:0];
//sign-extend lower 16 bits
assign SignExtendOffset = { {16{offset[15]}} , offset[15:0]};
// Multiply by 4 the PC offset
assign PCOffset = SignExtendOffset << 2;
// Write the address of the next instruction into the program counter
assign 	PCadd   	= (Jump)			? PCOffsetadd 		: PC+4;//Additional PC value
assign  PCOffsetadd = {PC[31:28],offset[15:0],2'b 00};

always@(posedge clock)
begin
	if (reset)
	PC<=32'h00000000;
	else if(Jump==1)
	PC<=PCadd;
	else
	PC<=PCValue;
	end
endmodule
	

module MIPS1CYCLE(clock, reset,opcode, ALUResultOut ,DReadData, X, Y, Z, T, PC);
	input 	clock, 	reset;
	output	[5:0] 	opcode;
	output	[31:0]	ALUResultOut ,DReadData, X, Y, Z, T, PC; // For simulation purposes
	
	wire [1:0] ALUOp;
	wire [5:0] opcode;
	wire [31:0] SignExtend,ALUResultOut ,DReadData, X, Y, Z, T, PC;
	wire RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;

	// Instantiate the Datapath
	DataPath MIPSDP (RegDst,Branch,MemRead,MemtoReg,ALUOp,
	MemWrite,ALUSrc,RegWrite,clock, reset, opcode, ALUResultOut ,DReadData, X, Y, Z, T, PC, Jump);

	//Instantiate the combinational control unit
	Control MIPSControl (opcode,RegDst,Branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Jump);
endmodule
