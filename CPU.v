module processor ( 
		  input         CLK, RESET,
                  output [31:0] PCcurrent,//IMEM
                  input  [31:0] instruction,//IMEM
                  output        WEmem,//DMEM
                  output [31:0] Adres_mem,//DMEM
                  output [31:0] WriteData,//DMEM
                  input  [31:0] ReadData//DMEM
                );
	wire [3:0]  ALUControl; 
   	wire       Destination   ;
   	wire       AluIMMe    ;
    	wire       WEreg;
    	wire       PCSrcJal ;
    	wire       PCSrcJr   ;
	wire	   PCSrcBQ;
	wire MemToReg;

    controller controller_inst (
			instruction,
			ALUControl,
			Destination, 
			AluIMMe, 
			WEmem, 
			MemToReg, 
			WEreg,
			PCSrcJal, 
			PCSrcJr,
			PCSrcBQ
    );

	datapath compliting_instr (
			instruction,
			CLK,
			RESET,
			ReadData,
			ALUControl,
			Destination, 
			AluIMMe, 
			WEmem, 
			MemToReg, 
			WEreg,
			PCSrcJal, 
			PCSrcJr,
			PCSrcBQ,
			PCcurrent,
			Adres_mem,
			WriteData
    );

endmodule





module datapath (
	input [31:0]instruction,
	input CLK,
	input RESET,
	input [31:0]ReadData,
	input [3:0]ALUControl,
	input Destination, 
	input AluIMMe, 
	output WEmem, 
	input MemToReg, 
	input WEreg,
	input PCSrcJal, 
	input PCSrcJr,
	input PCSrcBQ,
	output [31:0]PCcurrent,
	output [31:0]Adres_mem,
	output [31:0]WriteData

);
wire [31:0] PCnext;
wire [31:0] PCjal;
wire [31:0] extimm;
wire [31:0] PCbeq;
wire [31:0] SrcA;
wire [31:0] SrcBtoAlu;
wire [31:0] PCSrcPC;
wire [31:0] ReadDataToReg;
wire [31:0] toWriteReg;
wire [4:0] CurrentRegD;
wire [4:0] defiRegD;
wire Zero;
wire PCSrcBeq;


PCAdder incCouter ( PCcurrent, PCnext);

jaladresmaker makeJaladres ( PCnext, instruction[25:0], PCjal);

signext extention ( instruction[15:0], extimm );

PCbranc makeBQadr ( extimm, PCnext, CLK, PCbeq );

mux2x1_5_dest choose_dest ( instruction[20:16], instruction[15:11], Destination, defiRegD );

mux2x1_JALdest Jal_dest (defiRegD, PCSrcJal, CurrentRegD);

mux2x1_32_ALUBSrc choose_ALUsrc( WriteData, extimm, AluIMMe, SrcBtoAlu); 

Registers redfile( instruction[25:21], instruction[20:16], CurrentRegD, ReadDataToReg, WEreg, CLK, SrcA, WriteData );

ALU32Bit Alu( ALUControl, SrcA, SrcBtoAlu, Adres_mem, Zero );

andbeq detectedBEQflag( Zero, PCSrcBeq);

mux_4to1_32toPC choose_new_PC( PCnext, PCjal, SrcA, PCbeq, PCSrcBeq, PCSrcBQ, PCSrcJr, PCSrcJal, PCSrcPC);

programcounter PC( PCSrcPC, RESET, CLK, PCcurrent );

mux2x1_32tomem choose_SRC_to_reg( Adres_mem, ReadData, MemToReg, toWriteReg);

mux_2x1_32jalwrite choose_normaly_or_JAL_reg( toWriteReg, PCnext, PCSrcJal, ReadDataToReg);

endmodule


module controller(

	
	input  [31:0] instruction,
	output  [3:0]  ALUControl, 
	output  Destination,
	output  AluIMMe,
	output  WEmem,
	output  MemToReg,
	output  WEreg,
	output  PCSrcJal,
	output  PCSrcJr,
	output  PCSrcBQ
);
decoder decod_inst (
			instruction,
			Destination, 
			AluIMMe, 
			WEmem, 
			MemToReg, 
			WEreg,
			PCSrcJal, 
			PCSrcJr,
			PCSrcBQ
    );

alu_dec alu_comand(
			instruction,
			ALUControl

   );	
	
endmodule




module alu_dec (
	input  [31:0] instruction     ,
	output [ 3:0] ALUControl
);

	reg [3:0] ALUControl;

	always @(instruction) begin
		casex ({instruction[31:26], instruction[10:0]})
			17'b00000000000100000  : ALUControl = 4'b0010;
			17'b00000000000100010  : ALUControl = 4'b0110;
			17'b00000000000100100  : ALUControl = 4'b0000;
			17'b00000000000100101  : ALUControl = 4'b0001;
			17'b00000000000101010  : ALUControl = 4'b0111;
			17'b001000xxxxxxxxxxx  : ALUControl = 4'b0010; 
			17'b100011xxxxxxxxxxx  : ALUControl = 4'b0010; 
			17'b101011xxxxxxxxxxx  : ALUControl = 4'b0010; 
			17'b000100xxxxxxxxxxx  : ALUControl = 4'b0110;
			17'b01111100000010000  : ALUControl = 4'b1000;
			17'b01111100100010000  : ALUControl = 4'b1001;
			17'b000000xxxxx000100  : ALUControl = 4'b1010;
			17'b00000000000000110  : ALUControl = 4'b1011;
			17'b00000000000000111  : ALUControl = 4'b1100;
			default          : ALUControl = 4'b0010;
		endcase
	end

endmodule





module decoder(

	
	input  [31:0] instruction, 
	output  Destination,
	output  AluIMMe,
	output  WEmem,
	output  MemToReg,
	output  WEreg,
	output  PCSrcJal,
	output  PCSrcJr,
	output  PCSrcBQ
);

	wire [5:0] opcode;
	assign opcode = instruction[31:26];

	wire [5:0] func;
	assign func = instruction[5:0];
	
	wire [4:0] sramt;
	assign sramt = instruction[10:6];

	wire is_add = ((opcode == 6'h00) & (func == 6'h20));
	wire is_sub = ((opcode == 6'h00) & (func == 6'h22));
	wire is_and = ((opcode == 6'h00) & (func == 6'h24));
	wire is_or  = ((opcode == 6'h00) & (func == 6'h25));
	wire is_slt = ((opcode == 6'h00) & (func == 6'h2A));
	wire is_sllv = ((opcode == 6'h00) & (func == 6'h04));
	wire is_srlv = ((opcode == 6'h00) & (func == 6'h06));
	wire is_srav = ((opcode == 6'h00) & (func == 6'h07));

	wire is_addu_s = (( opcode == 6'h1f ) & (sramt == 5'h04));			
	wire is_addu = (( opcode == 6'h1f ) & (sramt == 5'h00));


	wire is_lw = (opcode == 6'h23);
	wire is_sw = (opcode == 6'h2B);

	wire is_beq  = (opcode == 6'h04);
	wire is_addi = (opcode == 6'h08);
	wire is_jr    = ( opcode == 6'h07 );
	wire is_jal   = ( opcode == 6'h03 );

	assign PCSrcBQ    = is_beq;
	assign PCSrcJr    = is_jr;
	assign PCSrcJal   = is_jal;
	assign MemToReg = is_lw;
	assign WEmem  = is_sw ;
	assign Destination    = is_add | is_sub | is_and | is_or | is_slt | is_sllv | is_srlv | is_srav | is_addu_s | is_addu;
	assign WEreg  = is_add | is_sub | is_and | is_or | is_slt | is_sllv | is_srlv | is_srav | is_addu_s | is_addu | is_lw  | is_addi;
	assign AluIMMe    = is_addi | is_lw | is_sw;

endmodule	


	



module ALU32Bit(

	input   [3:0]   ALUControl,   
	input   [31:0]  SrcA, SrcB,	    

	
	
	output  reg [31:0]  ALUResult,	
	output  reg     Zero
	);	   
		
        
	
 	
	

    always @(ALUControl,SrcA,SrcB)	
    begin



	
		case (ALUControl)
			4'b0000: // AND
				ALUResult = SrcA & SrcB;				
			4'b0001: // OR
				ALUResult = SrcA | SrcB;
			4'b0010: // ADD
				begin					
					ALUResult = SrcA + SrcB;
					
				end
			4'b0011: // XOR
				ALUResult = SrcA ^ SrcB;
			4'b0110: // SUB
					begin
						ALUResult = SrcA - SrcB;
						
					end
			4'b0111:  // SLT
				begin 
					
						if (SrcA < SrcB)
						begin
							ALUResult = 1 ;
						end
						
						else
						begin
							ALUResult = 0 ;
							
						end
					
				end
			4'b1000: // addu.qb
				begin
					
					ALUResult[7:0] = SrcA[6:0] + SrcB[6:0];
					ALUResult[7] = ALUResult[7] ^  (SrcA[7] ^ SrcB[7]);
						
					ALUResult[15:8] = SrcA[14:8] + SrcB[14:8];
					ALUResult[15] = ALUResult[15] ^  (SrcA[15] ^ SrcB[15]);
						
					ALUResult[23:16] = SrcA[22:16] + SrcB[22:16];
					ALUResult[23] = ALUResult[23] ^  (SrcA[23] ^ SrcB[23]);
						
					ALUResult[31:24] = SrcA[30:24] + SrcB[30:24];
					ALUResult[31] = ALUResult[31] ^  (SrcA[31] ^ SrcB[31]);			
					 	
				end
			
			4'b1001:// addu_s.qb
				begin
					
					ALUResult[7:0] = SrcA[6:0] + SrcB[6:0];
					ALUResult[7] = ALUResult[7] ^  (SrcA[7] ^ SrcB[7]);
						
						if (SrcB[7:0] > ALUResult[7:0] || SrcA[7:0] > ALUResult[7:0])
						begin
						ALUResult[7:0]=8'hff;
						end

					ALUResult[15:8] = SrcA[14:8] + SrcB[14:8];
					ALUResult[15] = ALUResult[15] ^  (SrcA[15] ^ SrcB[15]);

						if (SrcB[15:8] > ALUResult[15:8] || SrcA[15:8] > ALUResult[15:8])
						begin
						ALUResult[15:8]=8'hff;
						end
					ALUResult[23:16] = SrcA[22:16] + SrcB[22:16];
					ALUResult[23] = ALUResult[23] ^  (SrcA[23] ^ SrcB[23]);
						
						if (SrcB[23:16] > ALUResult[23:16] || SrcA[23:16] > ALUResult[23:16])
						begin
						ALUResult[23:16]=8'hff;
						end

					ALUResult[31:24] = SrcA[30:24] + SrcB[30:24];
					ALUResult[31] = ALUResult[31] ^  (SrcA[31] ^ SrcB[31]);		
					 	
						if (SrcB[31:24] > ALUResult[31:24] || SrcA[31:24] > ALUResult[31:24])
						begin
						ALUResult[31:24]=8'hff;
						end
				end
			4'b1010: //SLLV
				begin
					ALUResult = SrcB << (SrcA[4:0]);
				end
			4'b1011: //SRLV
				begin
					ALUResult = SrcB >> (SrcA[4:0]);
				end
			4'b1100: //SRAV
				begin
					case (SrcA[4:0])
					5'b00000: ALUResult = SrcB;
					5'b00001: ALUResult =  { SrcB[31] , SrcB[31:1] };
					5'b00010: ALUResult =  { {2{SrcB[31]}} , SrcB[31:2] };
					5'b00011: ALUResult =  { {3{SrcB[31]}} , SrcB[31:3] };
					5'b00100: ALUResult =  { {4{SrcB[31]}} , SrcB[31:4] };
					5'b00101: ALUResult =  { {5{SrcB[31]}} , SrcB[31:5] };
					5'b00110: ALUResult =  { {6{SrcB[31]}} , SrcB[31:6] };
					5'b00111: ALUResult =  { {7{SrcB[31]}} , SrcB[31:7] };
					5'b01000: ALUResult =  { {8{SrcB[31]}} , SrcB[31:8] };
					5'b01001: ALUResult =  { {9{SrcB[31]}} , SrcB[31:9] };
					5'b01010: ALUResult =  { {10{SrcB[31]}} , SrcB[31:10] };
					5'b01011: ALUResult =  { {11{SrcB[31]}} , SrcB[31:11] };
					5'b01100: ALUResult =  { {12{SrcB[31]}} , SrcB[31:12] };
					5'b01101: ALUResult =  { {13{SrcB[31]}} , SrcB[31:13] };
					5'b01110: ALUResult =  { {14{SrcB[31]}} , SrcB[31:14] };
					5'b01111: ALUResult =  { {15{SrcB[31]}} , SrcB[31:15] };
					5'b10000: ALUResult =  { {16{SrcB[31]}} , SrcB[31:16] };
					5'b10001: ALUResult =  { {17{SrcB[31]}} , SrcB[31:17] };
					5'b10010: ALUResult =  { {18{SrcB[31]}} , SrcB[31:18] };
					5'b10011: ALUResult =  { {19{SrcB[31]}} , SrcB[31:19] };
					5'b10100: ALUResult =  { {20{SrcB[31]}} , SrcB[31:20] };
					5'b10101: ALUResult =  { {21{SrcB[31]}} , SrcB[31:21] };
					5'b10110: ALUResult =  { {22{SrcB[31]}} , SrcB[31:22] };
					5'b10111: ALUResult =  { {23{SrcB[31]}} , SrcB[31:23] };
					5'b11000: ALUResult =  { {24{SrcB[31]}} , SrcB[31:24] };
					5'b11001: ALUResult =  { {25{SrcB[31]}} , SrcB[31:25] };
					5'b11010: ALUResult =  { {26{SrcB[31]}} , SrcB[31:26] };
					5'b11011: ALUResult =  { {27{SrcB[31]}} , SrcB[31:27] };
					5'b11100: ALUResult =  { {28{SrcB[31]}} , SrcB[31:28] };
					5'b11101: ALUResult =  { {29{SrcB[31]}} , SrcB[31:29] };
					5'b11110: ALUResult =  { {30{SrcB[31]}} , SrcB[31:30] };
					5'b11111: ALUResult =  { {31{SrcB[31]}} , SrcB[31] };
					endcase
				end
			endcase
	end

        always @(ALUResult) 
		begin
		if (ALUResult == 0 )  
		begin
			Zero = 1;
		end else 
		begin
			Zero = 0;
		end
	
		end


endmodule


module andbeq(

    
    input   Zero,
    output reg PCSrcBeq
	); 
always@ ( Zero)
	begin
	PCSrcBeq = Zero & 1'b1;
	end
endmodule


module jaladresmaker (

	input [31:0] PCnext,
	input [25:0] JalAdres,
	output reg [31:0] PCjal
	);

always @(JalAdres,PCnext)
	begin	
        
	PCjal = {PCnext[31:28], JalAdres[25:0], 2'b00 };

	end
endmodule




module mux2x1_32tomem(

    
    input   [31:0] ALUResult,
    input   [31:0] RD,
    input          MemToReg,
    output reg [31:0] toWriteReg
	);
     
	 always@(ALUResult,RD,MemToReg)
	 begin
  		  if (MemToReg == 1)
	 		begin
	  		toWriteReg = RD;
			end
	 	  else 
			begin
     			toWriteReg = ALUResult;
	 	        end
	 end
	 

endmodule

module mux2x1_5_dest(

    
    input   [4:0] regT,
    input   [4:0] regD,
    input          Destination,
    output reg [4:0] defiRegD
	);
     
	 always@(regT,regD,Destination)
	 begin
  		  if (Destination == 1)
	 		begin
	  		defiRegD = regD;
			end
	 	  else 
			begin
     			defiRegD = regT;
	 	        end
	 end
	 

endmodule




module mux2x1_32_ALUBSrc(

    
    input   [31:0] SrcB,
    input   [31:0] extimm,
    input          AlluIMMe,
    output reg [31:0] SrcBtoAlu
	);
     
	 always@(SrcB,extimm,AlluIMMe)
	 begin
  		  if (AlluIMMe == 1)
	 		begin
	  		SrcBtoAlu = extimm;
			end
	 	  else 
			begin
     			SrcBtoAlu = SrcB;
	 	        end
	 end
	 

endmodule



module mux_4to1_32toPC(

    
    input   [31:0] PCnext, PCjal, PCjr, PCbeq,
    input    PCSrcBeq, PCSrcBQ, PCSrcJr, PCSrcJal,
    output reg [31:0] PCcurrent
	); 
reg [2:0] key;
	 always@(PCnext , PCjal, PCSrcJal, PCbeq, PCSrcBeq, PCjr, PCSrcJr)
	 begin

	key=0;
	if(PCSrcBeq == 1 && PCSrcBQ == 1)
			begin	
			key = 1;
			end	
	else if ( PCSrcJr == 1 )
			begin	
			key = 2;
			end        
	else if (PCSrcJal == 1)
			begin
			key = 3;
			end	
		case (key) 
			0:
				PCcurrent = PCnext;
			1:
				PCcurrent = PCbeq;
			2:
				PCcurrent = PCjr;
			3:
				PCcurrent = PCjal;

		endcase
		
	 end
	 

endmodule


module mux2x1_JALdest(

    

    input   [4:0] regD,
    input          PCSrcJal,
    output reg [4:0] CurrentRegD
	);
     
	 always@(regD,PCSrcJal)
	 begin
  		  if (PCSrcJal == 1)
	 		begin
	  		CurrentRegD = 5'b11111;
			end
	 	  else 
			begin
     			CurrentRegD = regD;
	 	        end
	 end
	 

endmodule


module mux_2x1_32jalwrite(

   
    

    input   [31:0] toWriteReg, PCnext,
    input          PCSrcJal,
    output reg [31:0] WD3
	);
     
	 always@(PCnext,PCSrcJal,toWriteReg)
	 begin
  		  if (PCSrcJal == 1)
	 		begin
	  		WD3 = PCnext;
			end
	 	  else 
			begin
     			WD3 = toWriteReg;
	 	        end
	 end
	 

endmodule


module programcounter(
	input       [31:0]  PCNext,
	input               RESET, CLK,

	output reg  [31:0]  PCcurrent

);
	initial begin
	
		PCcurrent = 32'h00000000;
	end

    always @(posedge CLK)
    begin
    	
	
    	PCcurrent = PCNext;
		

    
	if (RESET == 1)
	    	begin
    		PCcurrent = 32'h00000000;
		end
    	

    end

endmodule

module PCAdder(

    input   [31:0]  PCcurrent,

    output reg [31:0]  PCnext
);


    always @(PCcurrent)
    begin
    	PCnext = PCcurrent + 32'h00000004;
    end

endmodule


module PCbranc(

input [31:0] extimm, PCnext, 
input CLK,
output reg [31:0] PCbeq
		);
wire [31:0] temp;
assign temp = extimm<<2;

always@(PCnext,extimm,CLK)
	 begin
	PCbeq = temp + PCnext;
	end
endmodule





module Registers(

	
	input [4:0] A1, A2, A3,
	input [31:0] WD3,
	input WEreg,CLK,
	output reg [31:0] SrcA,SrcB );
	
	
	reg [31:0] Registersmem [0:31];
	
	initial begin
		Registersmem[0] = 32'h00000000;
		Registersmem[7] = 32'h00000000;
		Registersmem[8] = 32'h00000000;
		Registersmem[9] = 32'h00000000;
		Registersmem[10] = 32'h00000000;
		Registersmem[11] = 32'h00000000;
		Registersmem[12] = 32'h00000000;
		Registersmem[13] = 32'h00000000;
		Registersmem[14] = 32'h00000000;
		Registersmem[15] = 32'h00000000;
		Registersmem[16] = 32'h00000000;
		Registersmem[17] = 32'h00000000;
		Registersmem[18] = 32'h00000000;
		Registersmem[19] = 32'h00000000;
		Registersmem[20] = 32'h00000000;
		Registersmem[21] = 32'h00000000;
		Registersmem[22] = 32'h00000000;
		Registersmem[23] = 32'h00000000;
		Registersmem[24] = 32'h00000000;
		Registersmem[25] = 32'h00000000;
		Registersmem[29] = 32'd252;
		Registersmem[31] = 32'b0;
	end
	
	always  @(negedge CLK)
		begin
	SrcA = Registersmem[A1];
	SrcB = Registersmem[A2];
		end
		
		
	
	always @(posedge CLK)
	begin


		if (WEreg == 1) 
		begin
			Registersmem[A3] = WD3;
		end
	
	end
	
	

endmodule










module signext(

    
      input   [15:0] imm,
      output reg [31:0] extimm
		);


        always@(imm)
	 begin
			extimm = {{16{imm[15]}}, imm};
	end
endmodule





