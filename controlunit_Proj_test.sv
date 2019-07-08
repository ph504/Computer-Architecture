// ----------------------------------------------------------------------- controller module:
module controller(input  logic         clk,
                  input  logic         reset,
                  input  logic [31:12] Instr,
                  input  logic [3:0]   ALUFlags,
                  output logic         PCWrite,
                  output logic         MemWrite,
                  output logic         RegWrite,
                  output logic         IRWrite,
                  output logic         AdrSrc,
                  output logic [1:0]   RegSrc,
                  output logic [1:0]   ALUSrcA,
                  output logic [1:0]   ALUSrcB,
                  output logic [1:0]   ResultSrc,
                  output logic [1:0]   ImmSrc,
                  output logic [1:0]   ALUControl);
                  
  		logic [1:0] FlagW;
  		logic       PCS, NextPC, RegW, MemW;
  
  		decode dec(clk, reset, Instr[27:26], Instr[25:20], Instr[15:12],
             	FlagW, PCS, NextPC, RegW, MemW,
             	IRWrite, AdrSrc, ResultSrc, 
            	ALUSrcA, ALUSrcB, ImmSrc, RegSrc, ALUControl);
  		// Conditional Logic for Conditional execution:
  		condlogic cl(clk, reset, Instr[31:28], ALUFlags,
               FlagW, PCS, NextPC, RegW, MemW,
               PCWrite, RegWrite, MemWrite);
endmodule // controller module.

// ----------------------------------------------------------------------- decoder module:
/*
 * specifies the signals needed for the corresponding state:
 * such as 
 * FlagW  		// goes to Cond Logic.
 * PCS  	  	// goes to Cond Logic. // 1 when we modify the PC. 
 				   					   // (look in the PC Logic module for more details of implementation).
	
 * NextPC 		// goes to Cond Logic. // 1 when it is in fetch state and is getting the next Instruction.
 * RegW 		// goes to Cond Logic.
 * MemW 		// goes to Cond Logic.
 * IRWrite 		// Enables reading the instruction from the Instruction memory.
 * AdrSrc		// chooses whether to read Data or Instruction from the Memory.
 * ResultSrc	// chooses the signal to be the Result from the ALU (ALUOut, Data, ALUResult).
 * ALUSrcA		// chooses the first operand for the ALU.
 * ALUSrcB		// chooses the second operand for the ALU.
 * ImmSrc		// chooses the correct form for extending the immediate given in the machie code.
 * RegSrc	    							// the first bit chooses whether to read the PC or the Rn for the first address.
 											//the second bit chooses whether the Rm or Rd should be picked 
 											//based on the instruction being Memory access or Data processing.
 * ALUControl	// chooses for the ALU to perform either ADD, SUB, AND, OR 
 				// respectively for the following values: 00,  01,  10, 11 

 */
module decode(	  input  logic       clk, reset,
	              input  logic [1:0] Op,
	              input  logic [5:0] Funct,
	              input  logic [3:0] Rd,
	              output logic [1:0] FlagW,
	              output logic       PCS, NextPC, RegW, MemW,
	              output logic       IRWrite, AdrSrc,
	              output logic [1:0] ResultSrc, ALUSrcA, ALUSrcB, 
	              output logic [1:0] ImmSrc, RegSrc, ALUControl);

  // ADD CODE HERE
  // Implement a microprogrammed controller
  // using a control memory (ROM).
  logic Branch, ALUOp;

  // PC Logic :
  pclogic 		pl(Rd, Branch, RegW, PCS);

  // ALU Decoder :
  aludecoder 	alu_dec(Funct[4:0], ALUOp, ALUControl, FlagW);

  // Instruction Decoder :
  instrdecoder 	ins_dec(Funct[5:0], Op[1:0], ImmSrc[1:0], RegSrc[1:0]);

  //Main Decoder :

endmodule // decode module.

// ----------------------------------------------------------------------- instruction decoder module :
module instrdecoder(input  logic [5:0] Funct,
					input  logic [1:0] Op,
					output logic [1:0] ImmSrc,
					output logic [1:0] RegSrc);

		always_comb
			case({Op,Funct[5],Funct[0]})
				4'b01?1: RegSrc = 2'b?0;// Memory Load
				4'b01?0: RegSrc = 2'b10;// Memory Store 
				4'b001?: RegSrc = 2'b?0;// Data Processing immediate
				4'b000?: RegSrc = 2'b00;// Data Processing register
				4'b10??: RegSrc = 2'b?1;// Branch
			endcase // {Op,Funct[5],Funct[0]}

		assign ImmSrc[1:0] = Op[1:0];	// Immediate Source is exactly equivalent to Operation bits 
										// except the time Op = 2'b11.

endmodule // instrdecoder module.

// ----------------------------------------------------------------------- ALU decoder module :
// ALU Decoder which determines the ALUControl [1:0] and the FlagW [1:0] signals:
module aludecoder(	input  logic [4:0]	Funct,
				  	input  logic 		ALUOp,
				  	output logic [1:0]	ALUControl,
				  	output logic [1:0]	FlagW);
		
		always_comb
			if(ALUOp)begin
				case(Funct[4:1])
					4'b0100 : ALUControl = 2'b00;	// ADD
					4'b0010 : ALUControl = 2'b01;	// SUB
					4'b0000 : ALUControl = 2'b10;	// AND
					4'B1100 : ALUControl = 2'b11;	// OR
					default : ALUControl = 2'bxx; 	// for the sake of error and combinational logic.
				endcase // Funct[4:0]
				// setting the flags if the S bit is 1.
				FlagW[1] 	= Funct[0];
				FlagW[0] 	= (ALUControl == 2'b00 | ALUControl == 2'b01) & Funct[0];
			end else begin
				ALUControl 	= 2'b00;	// set the alu for adding for other instructions (non-Data Processing).
				FlagW 		= 2'b00;	// and don't update the flags for the alu.
			end

endmodule // aluDecoder module.

// ----------------------------------------------------------------------- PC Logic module :
// PC Logic module which maintains if the PC should be updated.
module pclogic(	  	input  logic [3:0]	Rd,
			   	  	input  logic 		Branch,
			   	  	input  logic 		RegW,
			   	  	output logic 		PCS);
		
		assign PCS = ((Rd==15)&RegW)|Branch;	// if branching or if the R15 is needed 
												// and we want to set the registers (register wirte is enabled).
endmodule // pclogic module.

// ADD CODE BELOW
// Add code for the condlogic and condcheck modules. Remember, you may
// reuse code from the book.
// ----------------------------------------------------------------------- Conditional Logic module :
module condlogic( 	input  logic       clk, reset,
                  	input  logic [3:0] Cond,
                  	input  logic [3:0] ALUFlags,
                  	input  logic [1:0] FlagW,
                  	input  logic       PCS, NextPC, RegW, MemW,
                  	output logic       PCWrite, RegWrite, MemWrite);

  		logic [1:0] FlagWrite;
  		logic [3:0] Flags;
  		logic       CondEx, CondEx2;


  		// Delay writing flags until ALUWB state
  		flopenr #(2)flagwritereg(clk, reset, 1'b1, FlagW&{2{CondEx}}, FlagWrite); //----> I don't know what it does.

  		// ADD CODE HERE

  		// CondEx's Registers :
  		flopenr #(1)   condexreg(clk, reset, 1'b1, CondEx, CondEx2);
  		
  		// ALUFlags' Registers :
  		flopenr #(2)  ALUFlagsreg32(clk, reset, FlagWrite[1], ALUFlags[3:2], Flags[3:2]);
  		flopenr #(2)  ALUFlagsreg10(clk, reset, FlagWrite[0], ALUFlags[1:0], Flags[1:0]);

  		// Enables of PCWrite, RegWrite, MemWrite :
  		assign PCWrite 	= (PCS & CondEx2)|NextPC;
  		assign RegWrite = RegW & CondEx2;
  		assign MemWrite = MemW & CondEx2;

  		condcheck 	  cc(Cond[3:0], Flags[3:0], CondEx);

endmodule // condlogic module.

// ----------------------------------------------------------------------- Condition Check module :
// Condition check module used in the Conditional Logic module:
module condcheck( 	input  logic [3:0] Cond,
                  	input  logic [3:0] Flags,
                  	output logic       CondEx);

  		// ADD CODE HERE
  		logic neg, zero, carry, overflow, ge;

  		assign {neg, zero, carry, overflow} = Flags;
  		assign ge = (neg==overflow);

  		always_comb
  			case(Cond)
  				4'b0000 : CondEx = zero;				//EQ
  				4'b0001 : CondEx = ~zero;				//NE
  				4'b0010 : CondEx = carry;				//CS
  				4'b0011 : CondEx = ~carry; 				//CC
  				4'b0100 : CondEx = neg;					//MI
  				4'b0101 : CondEx = ~neg;				//PL
  				4'b0110 : CondEx = overflow;			//VS
  				4'b0111 : CondEx = ~overflow; 			//VC
  				4'b1000 : CondEx = carry & (~zero); 	//HI
  				4'b1001 : CondEx = ~(carry & (~zero));	//LS
  				4'b1010 : CondEx = ge;					//GE
  				4'b1011 : CondEx = ~ge;					//LT
  				4'b1100 : CondEx = ~zero & ge;			//GT
  				4'b1101 : CondEx = ~(~zero & ge);		//LE
  				4'b1110 : CondEx = 1'b1;				//Always
  				default : CondEx = 1'bx;				//undefined
  			endcase

endmodule // condcheck module.

// ----------------------------------------------------------------------- register module:
// enable = 1 if it has no enable signals
// reset  = 0 if it has no reset  signals
module flopenr#(parameter WIDTH = 8)
			   (input  logic				clk, reset, en,
			   	input  logic [WIDTH-1:0]	d,
			   	output logic [WIDTH-1:0]	q);

		// a flip-flop with the corresponding needs.
		always_ff @(posedge clk, posedge reset)
			if(reset)	 q<=0;
			else if (en) q<=d;


endmodule // flopenr module 