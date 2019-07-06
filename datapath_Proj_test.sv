// ----------------------------------------------------------------------- multicycle datapath:
module datapath(input  logic        clk, reset,
                output logic [31:0] Adr, WriteData,
                input  logic [31:0] ReadData,
                output logic [31:0] Instr,
                output logic [3:0]  ALUFlags,
                input  logic        PCWrite, RegWrite,
                input  logic        IRWrite,
                input  logic        AdrSrc, 
                input  logic [1:0]  RegSrc, 
                input  logic [1:0]  ALUSrcA, ALUSrcB, ResultSrc,
                input  logic [1:0]  ImmSrc, ALUControl);

  logic [31:0] PCNext, PC;
  logic [31:0] ExtImm, SrcA, SrcB, Result;
  logic [31:0] Data, RD1, RD2, A, ALUResult, ALUOut;
  logic [3:0]  RA1, RA2;

  // Your datapath hardware goes below. Instantiate each of the 
  // submodules that you need. Remember that you can reuse hardware
  // from the book. Be sure to give your instantiated modules 
  // applicable names such as pcreg (PC register), adrmux 
  // (Address Mux), etc. so that your code is easier to understand.

  // ADD CODE HERE
  
  // PC register:
  flopenr #(32)pc_pcNext(clk, 1'b0, PCWrite, PCNext, PC);

  // AdrSrc multiplexer:
  assign Adr = AdrSrc? Result : PC;

  // Instr non-architectural register:
  flopenr #(32)Instr_RD(clk, 1'b0, IRWrite, ReadData, Instr);

  // Data non-architectural register:
  flopenr #(32)d_RD(clk, 1'b0, 1'b1, ReadData, Data);

  // RegSrc[1:0] multiplexers:
  assign RA1 = RegSrc[0]? 15 : Instr[19:16];
  assign RA2 = RegSrc[1]? Instr[15:12] : Instr[3:0];

  // Extend Immediate module:
  extend_immediate ext(Instr[23:0], ImmSrc[1:0], ExtImm);

  // register file:
  register_file rf(clk, RegWrite, RA1, RA2, 
  	Instr[15:12], Result, Result, RD1, RD2);

  // WriteData 's non-architectural register:
  flopenr #(32)WD_rd2(clk, 1'b0, 1'b1, RD2, WriteData);

  // A from the RD1 's non-architectural register:
  flopenr #(32)a_rd1(clk, 1'b0, 1'b1, RD1, A);

  // ALUSrcA multiplexer:
  assign SrcA = ALUSrcA ? PC : A;

  // ALUSrcB multiplexer:
  mux3 	  #(32)srcb(WriteData, ExtImm, 4);

  // ALU:
  ALU alu(SrcA, SrcB, ALUControl, ALUResult, ALUFlags);

  // ALU 's non-architectural register:
  flopenr #(32)alureg(clk, 1'b0, 1'b1, ALUResult, ALUOut);

  // Result's multiplexer:
  mux3    #(32)res(ALUOut, Data, ALUResult);


endmodule // datapath


// ----------------------------------------------------------------------- Extend module:
module extend_immediate (	input	logic [23:0] 	Instr, 
			   				input 	logic [1:0]		ImmSrc,
			   				output 	logic [31:0]	ExtImm  );
		always_comb
			case(ImmSrc)
				2'b00: ExtImm = {24'b0, Instr[7:0]};					// 8-bit unsigned immediate.

				2'b01: ExtImm = {20'b0, Instr[11:0]}; 					// 12-bit unsigend immediate.

				2'b10: ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00};	// 24-bit two's complement shifted branch.

				default: ExtImm = 32'bx; 								// undefined
																		// for the sake of error 
																		// or combinational logic
			endcase // ImmSrc

endmodule // extend_immediate module 

// ----------------------------------------------------------------------- ALU:
module ALU (input 	logic [31:0] A,
			input 	logic [31:0] B,
			input 	logic [1:0]  ALUControl,
			output 	logic [31:0] Result,
			output 	logic [3:0]  flags );
	
	logic cout;
	logic N, Z, C, V;

	always_comb
		case (ALUControl)
			2'b00 	: {cout, Result} = A + B;	// add.
			2'b01 	: {cout, Result} = A - B;	// subtract.
			2'b10 	: Result = A & B;			// and.
			2'b11 	: Result = A | B;			// or.
			default : Result = 32'bx;			// for the sake of combinational logic.
		endcase

	// ALU flags:
	// Z = 1 if the result is zero.
	// N = 1 if the result is negative.
	// C = 1 if the result has a carry out.
	// V = 1 if the result has overflowed.
	assign Z 	 = ~| Result;
	assign N 	 = Result[31];
	assign C 	 = cout & ~ALUControl[1];
	assign V 	 = ~(A[31] ^ B[31] ^ ALUControl[0]) & (A[31] ^ Result[31]) & ~ALUControl[1];
	assign flags = {N,Z,C,V};

endmodule // ALU module


// ----------------------------------------------------------------------- register file module:
// three ported register file
// read two ports combinationally
// write third port on the rising edge of clock
// register 15 reads PC+8 instead:
module register_file(input 	logic 		clk,
					 input 	logic 		we3,
					 input 	logic [3:0]	ra1, ra2, wa3,
					 input 	logic [31:0] wd3, r15,
					 output logic [31:0] rd1, rd2);
		logic [31:0] rf[14:0];
		always_ff @(posedge clk)
			if(we3) rf[wa3] <= wd3;

		assign rd1 = (ra1 == 4'b1111) ? r15 : rf[ra1];
		assign rd2 = (ra2 == 4'b1111) ? r15 : rf[ra2];

endmodule // register_file module.


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

// ----------------------------------------------------------------------- mux 3-bit to 1-bit:
//mux 3 module which takes 3 inputs to 1 output:
module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule//mux3 module

// you should consider the possible change in the datapath design because of the change in the Microprogrammed Control Unit.