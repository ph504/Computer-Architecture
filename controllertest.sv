
//This testbench must test the following for the Control unit:
		/*
		 *  branch -taken.
			      -not taken.
		
		 *  ORR
		 *  AND
		 *  SUB
		 *  ADD
		// imidiates and registers operands without shifts.
		 *  LDR 
		 *  STR
		// with positive imidiates offsets


// The input signals it gives the control unit are:
	clk,
	reset,
	Instr[31:12] which includes:
		-Cond
		-Op
		-Funct
		-Rd
	and ALUFlags[3:0].
*/
module controllertest();

	// input signals that would b e given to the controller.
	logic 			clk;
	logic 			reset;
	logic [31:12]	Instr;
	logic [3:0] 	ALUFlags;
	// ouput signals extracted from the controller.
	logic 			PCWrite;
    logic 			MemWrite;
    logic 			RegWrite;
    logic 			IRWrite;
    logic 			AdrSrc;
    logic [1:0]   	RegSrc;
    logic [1:0]   	ALUSrcA;
    logic [1:0]   	ALUSrcB;
    logic [1:0]   	ResultSrc;
    logic [1:0]   	ImmSrc;
    logic [1:0]   	ALUControl;

    // Connecting the controller module to the signals.
    controller ctrlr
    (clk, reset, Instr, ALUFlags, 
     PCWrite, MemWrite, RegWrite,
     IRWrite, AdrSrc, RegSrc, ALUSrcA,
     ALUSrcB, ResultSrc, ImmSrc, ALUControl);

endmodule // controller module