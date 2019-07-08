module datapath_testbench();

    logic        clk, reset;
        logic [31:0] Adr, WriteData;
        logic [31:0] ReadData;
        logic [31:0] Instr;
        logic [3:0]  ALUFlags;
        logic        PCWrite, RegWrite;
        logic        IRWrite;
        logic        AdrSrc;
        logic [1:0]  RegSrc; 
        logic [1:0]  ALUSrcA, ALUSrcB, ResultSrc;
        logic [1:0]  ImmSrc, ALUControl;
        datapath dp_DUT(  clk, reset,
                          Adr, WriteData,
                          ReadData,
                          Instr,
                          ALUFlags,
                          PCWrite, RegWrite,
                          IRWrite,
                          AdrSrc, 
                          RegSrc, 
                          ALUSrcA, ALUSrcB, ResultSrc,
                          ImmSrc, ALUControl);

endmodule // datapath_testbench