`timescale 1ns / 1ps

// =============================================================================
// Module:      sram_bank
// Description: Simple dual-port SRAM parameterized for BRAM inference.
// =============================================================================

module sram_bank #(
    parameter DATA_WIDTH  = 32,
    parameter DEPTH       = 256,
    parameter SRAM_DEPTH  = DEPTH,  // alias for instantiation compatibility
    parameter ADDR_WIDTH  = 8      // clog2(DEPTH)
)(
    input  wire                    clk,
    input  wire                    we,        // write enable
    input  wire [ADDR_WIDTH-1:0]   waddr,     // write address
    input  wire [DATA_WIDTH-1:0]   wdata,     // write data
    input  wire [ADDR_WIDTH-1:0]   raddr,     // read address
    output reg  [DATA_WIDTH-1:0]   rdata      // read data (1-cycle latency)
);

    // Xilinx Block RAM inference attribute
    (* ram_style = "block" *)
    reg [DATA_WIDTH-1:0] mem [0:SRAM_DEPTH-1];

    // Initialization
    integer i;
    initial begin
        for (i = 0; i < SRAM_DEPTH; i = i + 1) begin
            mem[i] = {DATA_WIDTH{1'b0}};
        end
    end

    // Synchronous write and read
    always @(posedge clk) begin
        if (we) begin
            mem[waddr] <= wdata;
        end
        rdata <= mem[raddr];
    end

endmodule
