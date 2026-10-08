`timescale 1ns / 1ps

// =============================================================================
// Module:      systolic_array_core
// Description: Array of Processing Elements with 2D connections for
//              weight stationary matrix multiplication.
// =============================================================================

module systolic_array_core #(
    parameter ARRAY_SIZE = 8,
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  wire                                    clk,
    input  wire                                    rst_n,
    input  wire                                    compute_enable,  // enables MAC in all PEs
    input  wire                                    weight_load_en,  // weight loading mode
    input  wire                                    acc_clear,       // clear accumulators for new tile
    
    // Weight inputs - one per column, enter from top row
    input  wire signed [ARRAY_SIZE*DATA_WIDTH-1:0]  weight_in_flat,
    
    // Activation inputs - one per row, enter from left column (PRE-SKEWED externally)
    input  wire signed [ARRAY_SIZE*DATA_WIDTH-1:0]  act_in_flat,
    
    // Partial sum outputs - from bottom row of each column
    output wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]   psum_out_flat,
    
    // Zero flags output - per row, from leftmost column
    output wire [ARRAY_SIZE-1:0]                    zero_flags
);

    // 2D Wires to interconnect PEs
    wire signed [DATA_WIDTH-1:0] weight_wire [0:ARRAY_SIZE][0:ARRAY_SIZE-1];
    wire signed [DATA_WIDTH-1:0] act_wire    [0:ARRAY_SIZE-1][0:ARRAY_SIZE];
    wire signed [ACC_WIDTH-1:0]  psum_wire   [0:ARRAY_SIZE][0:ARRAY_SIZE-1];

    // Connect top boundary (weights and initial psums)
    genvar col;
    generate
        for (col = 0; col < ARRAY_SIZE; col = col + 1) begin : top_bnd
            assign weight_wire[0][col] = weight_in_flat[(col*DATA_WIDTH) +: DATA_WIDTH];
            assign psum_wire[0][col]   = {ACC_WIDTH{1'b0}};
        end
    endgenerate

    // Connect left boundary (activations) and zero flags
    genvar row;
    generate
        for (row = 0; row < ARRAY_SIZE; row = row + 1) begin : left_bnd
            assign act_wire[row][0] = act_in_flat[(row*DATA_WIDTH) +: DATA_WIDTH];
            // Connect to zero flags (leftmost column)
            // Note: pe module exposes zero_detected. We'll wire it directly from the PE instance
        end
    endgenerate

    // PE Instantiation and Grid Connections
    genvar i, j;
    generate
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin : pe_row
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin : pe_col
                
                wire zero_det;
                
                pe #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .ACC_WIDTH(ACC_WIDTH)
                ) u_pe (
                    .clk(clk),
                    .rst_n(rst_n),
                    .pe_enable(compute_enable),
                    .weight_load(weight_load_en),
                    .acc_clear(acc_clear),
                    .weight_in(weight_wire[i][j]),
                    .act_in(act_wire[i][j]),
                    .psum_in(psum_wire[i][j]),
                    .weight_out(weight_wire[i+1][j] /* Only valid if i+1 < ARRAY_SIZE */),
                    .act_out(act_wire[i][j+1]),
                    .psum_out(psum_wire[i+1][j]),
                    .zero_detected(zero_det)
                );
                
                if (j == 0) begin
                    assign zero_flags[i] = zero_det;
                end
                
            end
        end
    endgenerate

    // Connect bottom boundary (outputs)
    generate
        for (col = 0; col < ARRAY_SIZE; col = col + 1) begin : bot_bnd
            assign psum_out_flat[(col*ACC_WIDTH) +: ACC_WIDTH] = psum_wire[ARRAY_SIZE][col];
        end
    endgenerate

endmodule
