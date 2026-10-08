`timescale 1ns / 1ps

// =============================================================================
// Module:      pe
// Description: Processing Element for the Systolic Array.
//              Implements a weight-stationary MAC with optimizations.
//              Includes zero-skip to save power.
// =============================================================================

module pe #(
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          pe_enable,      // gates computation
    input  wire                          weight_load,    // 1=load weight from weight_in, 0=hold
    input  wire                          acc_clear,      // clear accumulator for new tile
    input  wire signed [DATA_WIDTH-1:0]  weight_in,      // weight from top PE or external
    input  wire signed [DATA_WIDTH-1:0]  act_in,         // activation from left PE or external
    input  wire signed [ACC_WIDTH-1:0]   psum_in,        // partial sum from PE above (0 for top row)
    output reg  signed [DATA_WIDTH-1:0]  weight_out,     // weight passed to PE below
    output reg  signed [DATA_WIDTH-1:0]  act_out,        // activation passed to PE right
    output reg  signed [ACC_WIDTH-1:0]   psum_out,       // partial sum to PE below
    output wire                          zero_detected   // 1 when act_in == 0
);

    // Internal registers
    reg signed [DATA_WIDTH-1:0] weight_reg;

    // Zero detection
    assign zero_detected = (act_in == 0);

    always @(posedge clk) begin
        if (!rst_n) begin
            weight_reg <= 0;
            weight_out <= 0;
            act_out    <= 0;
            psum_out   <= 0;
        end else begin
            // 1. Weight storage and pass-through
            if (weight_load) begin
                weight_reg <= weight_in;
                weight_out <= weight_in;
            end else begin
                weight_out <= weight_reg;
            end

            // 2. Activation pass-through
            if (pe_enable) begin
                act_out <= act_in;
            end

            // 3. MAC operation
            if (pe_enable) begin
                if (acc_clear) begin
                    psum_out <= weight_reg * act_in;
                end else if (!zero_detected) begin
                    psum_out <= psum_in + (weight_reg * act_in);
                end else begin
                    // Zero-skip logic: simply pass the previous sum
                    psum_out <= psum_in;
                end
            end
        end
    end

endmodule
