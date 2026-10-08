`timescale 1ns / 1ps
// ============================================================================
// Module      : quantizer
// Description : Converts 32-bit values to 16-bit with configurable right-shift
//               and saturation clipping. 1-cycle latency pipeline stage.
// ============================================================================
module quantizer #(
    parameter ARRAY_SIZE    = 8,
    parameter ACC_WIDTH     = 32,
    parameter OUTPUT_WIDTH  = 16
)(
    input  wire                                         clk,
    input  wire                                         rst_n,
    input  wire [3:0]                                   shift_amount,
    input  wire                                         valid_in,
    input  wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]       data_in_flat,
    output reg                                          valid_out,
    output reg  signed [ARRAY_SIZE*OUTPUT_WIDTH-1:0]    data_out_flat
);

    localparam signed [ACC_WIDTH-1:0] MAX_POS = (1 << (OUTPUT_WIDTH-1)) - 1;   // 32767 for 16-bit
    localparam signed [ACC_WIDTH-1:0] MAX_NEG = -(1 << (OUTPUT_WIDTH-1));      // -32768 for 16-bit

    wire signed [ACC_WIDTH-1:0] data_in [0:ARRAY_SIZE-1];

    genvar i;
    generate
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin : gen_unpack
            assign data_in[i] = data_in_flat[i*ACC_WIDTH +: ACC_WIDTH];
        end
    endgenerate

    integer j;
    reg signed [ACC_WIDTH-1:0] shifted_val;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_out     <= 1'b0;
            data_out_flat <= {(ARRAY_SIZE*OUTPUT_WIDTH){1'b0}};
        end else begin
            valid_out <= valid_in;

            if (valid_in) begin
                for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                    shifted_val = data_in[j] >>> shift_amount;

                    if (shifted_val > MAX_POS) begin
                        data_out_flat[j*OUTPUT_WIDTH +: OUTPUT_WIDTH] <= MAX_POS[OUTPUT_WIDTH-1:0];
                    end else if (shifted_val < MAX_NEG) begin
                        data_out_flat[j*OUTPUT_WIDTH +: OUTPUT_WIDTH] <= MAX_NEG[OUTPUT_WIDTH-1:0];
                    end else begin
                        data_out_flat[j*OUTPUT_WIDTH +: OUTPUT_WIDTH] <= shifted_val[OUTPUT_WIDTH-1:0];
                    end
                end
            end
        end
    end

endmodule 