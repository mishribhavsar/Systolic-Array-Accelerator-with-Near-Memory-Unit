`timescale 1ns / 1ps
// ============================================================================
// Module      : bias_adder
// Description : Per-element bias addition with 32-bit saturating arithmetic.
//               1-cycle latency pipeline stage.
// ============================================================================
module bias_adder #(
    parameter ARRAY_SIZE = 8,
    parameter ACC_WIDTH  = 32
)(
    input  wire                                     clk,
    input  wire                                     rst_n,
    input  wire                                     enable,
    input  wire                                     valid_in,
    input  wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]   data_in_flat,
    input  wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]   bias_flat,
    output reg                                      valid_out,
    output reg  signed [ARRAY_SIZE*ACC_WIDTH-1:0]   data_out_flat
);

    localparam signed [ACC_WIDTH-1:0] MAX_POS = {1'b0, {(ACC_WIDTH-1){1'b1}}};
    localparam signed [ACC_WIDTH-1:0] MAX_NEG = {1'b1, {(ACC_WIDTH-1){1'b0}}};

    wire signed [ACC_WIDTH-1:0] data_in  [0:ARRAY_SIZE-1];
    wire signed [ACC_WIDTH-1:0] bias_in  [0:ARRAY_SIZE-1];

    genvar i;
    generate
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin : gen_unpack
            assign data_in[i] = data_in_flat[i*ACC_WIDTH +: ACC_WIDTH];
            assign bias_in[i] = bias_flat[i*ACC_WIDTH +: ACC_WIDTH];
        end
    endgenerate

    integer j;
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_out     <= 1'b0;
            data_out_flat <= {(ARRAY_SIZE*ACC_WIDTH){1'b0}};
        end else begin
            valid_out <= valid_in;
            
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                if (enable && valid_in) begin
                    // Perform signed addition with saturation
                    if ((data_in[j][ACC_WIDTH-1] == 1'b0) && (bias_in[j][ACC_WIDTH-1] == 1'b0) && 
                        ((data_in[j] + bias_in[j]) < 0)) begin
                        data_out_flat[j*ACC_WIDTH +: ACC_WIDTH] <= MAX_POS;
                    end else if ((data_in[j][ACC_WIDTH-1] == 1'b1) && (bias_in[j][ACC_WIDTH-1] == 1'b1) && 
                               ((data_in[j] + bias_in[j]) >= 0)) begin
                        data_out_flat[j*ACC_WIDTH +: ACC_WIDTH] <= MAX_NEG;
                    end else begin
                        data_out_flat[j*ACC_WIDTH +: ACC_WIDTH] <= data_in[j] + bias_in[j];
                    end
                end else if (!enable && valid_in) begin
                    data_out_flat[j*ACC_WIDTH +: ACC_WIDTH] <= data_in[j];
                end
            end
        end
    end

endmodule
