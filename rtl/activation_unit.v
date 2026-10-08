`timescale 1ns / 1ps
// ============================================================================
// Module      : activation_unit
// Description : Applies Configurable Activation Function (ReLU, Leaky ReLU).
//               1-cycle latency pipeline stage.
// ============================================================================
module activation_unit #(
    parameter ARRAY_SIZE = 8,
    parameter ACC_WIDTH  = 32
)(
    input  wire                                     clk,
    input  wire                                     rst_n,
    input  wire [1:0]                               mode,           // 0=bypass, 1=ReLU, 2=Leaky ReLU
    input  wire [3:0]                               leaky_shift,    // Leaky slope = 2^(-leaky_shift)
    input  wire                                     valid_in,
    input  wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]   data_in_flat,
    output reg                                      valid_out,
    output reg  signed [ARRAY_SIZE*ACC_WIDTH-1:0]   data_out_flat
);

    wire signed [ACC_WIDTH-1:0] data_in  [0:ARRAY_SIZE-1];

    genvar i;
    generate
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin : gen_unpack
            assign data_in[i] = data_in_flat[i*ACC_WIDTH +: ACC_WIDTH];
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
                if (valid_in) begin
                    case (mode)
                        2'd1: begin // ReLU
                            if (data_in[j] > 0)
                                data_out_flat[j*ACC_WIDTH +: ACC_WIDTH] <= data_in[j];
                            else
                                data_out_flat[j*ACC_WIDTH +: ACC_WIDTH] <= {ACC_WIDTH{1'b0}};
                        end
                        2'd2: begin // Leaky ReLU
                            if (data_in[j] > 0)
                                data_out_flat[j*ACC_WIDTH +: ACC_WIDTH] <= data_in[j];
                            else
                                data_out_flat[j*ACC_WIDTH +: ACC_WIDTH] <= data_in[j] >>> leaky_shift;
                        end
                        default: begin // Bypass (Mode 0)
                            data_out_flat[j*ACC_WIDTH +: ACC_WIDTH] <= data_in[j];
                        end
                    endcase
                end
            end
        end
    end

endmodule
