`timescale 1ns / 1ps
// ============================================================================
// Module      : pooling_unit
// Description : Performs 2x2 max pooling with stride 2 on quantized rows.
//               Output valid on every other input row pair.
// ============================================================================
module pooling_unit #(
    parameter ARRAY_SIZE    = 8,
    parameter OUTPUT_WIDTH  = 16
)(
    input  wire                                         clk,
    input  wire                                         rst_n,
    input  wire                                         enable,         // 1=pool, 0=bypass
    input  wire                                         valid_in,
    input  wire signed [ARRAY_SIZE*OUTPUT_WIDTH-1:0]    data_in_flat,   // current row
    output reg                                          valid_out,
    output reg  signed [(ARRAY_SIZE/2)*OUTPUT_WIDTH-1:0] data_out_flat,  // pooled row
    output reg  signed [ARRAY_SIZE*OUTPUT_WIDTH-1:0]    data_bypass_flat // unpooled pass-through
);

    wire signed [OUTPUT_WIDTH-1:0] data_in [0:ARRAY_SIZE-1];
    
    // Internal row buffer for pooling
    reg signed [OUTPUT_WIDTH-1:0] row_buffer [0:ARRAY_SIZE-1];
    reg row_parity;

    genvar i;
    generate
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin : gen_unpack
            assign data_in[i] = data_in_flat[i*OUTPUT_WIDTH +: OUTPUT_WIDTH];
        end
    endgenerate

    integer j;
    reg signed [OUTPUT_WIDTH-1:0] max_row0;
    reg signed [OUTPUT_WIDTH-1:0] max_row1;
    reg signed [OUTPUT_WIDTH-1:0] pool_max;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            data_out_flat <= {((ARRAY_SIZE/2)*OUTPUT_WIDTH){1'b0}};
            data_bypass_flat <= {(ARRAY_SIZE*OUTPUT_WIDTH){1'b0}};
            row_parity <= 1'b0;
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                row_buffer[j] <= {OUTPUT_WIDTH{1'b0}};
            end
        end else begin
            if (!enable) begin
                valid_out <= valid_in;
                data_bypass_flat <= data_in_flat;
                row_parity <= 1'b0; // reset pooling state
            end else begin
                // Passing through bypass regardless for continuity if needed
                data_bypass_flat <= data_in_flat;
                
                if (valid_in) begin
                    if (row_parity == 1'b0) begin
                        // Store even row
                        for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                            row_buffer[j] <= data_in[j];
                        end
                        row_parity <= 1'b1;
                        valid_out <= 1'b0;
                    end else begin
                        // Process odd row with buffered even row
                        for (j = 0; j < ARRAY_SIZE/2; j = j + 1) begin
                            // Calculate max of 2 elements from row 0
                            max_row0 = (row_buffer[2*j] > row_buffer[2*j+1]) ? row_buffer[2*j] : row_buffer[2*j+1];
                            // Calculate max of 2 elements from row 1 (current)
                            max_row1 = (data_in[2*j] > data_in[2*j+1]) ? data_in[2*j] : data_in[2*j+1];
                            // Final max for this 2x2 block
                            pool_max = (max_row0 > max_row1) ? max_row0 : max_row1;
                            
                            data_out_flat[j*OUTPUT_WIDTH +: OUTPUT_WIDTH] <= pool_max;
                        end
                        row_parity <= 1'b0;
                        valid_out <= 1'b1;
                    end
                end else begin
                    valid_out <= 1'b0;
                end
            end
        end
    end

endmodule
