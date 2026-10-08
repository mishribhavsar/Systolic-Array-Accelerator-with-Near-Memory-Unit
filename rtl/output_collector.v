`timescale 1ns / 1ps

// =============================================================================
// Module:      output_collector
// Description: Collects skewed outputs from the systolic array and de-skews
//              them using delay chains.
// =============================================================================

module output_collector #(
    parameter ARRAY_SIZE = 8,
    parameter ACC_WIDTH  = 32
)(
    input  wire                                    clk,
    input  wire                                    rst_n,
    input  wire                                    drain_enable,    // enable output collection
    input  wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]  psum_in_flat,   // from bottom row
    output reg  signed [ARRAY_SIZE*ACC_WIDTH-1:0]  result_flat,    // de-skewed output row
    output reg                                     result_valid    // pulse when result_flat is valid
);

    // 2D Array for unpacking
    wire signed [ACC_WIDTH-1:0] psum_in [0:ARRAY_SIZE-1];
    integer s;
    genvar col;
    generate
        for (col = 0; col < ARRAY_SIZE; col = col + 1) begin : unpack
            assign psum_in[col] = psum_in_flat[(col*ACC_WIDTH) +: ACC_WIDTH];
        end
    endgenerate

    // Delay chains for de-skewing
    // Delay for column j = ARRAY_SIZE - 1 - j
    
    // We'll declare a 3D array of registers for the delay chain
    // delay_chain[col][stage]
    // To handle Verilog limitations with variable sized genvars, 
    // we define the max possible stages for each col.
    
    genvar j, k;
    generate
        for (j = 0; j < ARRAY_SIZE; j = j + 1) begin : deskew_col
            localparam DELAY = ARRAY_SIZE - 1 - j;
            
            if (DELAY == 0) begin
                // No delay for the last column
                always @(*) begin
                    result_flat[(j*ACC_WIDTH) +: ACC_WIDTH] = psum_in[j];
                end
            end else begin
                // Pipeline stages
                reg signed [ACC_WIDTH-1:0] shift_reg [0:DELAY-1];
                
                always @(posedge clk) begin
                    if (!rst_n) begin
                        //integer s;
                        for (s = 0; s < DELAY; s = s + 1) begin
                            shift_reg[s] <= 0;
                        end
                    end else if (drain_enable) begin
                        shift_reg[0] <= psum_in[j];
                        
                        for (s = 1; s < DELAY; s = s + 1) begin
                            shift_reg[s] <= shift_reg[s-1];
                        end
                    end
                end
                
                always @(*) begin
                    result_flat[(j*ACC_WIDTH) +: ACC_WIDTH] = shift_reg[DELAY-1];
                end
            end
        end
    endgenerate

    // result_valid logic
    // Track when the first complete row is ready
    reg [31:0] drain_cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            drain_cnt <= 0;
            result_valid <= 0;
        end else if (drain_enable) begin
            if (drain_cnt < (ARRAY_SIZE - 1 + ARRAY_SIZE)) begin
                drain_cnt <= drain_cnt + 1;
            end
            
            if (drain_cnt >= (ARRAY_SIZE - 1) && drain_cnt < (ARRAY_SIZE - 1 + ARRAY_SIZE)) begin
                result_valid <= 1'b1;
            end else begin
                result_valid <= 1'b0;
            end
        end else begin
            drain_cnt <= 0;
            result_valid <= 0;
        end
    end

endmodule
