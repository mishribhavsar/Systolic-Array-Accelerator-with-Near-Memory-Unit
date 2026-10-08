`timescale 1ns / 1ps

// =============================================================================
// Module:      near_memory_input
// Description: NMU input unit. Holds the weight and activation operands in
//              on-chip Block RAM and performs zero detection on the
//              activation row being fed to the systolic array.
//
//              - Weight BRAM     : SRAM_DEPTH x (ARRAY_SIZE*DATA_WIDTH) bits
//                                  word k = row k of the weight matrix
//              - Activation BRAM : SRAM_DEPTH x (ARRAY_SIZE*DATA_WIDTH) bits
//                                  word m = activation vector m
//              - Synchronous read, 1-cycle latency (hidden by the address
//                generator prefetch)
//
//              The host writes both memories through a shared address/data
//              bus with separate write enables, before the start pulse.
//
//              buffer_swap is kept on the port list for a future ping-pong
//              (double-buffered) extension; it is not used in this version.
// =============================================================================
module near_memory_input #(
    parameter ARRAY_SIZE  = 8,
    parameter DATA_WIDTH  = 8,
    parameter SRAM_DEPTH  = 256,
    parameter ADDR_WIDTH  = 8
)(
    input  wire                                    clk,
    input  wire                                    rst_n,
    // Control from FSM
    input  wire                                    load_weights,     // FSM reading weights from SRAM
    input  wire                                    load_activations, // FSM reading activations from SRAM
    input  wire                                    compute_start,    // begin feeding data to array
    input  wire                                    buffer_swap,      // reserved (unused)

    // External write port - shared address and data bus
    input  wire [ADDR_WIDTH-1:0]                   ext_waddr,
    input  wire [ARRAY_SIZE*DATA_WIDTH-1:0]        ext_wdata,

    // Separate write enables for weight and activation SRAMs
    input  wire                                    ext_weight_we,    // write to weight SRAM
    input  wire                                    ext_act_we,       // write to activation SRAM

    // Weight read port
    output wire signed [ARRAY_SIZE*DATA_WIDTH-1:0] weight_out_flat,
    input  wire [ADDR_WIDTH-1:0]                   weight_raddr,

    // Activation read port
    output wire signed [ARRAY_SIZE*DATA_WIDTH-1:0] act_out_flat,
    input  wire [ADDR_WIDTH-1:0]                   act_raddr,

    // Zero detection output (one flag per activation lane)
    output wire [ARRAY_SIZE-1:0]                   act_zero_flags
);

    // Weight BRAM
    sram_bank #(
        .DATA_WIDTH(ARRAY_SIZE*DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .SRAM_DEPTH(SRAM_DEPTH)
    ) weight_bank (
        .clk   (clk),
        .we    (ext_weight_we),
        .waddr (ext_waddr),
        .wdata (ext_wdata),
        .raddr (weight_raddr),
        .rdata (weight_out_flat)
    );

    // Activation BRAM
    sram_bank #(
        .DATA_WIDTH(ARRAY_SIZE*DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .SRAM_DEPTH(SRAM_DEPTH)
    ) act_bank (
        .clk   (clk),
        .we    (ext_act_we),
        .waddr (ext_waddr),
        .wdata (ext_wdata),
        .raddr (act_raddr),
        .rdata (act_out_flat)
    );

    // Zero detection
    genvar i;
    generate
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin : ZERO_DET
            assign act_zero_flags[i] = (act_out_flat[i*DATA_WIDTH +: DATA_WIDTH] == {DATA_WIDTH{1'b0}});
        end
    endgenerate

endmodule
