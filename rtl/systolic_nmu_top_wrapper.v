`timescale 1ns / 1ps

// =============================================================================
// Module:      systolic_nmu_top_wrapper
// Description: I/O pin-reduction wrapper for systolic_nmu_top.
//              17 physical pins with default parameters:
//              clk, rst_n, serial_in, serial_in_valid, serial_out_load,
//              serial_out, busy, done, output_valid, output_count[7:0].
//
// SERIAL PROTOCOL
//   Input frame  : IN_BITS bits, shifted MSB-first, one bit per clock.
//                  Assert serial_in_valid for ONE cycle after the last bit
//                  (serial_in is ignored in that cycle).
//                  Layout (MSB -> LSB):
//                  {cfg_we, cfg_addr, cfg_wdata, cfg_raddr,
//                   ext_waddr, ext_wdata, ext_weight_we, ext_act_we,
//                   bias_waddr, bias_wdata, bias_we, out_raddr}
//                  Write enables are issued as SINGLE-CYCLE strobes.
//                  Addresses / data stay latched until the next frame.
//   Output frame : OUT_BITS = {cfg_rdata[31:0], out_rdata}.
//                  Wait >= 2 clocks after the frame that set out_raddr /
//                  cfg_raddr (registered SRAM read), pulse serial_out_load for
//                  one cycle. serial_out shows the MSB right after that edge
//                  and one new bit after every following edge.
// =============================================================================

module systolic_nmu_top_wrapper #(
    parameter ARRAY_SIZE     = 8,
    parameter DATA_WIDTH     = 8,
    parameter ACC_WIDTH      = 32,
    parameter OUTPUT_WIDTH   = 16,
    parameter SRAM_DEPTH     = 256,
    parameter ADDR_WIDTH     = 8,
    parameter CFG_ADDR_WIDTH = 4
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Serial shift interface
    input  wire                  serial_in,
    input  wire                  serial_in_valid,
    input  wire                  serial_out_load,
    output wire                  serial_out,

    // Status
    output wire                  busy,
    output wire                  done,
    output wire                  output_valid,
    output wire [ADDR_WIDTH-1:0] output_count
);

    // =========================================================================
    // Frame widths
    // =========================================================================
    localparam INPUT_SHIFT_BITS  = 1 + CFG_ADDR_WIDTH + 32 + CFG_ADDR_WIDTH
                                 + ADDR_WIDTH + ARRAY_SIZE*DATA_WIDTH + 1 + 1
                                 + ADDR_WIDTH + ARRAY_SIZE*ACC_WIDTH + 1
                                 + ADDR_WIDTH;                  // 388 @ defaults
    localparam OUTPUT_SHIFT_BITS = 32 + ARRAY_SIZE*OUTPUT_WIDTH; // 160 @ defaults

    reg [INPUT_SHIFT_BITS-1:0]  input_shift_reg;
    reg [OUTPUT_SHIFT_BITS-1:0] output_shift_reg;

    // Latched core inputs
    reg                                     core_cfg_we;
    reg  [CFG_ADDR_WIDTH-1:0]               core_cfg_addr;
    reg  [31:0]                             core_cfg_wdata;
    reg  [CFG_ADDR_WIDTH-1:0]               core_cfg_raddr;
    reg  [ADDR_WIDTH-1:0]                   core_ext_waddr;
    reg  [ARRAY_SIZE*DATA_WIDTH-1:0]        core_ext_wdata;
    reg                                     core_ext_weight_we;
    reg                                     core_ext_act_we;
    reg  [ADDR_WIDTH-1:0]                   core_bias_waddr;
    reg  signed [ARRAY_SIZE*ACC_WIDTH-1:0]  core_bias_wdata;
    reg                                     core_bias_we;
    reg  [ADDR_WIDTH-1:0]                   core_out_raddr;

    // Core outputs
    wire [31:0]                             core_cfg_rdata;
    wire [ARRAY_SIZE*OUTPUT_WIDTH-1:0]      core_out_rdata;

    // =========================================================================
    // Input shift register + latch
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_shift_reg    <= {INPUT_SHIFT_BITS{1'b0}};
            core_cfg_we        <= 1'b0;
            core_cfg_addr      <= {CFG_ADDR_WIDTH{1'b0}};
            core_cfg_wdata     <= 32'd0;
            core_cfg_raddr     <= {CFG_ADDR_WIDTH{1'b0}};
            core_ext_waddr     <= {ADDR_WIDTH{1'b0}};
            core_ext_wdata     <= {(ARRAY_SIZE*DATA_WIDTH){1'b0}};
            core_ext_weight_we <= 1'b0;
            core_ext_act_we    <= 1'b0;
            core_bias_waddr    <= {ADDR_WIDTH{1'b0}};
            core_bias_wdata    <= {(ARRAY_SIZE*ACC_WIDTH){1'b0}};
            core_bias_we       <= 1'b0;
            core_out_raddr     <= {ADDR_WIDTH{1'b0}};
        end else begin
            input_shift_reg <= {input_shift_reg[INPUT_SHIFT_BITS-2:0], serial_in};

            // Strobes default low -> exactly one cycle per frame
            core_cfg_we        <= 1'b0;
            core_ext_weight_we <= 1'b0;
            core_ext_act_we    <= 1'b0;
            core_bias_we       <= 1'b0;

            if (serial_in_valid) begin
                {
                    core_cfg_we,
                    core_cfg_addr,
                    core_cfg_wdata,
                    core_cfg_raddr,
                    core_ext_waddr,
                    core_ext_wdata,
                    core_ext_weight_we,
                    core_ext_act_we,
                    core_bias_waddr,
                    core_bias_wdata,
                    core_bias_we,
                    core_out_raddr
                } <= input_shift_reg;
            end
        end
    end

    // =========================================================================
    // Output shift register
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            output_shift_reg <= {OUTPUT_SHIFT_BITS{1'b0}};
        else if (serial_out_load)
            output_shift_reg <= {core_cfg_rdata, core_out_rdata};
        else
            output_shift_reg <= {output_shift_reg[OUTPUT_SHIFT_BITS-2:0], 1'b0};
    end

    assign serial_out = output_shift_reg[OUTPUT_SHIFT_BITS-1];

    // =========================================================================
    // Core
    // =========================================================================
    systolic_nmu_top #(
        .ARRAY_SIZE     (ARRAY_SIZE),
        .DATA_WIDTH     (DATA_WIDTH),
        .ACC_WIDTH      (ACC_WIDTH),
        .OUTPUT_WIDTH   (OUTPUT_WIDTH),
        .SRAM_DEPTH     (SRAM_DEPTH),
        .ADDR_WIDTH     (ADDR_WIDTH),
        .CFG_ADDR_WIDTH (CFG_ADDR_WIDTH)
    ) u_core (
        .clk            (clk),
        .rst_n          (rst_n),
        .cfg_we         (core_cfg_we),
        .cfg_addr       (core_cfg_addr),
        .cfg_wdata      (core_cfg_wdata),
        .cfg_raddr      (core_cfg_raddr),
        .cfg_rdata      (core_cfg_rdata),
        .ext_waddr      (core_ext_waddr),
        .ext_wdata      (core_ext_wdata),
        .ext_weight_we  (core_ext_weight_we),
        .ext_act_we     (core_ext_act_we),
        .bias_waddr     (core_bias_waddr),
        .bias_wdata     (core_bias_wdata),
        .bias_we        (core_bias_we),
        .out_raddr      (core_out_raddr),
        .out_rdata      (core_out_rdata),
        .busy           (busy),
        .done           (done),
        .output_valid   (output_valid),
        .output_count   (output_count)
    );

endmodule