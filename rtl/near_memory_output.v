`timescale 1ns / 1ps
// ============================================================================
// Module      : near_memory_output
// Description : NMU Output Pipeline integrating post-processing and SRAM buffer.
//               bias -> activation -> quantize -> pool -> SRAM
// ============================================================================
module near_memory_output #(
    parameter ARRAY_SIZE    = 8,
    parameter ACC_WIDTH     = 32,
    parameter OUTPUT_WIDTH  = 16,
    parameter SRAM_DEPTH    = 256,
    parameter ADDR_WIDTH    = 8
)(
    input  wire                                         clk,
    input  wire                                         rst_n,
    
    // Input from output collector
    input  wire                                         result_valid,
    input  wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]       result_flat,
    
    // Bias values
    input  wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]       bias_flat,
    
    // Configuration
    input  wire [1:0]                                   activation_mode,
    input  wire [3:0]                                   leaky_shift,
    input  wire [3:0]                                   quantize_shift,
    input  wire                                         pooling_enable,
    input  wire                                         post_proc_enable,
    
    // Output SRAM read interface (for external consumption)
    input  wire [ADDR_WIDTH-1:0]                        out_raddr,
    output wire [ARRAY_SIZE*OUTPUT_WIDTH-1:0]           out_rdata,
    
    // Status
    output wire                                         output_valid,
    output wire [ADDR_WIDTH-1:0]                        output_count
);

    // -------------------------------------------------------------------------
    // Signals
    // -------------------------------------------------------------------------
    wire                                        bias_valid_out;
    wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]      bias_data_out;
    
    wire                                        act_valid_out;
    wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]      act_data_out;
    
    wire                                        quant_valid_out;
    wire signed [ARRAY_SIZE*OUTPUT_WIDTH-1:0]   quant_data_out;
    
    wire                                        pool_valid_out;
    wire signed [(ARRAY_SIZE/2)*OUTPUT_WIDTH-1:0] pool_data_out;
    wire signed [ARRAY_SIZE*OUTPUT_WIDTH-1:0]   pool_bypass_out;
    
    // -------------------------------------------------------------------------
    // Pipeline Instantiations
    // -------------------------------------------------------------------------
    
    bias_adder #(
        .ARRAY_SIZE(ARRAY_SIZE),
        .ACC_WIDTH (ACC_WIDTH)
    ) u_bias_adder (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (post_proc_enable),
        .valid_in     (result_valid),
        .data_in_flat (result_flat),
        .bias_flat    (bias_flat),
        .valid_out    (bias_valid_out),
        .data_out_flat(bias_data_out)
    );

    activation_unit #(
        .ARRAY_SIZE(ARRAY_SIZE),
        .ACC_WIDTH (ACC_WIDTH)
    ) u_activation_unit (
        .clk          (clk),
        .rst_n        (rst_n),
        .mode         (post_proc_enable ? activation_mode : 2'b00),
        .leaky_shift  (leaky_shift),
        .valid_in     (bias_valid_out),
        .data_in_flat (bias_data_out),
        .valid_out    (act_valid_out),
        .data_out_flat(act_data_out)
    );

    quantizer #(
        .ARRAY_SIZE  (ARRAY_SIZE),
        .ACC_WIDTH   (ACC_WIDTH),
        .OUTPUT_WIDTH(OUTPUT_WIDTH)
    ) u_quantizer (
        .clk          (clk),
        .rst_n        (rst_n),
        .shift_amount (post_proc_enable ? quantize_shift : 4'd0),
        .valid_in     (act_valid_out),
        .data_in_flat (act_data_out),
        .valid_out    (quant_valid_out),
        .data_out_flat(quant_data_out)
    );

    pooling_unit #(
        .ARRAY_SIZE  (ARRAY_SIZE),
        .OUTPUT_WIDTH(OUTPUT_WIDTH)
    ) u_pooling_unit (
        .clk              (clk),
        .rst_n            (rst_n),
        .enable           (pooling_enable & post_proc_enable),
        .valid_in         (quant_valid_out),
        .data_in_flat     (quant_data_out),
        .valid_out        (pool_valid_out),
        .data_out_flat    (pool_data_out),
        .data_bypass_flat (pool_bypass_out)
    );

    // -------------------------------------------------------------------------
    // Output SRAM Selection & Write Logic
    // -------------------------------------------------------------------------
    
    wire sram_wen;
    wire [ARRAY_SIZE*OUTPUT_WIDTH-1:0] sram_wdata;
    
    assign sram_wen = pool_valid_out;
    
    // Mux for final SRAM write data based on pooling enable
    assign sram_wdata = (pooling_enable & post_proc_enable) ? 
                        {{(ARRAY_SIZE/2*OUTPUT_WIDTH){1'b0}}, pool_data_out} : 
                        pool_bypass_out;

    // Counter for SRAM Write Address / output count
    reg [ADDR_WIDTH-1:0] waddr_cnt;
    
    always @(posedge clk) begin
        if (!rst_n || !post_proc_enable) begin
            waddr_cnt <= {ADDR_WIDTH{1'b0}};
        end else if (sram_wen) begin
            waddr_cnt <= waddr_cnt + 1;
        end
    end
    
    assign output_count = waddr_cnt;
    assign output_valid = sram_wen;

    // -------------------------------------------------------------------------
    // SRAM Bank Inference (Xilinx Block RAM)
    // -------------------------------------------------------------------------
    
    (* ram_style = "block" *) 
    reg [ARRAY_SIZE*OUTPUT_WIDTH-1:0] out_sram [0:SRAM_DEPTH-1];
    reg [ARRAY_SIZE*OUTPUT_WIDTH-1:0] sram_rdata_reg;

    always @(posedge clk) begin
        // Write Port
        if (sram_wen) begin
            out_sram[waddr_cnt] <= sram_wdata;
        end
        // Read Port
        sram_rdata_reg <= out_sram[out_raddr];
    end
    
    assign out_rdata = sram_rdata_reg;

endmodule
