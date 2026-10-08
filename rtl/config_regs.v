`timescale 1ns / 1ps

/*
 * Module: config_regs
 * Description: Runtime Configuration Registers for Systolic Array NMU.
 * Holds runtime-configurable parameters accessible via simple write interface.
 */
module config_regs #(
    parameter ADDR_WIDTH = 4
)(
    input  wire        clk,
    input  wire        rst_n,
    // Write interface
    input  wire        cfg_we,
    input  wire [ADDR_WIDTH-1:0] cfg_addr,
    input  wire [31:0] cfg_wdata,
    // Read interface
    input  wire [ADDR_WIDTH-1:0] cfg_raddr,
    output reg  [31:0] cfg_rdata,
    // Decoded outputs
    output wire [15:0] matrix_m_size,   
    output wire [15:0] matrix_k_size,   
    output wire [15:0] matrix_n_size,   
    output wire [1:0]  activation_mode, 
    output wire        pooling_enable,  
    output wire [3:0]  quantize_shift,  
    output wire [3:0]  leaky_shift,     
    output wire        start_pulse,     
    output wire [15:0] bias_base_addr   
);

    reg [31:0] regs [0:15];
    reg start_pulse_reg;
    
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 16; i = i + 1) begin
                if (i == 1 || i == 2 || i == 3) regs[i] <= 32'd8; // MAT_M, MAT_K, MAT_N
                else if (i == 6) regs[i] <= 32'd8; // QUANT
                else if (i == 7) regs[i] <= 32'd3; // LEAK
                else regs[i] <= 32'd0;
            end
            start_pulse_reg <= 1'b0;
        end else begin
            start_pulse_reg <= 1'b0; // default clear

            if (cfg_we) begin
                if (cfg_addr == 0) begin
                    start_pulse_reg <= cfg_wdata[0];
                end else begin
                    regs[cfg_addr] <= cfg_wdata;
                end
            end
        end
    end

    always @(*) begin
        if (cfg_raddr == 0) cfg_rdata = {31'b0, start_pulse_reg};
        else cfg_rdata = regs[cfg_raddr];
    end

    assign start_pulse     = start_pulse_reg;
    assign matrix_m_size   = regs[1][15:0];
    assign matrix_k_size   = regs[2][15:0];
    assign matrix_n_size   = regs[3][15:0];
    assign activation_mode = regs[4][1:0];
    assign pooling_enable  = regs[5][0];
    assign quantize_shift  = regs[6][3:0];
    assign leaky_shift     = regs[7][3:0];
    assign bias_base_addr  = regs[8][15:0];

endmodule
