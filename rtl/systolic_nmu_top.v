`timescale 1ns / 1ps
// =============================================================================
// Module:      systolic_nmu_top
// Description: Top-level integration of the 8x8 weight-stationary systolic
//              array with the Near-Memory Unit (NMU).
//                config_regs -> controller_fsm
//                near_memory_input (weight / activation BRAM) -> input skew
//                systolic_array_core -> output_collector (de-skew)
//                near_memory_output (bias -> act -> quantize -> pool -> out BRAM)
//              The bias BRAM lives here and is read one row per collector
//              valid, starting at BIAS_BASE_ADDR.
// Note:        Bias read uses the *next* address so that bias_data_reg always
//              matches bias_sram[bias_raddr_reg]; the pointer is reloaded during
//              the weight-load phase, before the first result row arrives.
// =============================================================================
module systolic_nmu_top #(
    parameter ARRAY_SIZE    = 8,
    parameter DATA_WIDTH    = 8,
    parameter ACC_WIDTH     = 32,
    parameter OUTPUT_WIDTH  = 16,
    parameter SRAM_DEPTH    = 256,
    parameter ADDR_WIDTH    = 8,
    parameter CFG_ADDR_WIDTH = 4
)(
    input  wire                                     clk,
    input  wire                                     rst_n,
    input  wire                                     cfg_we,
    input  wire [CFG_ADDR_WIDTH-1:0]                cfg_addr,
    input  wire [31:0]                              cfg_wdata,
    input  wire [CFG_ADDR_WIDTH-1:0]                cfg_raddr,
    output wire [31:0]                              cfg_rdata,
    input  wire [ADDR_WIDTH-1:0]                    ext_waddr,
    input  wire [ARRAY_SIZE*DATA_WIDTH-1:0]         ext_wdata,
    input  wire                                     ext_weight_we,
    input  wire                                     ext_act_we,
    input  wire [ADDR_WIDTH-1:0]                    bias_waddr,
    input  wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]   bias_wdata,
    input  wire                                     bias_we,
    input  wire [ADDR_WIDTH-1:0]                    out_raddr,
    output wire [ARRAY_SIZE*OUTPUT_WIDTH-1:0]       out_rdata,
    output wire                                     busy,
    output wire                                     done,
    output wire                                     output_valid,
    output wire [ADDR_WIDTH-1:0]                    output_count
);
    wire [15:0] matrix_m_size, matrix_k_size, matrix_n_size;
    wire [1:0]  activation_mode;
    wire        pooling_enable;
    wire [3:0]  quantize_shift, leaky_shift;
    wire        start_pulse;
    wire [15:0] bias_base_addr;

    wire        fsm_weight_load_en;
    wire [ADDR_WIDTH-1:0] fsm_weight_load_addr;
    wire        fsm_weight_load_done;
    wire        fsm_act_feed_en;
    wire [ADDR_WIDTH-1:0] fsm_act_feed_addr;
    wire        fsm_compute_enable;
    wire        fsm_acc_clear;
    wire        fsm_drain_enable;
    wire        fsm_drain_done;
    wire        fsm_buffer_swap;
    wire        fsm_post_proc_enable;
    wire        fsm_post_proc_valid;
    wire [15:0] current_tile_m, current_tile_n, current_tile_k;

    wire signed [ARRAY_SIZE*DATA_WIDTH-1:0] nmu_weight_out;
    wire signed [ARRAY_SIZE*DATA_WIDTH-1:0] nmu_act_out;
    wire [ARRAY_SIZE-1:0]                   nmu_act_zero_flags;
    wire signed [ARRAY_SIZE*DATA_WIDTH-1:0] skewed_act_flat;
    wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]  core_psum_out;
    wire [ARRAY_SIZE-1:0]                   core_zero_flags;
    wire signed [ARRAY_SIZE*ACC_WIDTH-1:0]  collector_result;
    wire                                    collector_result_valid;

    (* ram_style = "block" *)
    reg signed [ARRAY_SIZE*ACC_WIDTH-1:0] bias_sram [0:SRAM_DEPTH-1];
    reg signed [ARRAY_SIZE*ACC_WIDTH-1:0] bias_data_reg;
    reg [ADDR_WIDTH-1:0] bias_raddr_reg;
    wire [ADDR_WIDTH-1:0] bias_raddr_next;

    config_regs #(.ADDR_WIDTH(CFG_ADDR_WIDTH)) u_config_regs (
        .clk(clk), .rst_n(rst_n), .cfg_we(cfg_we), .cfg_addr(cfg_addr),
        .cfg_wdata(cfg_wdata), .cfg_raddr(cfg_raddr), .cfg_rdata(cfg_rdata),
        .matrix_m_size(matrix_m_size), .matrix_k_size(matrix_k_size),
        .matrix_n_size(matrix_n_size), .activation_mode(activation_mode),
        .pooling_enable(pooling_enable), .quantize_shift(quantize_shift),
        .leaky_shift(leaky_shift), .start_pulse(start_pulse),
        .bias_base_addr(bias_base_addr)
    );

    controller_fsm #(.ARRAY_SIZE(ARRAY_SIZE), .ADDR_WIDTH(ADDR_WIDTH)) u_controller_fsm (
        .clk(clk), .rst_n(rst_n), .start(start_pulse), .done(done), .busy(busy),
        .matrix_m_size(matrix_m_size), .matrix_k_size(matrix_k_size),
        .matrix_n_size(matrix_n_size),
        .weight_load_en(fsm_weight_load_en), .weight_load_addr(fsm_weight_load_addr),
        .weight_load_done(fsm_weight_load_done),
        .act_feed_en(fsm_act_feed_en), .act_feed_addr(fsm_act_feed_addr),
        .compute_enable(fsm_compute_enable), .acc_clear(fsm_acc_clear),
        .drain_enable(fsm_drain_enable), .drain_done(fsm_drain_done),
        .buffer_swap(fsm_buffer_swap),
        .post_proc_enable(fsm_post_proc_enable), .post_proc_valid(fsm_post_proc_valid),
        .current_tile_m(current_tile_m), .current_tile_n(current_tile_n),
        .current_tile_k(current_tile_k)
    );

    near_memory_input #(
        .ARRAY_SIZE(ARRAY_SIZE), .DATA_WIDTH(DATA_WIDTH),
        .SRAM_DEPTH(SRAM_DEPTH), .ADDR_WIDTH(ADDR_WIDTH)
    ) u_near_memory_input (
        .clk(clk), .rst_n(rst_n),
        .load_weights(fsm_weight_load_en), .load_activations(fsm_act_feed_en),
        .compute_start(fsm_compute_enable), .buffer_swap(fsm_buffer_swap),
        .ext_waddr(ext_waddr), .ext_wdata(ext_wdata),
        .ext_weight_we(ext_weight_we), .ext_act_we(ext_act_we),
        .weight_out_flat(nmu_weight_out), .weight_raddr(fsm_weight_load_addr),
        .act_out_flat(nmu_act_out), .act_raddr(fsm_act_feed_addr),
        .act_zero_flags(nmu_act_zero_flags)
    );

    // Input skew: row i delayed by i + 1 cycles
    genvar row_idx, s;
    generate
        for (row_idx = 0; row_idx < ARRAY_SIZE; row_idx = row_idx + 1) begin : gen_skew
            wire signed [DATA_WIDTH-1:0] row_act_in;
            assign row_act_in = nmu_act_out[(row_idx*DATA_WIDTH) +: DATA_WIDTH];
            if (row_idx == 0) begin : no_skew
                reg signed [DATA_WIDTH-1:0] delay_reg;
                always @(posedge clk) begin
                    if (!rst_n)                                   delay_reg <= {DATA_WIDTH{1'b0}};
                    else if (fsm_act_feed_en || fsm_compute_enable) delay_reg <= row_act_in;
                    else                                          delay_reg <= {DATA_WIDTH{1'b0}};
                end
                assign skewed_act_flat[DATA_WIDTH-1:0] = delay_reg;
            end else begin : with_skew
                reg signed [DATA_WIDTH-1:0] shift_reg [0:row_idx];
                always @(posedge clk) begin
                    if (!rst_n)                                   shift_reg[0] <= {DATA_WIDTH{1'b0}};
                    else if (fsm_act_feed_en || fsm_compute_enable) shift_reg[0] <= row_act_in;
                    else                                          shift_reg[0] <= {DATA_WIDTH{1'b0}};
                end
                for (s = 1; s <= row_idx; s = s + 1) begin : shift_stages
                    always @(posedge clk) begin
                        if (!rst_n)                                   shift_reg[s] <= {DATA_WIDTH{1'b0}};
                        else if (fsm_act_feed_en || fsm_compute_enable) shift_reg[s] <= shift_reg[s-1];
                        else                                          shift_reg[s] <= {DATA_WIDTH{1'b0}};
                    end
                end
                assign skewed_act_flat[(row_idx*DATA_WIDTH) +: DATA_WIDTH] = shift_reg[row_idx];
            end
        end
    endgenerate

    systolic_array_core #(
        .ARRAY_SIZE(ARRAY_SIZE), .DATA_WIDTH(DATA_WIDTH), .ACC_WIDTH(ACC_WIDTH)
    ) u_systolic_array_core (
        .clk(clk), .rst_n(rst_n), .compute_enable(fsm_compute_enable),
        .weight_load_en(fsm_weight_load_en), .acc_clear(fsm_acc_clear),
        .weight_in_flat(nmu_weight_out), .act_in_flat(skewed_act_flat),
        .psum_out_flat(core_psum_out), .zero_flags(core_zero_flags)
    );

    output_collector #(.ARRAY_SIZE(ARRAY_SIZE), .ACC_WIDTH(ACC_WIDTH)) u_output_collector (
        .clk(clk), .rst_n(rst_n), .drain_enable(fsm_drain_enable),
        .psum_in_flat(core_psum_out), .result_flat(collector_result),
        .result_valid(collector_result_valid)
    );

    // =========================================================================
    // 7. Bias SRAM (Block RAM, next-address read for row alignment)
    // =========================================================================
    integer bi;
    initial for (bi = 0; bi < SRAM_DEPTH; bi = bi + 1) bias_sram[bi] = {(ARRAY_SIZE*ACC_WIDTH){1'b0}};

    assign bias_raddr_next = fsm_weight_load_en     ? bias_base_addr[ADDR_WIDTH-1:0] :
                             collector_result_valid ? bias_raddr_reg + 1'b1 :
                                                      bias_raddr_reg;

    always @(posedge clk) begin
        if (!rst_n)
            bias_raddr_reg <= {ADDR_WIDTH{1'b0}};
        else
            bias_raddr_reg <= bias_raddr_next;
    end

    always @(posedge clk) begin
        if (bias_we)
            bias_sram[bias_waddr] <= bias_wdata;
        bias_data_reg <= bias_sram[bias_raddr_next];   // == bias_sram[bias_raddr_reg] next cycle
    end

    near_memory_output #(
        .ARRAY_SIZE(ARRAY_SIZE), .ACC_WIDTH(ACC_WIDTH), .OUTPUT_WIDTH(OUTPUT_WIDTH),
        .SRAM_DEPTH(SRAM_DEPTH), .ADDR_WIDTH(ADDR_WIDTH)
    ) u_near_memory_output (
        .clk(clk), .rst_n(rst_n),
        .result_valid(collector_result_valid), .result_flat(collector_result),
        .bias_flat(bias_data_reg), .activation_mode(activation_mode),
        .leaky_shift(leaky_shift), .quantize_shift(quantize_shift),
        .pooling_enable(pooling_enable), .post_proc_enable(fsm_post_proc_enable),
        .out_raddr(out_raddr), .out_rdata(out_rdata),
        .output_valid(output_valid), .output_count(output_count)
    );
endmodule
