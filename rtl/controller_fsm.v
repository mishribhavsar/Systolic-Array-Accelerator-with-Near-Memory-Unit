`timescale 1ns / 1ps

/*
 * Module: controller_fsm
 * Description: Master Controller FSM for Systolic Array NMU.
 * 6-state FSM that orchestrates the entire systolic array NMU operation.
 */
module controller_fsm #(
    parameter ARRAY_SIZE = 8,
    parameter ADDR_WIDTH = 8
)(
    input  wire                    clk,
    input  wire                    rst_n,
    
    // Start/done
    input  wire                    start,              
    output reg                     done,               
    output reg                     busy,               
    
    // Matrix dimensions
    input  wire [15:0]             matrix_m_size,
    input  wire [15:0]             matrix_k_size,
    input  wire [15:0]             matrix_n_size,
    
    // Weight loading control
    output reg                     weight_load_en,     
    output reg  [ADDR_WIDTH-1:0]   weight_load_addr,   
    output reg                     weight_load_done,
    
    // Activation feeding control
    output reg                     act_feed_en,        
    output reg  [ADDR_WIDTH-1:0]   act_feed_addr,      
    
    // Array control
    output reg                     compute_enable,     
    output reg                     acc_clear,          
    
    // Output drain control
    output reg                     drain_enable,       
    output reg                     drain_done,
    
    // NMU buffer control
    output reg                     buffer_swap,        
    
    // Post-processing trigger
    output reg                     post_proc_enable,   
    output reg                     post_proc_valid,    
    
    // Tile tracking
    output reg  [15:0]             current_tile_m,
    output reg  [15:0]             current_tile_n,
    output reg  [15:0]             current_tile_k
);

    localparam S_IDLE            = 3'd0;
    localparam S_LOAD_WEIGHTS    = 3'd1;
    localparam S_LOAD_BIAS       = 3'd2;
    localparam S_LOAD_ACT        = 3'd3;
    localparam S_COMPUTE         = 3'd4;
    localparam S_DRAIN           = 3'd5;

    reg [2:0] state, next_state;
    reg [15:0] cycle_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
        end else begin
            state <= next_state;
        end
    end

    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start) next_state = S_LOAD_WEIGHTS;
            end
            S_LOAD_WEIGHTS: begin
                if (cycle_cnt == ARRAY_SIZE - 1) next_state = S_LOAD_BIAS;
            end
            S_LOAD_BIAS: begin
                if (cycle_cnt == ARRAY_SIZE - 1) next_state = S_LOAD_ACT;
            end
            S_LOAD_ACT: begin
                if (cycle_cnt == (2 * ARRAY_SIZE - 2)) next_state = S_COMPUTE;
            end
            S_COMPUTE: begin
                if (cycle_cnt == ARRAY_SIZE - 1) next_state = S_DRAIN;
            end
            S_DRAIN: begin
                if (cycle_cnt == (2 * ARRAY_SIZE - 2)) begin
                    // Simplified: return to IDLE (in full implementation, handle M/N/K tile loops)
                    next_state = S_IDLE;
                end
            end
            default: next_state = S_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_cnt        <= 0;
            done             <= 0;
            busy             <= 0;
            weight_load_en   <= 0;
            weight_load_addr <= 0;
            weight_load_done <= 0;
            act_feed_en      <= 0;
            act_feed_addr    <= 0;
            compute_enable   <= 0;
            acc_clear        <= 0;
            drain_enable     <= 0;
            drain_done       <= 0;
            buffer_swap      <= 0;
            post_proc_enable <= 0;
            post_proc_valid  <= 0;
            current_tile_m   <= 0;
            current_tile_n   <= 0;
            current_tile_k   <= 0;
        end else begin
            // Defaults
            weight_load_en   <= 0;
            weight_load_done <= 0;
            act_feed_en      <= 0;
            compute_enable   <= 0;
            acc_clear        <= 0;
            drain_enable     <= 0;
            drain_done       <= 0;
            buffer_swap      <= 0;
            post_proc_enable <= 0;
            post_proc_valid  <= 0;
            done             <= 0;

            case (state)
                S_IDLE: begin
                    busy <= 0;
                    cycle_cnt <= 0;
                    if (start) begin
                        busy <= 1;
                        current_tile_m <= 0;
                        current_tile_n <= 0;
                        current_tile_k <= 0;
                        weight_load_addr <= ARRAY_SIZE - 1;
                    end
                end
                
                S_LOAD_WEIGHTS: begin
                    weight_load_en <= 1;
                    if (cycle_cnt < ARRAY_SIZE - 1) begin
                        weight_load_addr <= weight_load_addr - 1;
                        cycle_cnt <= cycle_cnt + 1;
                    end else begin
                        weight_load_done <= 1;
                        cycle_cnt <= 0;
                    end
                end

                S_LOAD_BIAS: begin
                    if (cycle_cnt < ARRAY_SIZE - 1) begin
                        cycle_cnt <= cycle_cnt + 1;
                    end else begin
                        cycle_cnt <= 0;
                        act_feed_addr <= 0;
                    end
                end
                
                S_LOAD_ACT: begin
                    act_feed_en    <= 1;
                    compute_enable <= 1;
                    if (cycle_cnt == 0) begin
                        acc_clear <= 1;
                    end
                    // Assert drain_enable when the first output row starts emerging (cycle ARRAY_SIZE)
                    if (cycle_cnt >= ARRAY_SIZE) begin
                        drain_enable     <= 1;
                        post_proc_enable <= 1;
                    end
                    if (cycle_cnt < (2 * ARRAY_SIZE - 2)) begin
                        act_feed_addr <= act_feed_addr + 1;
                        cycle_cnt <= cycle_cnt + 1;
                    end else begin
                        cycle_cnt <= 0;
                    end
                end

                S_COMPUTE: begin
                    compute_enable   <= 1;
                    drain_enable     <= 1;
                    post_proc_enable <= 1;
                    if (cycle_cnt < ARRAY_SIZE - 1) begin
                        cycle_cnt <= cycle_cnt + 1;
                    end else begin
                        cycle_cnt <= 0;
                    end
                end
                
                S_DRAIN: begin
                    // Keep post_proc_enable high to allow pipeline to flush
                    post_proc_enable <= 1;
                    if (cycle_cnt < (2 * ARRAY_SIZE - 2)) begin
                        cycle_cnt <= cycle_cnt + 1;
                    end else begin
                        drain_done <= 1;
                        cycle_cnt <= 0;
                        done <= 1;
                        busy <= 0;
                    end
                end
            endcase
        end
    end

endmodule
