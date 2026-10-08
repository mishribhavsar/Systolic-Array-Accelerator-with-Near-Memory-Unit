`timescale 1ns / 1ps

// =============================================================================
// Testbench:  tb_systolic_nmu_wrapper
// DUT:        systolic_nmu_top_wrapper -> systolic_nmu_top (full NMU pipeline)
// Checks:     - end-to-end self-check against a behavioural reference:
//               out[m][j] = pool( sat( act( sum_k A[m][k]*W[k][j] + B[m][j] ) >>> qshift ) )
//             - PE weight registers after the FSM weight-load phase
//             - config decode (verifies serial frame packing + CFG_* map)
//             - write strobes are single-cycle, done is single-cycle,
//               exactly one start per run, no ghost restarts
//             - output_valid count and output_count at done
//             - logs cycles to console and to nmu_log.csv
//
// LOG CONVENTION (same as tb_systolic_array_core):
//   "cycle k" = state AFTER the k-th rising edge. Inputs change 1 ns after a
//   rising edge, sampling is 1 ns after the rising edge -> no races.
//   Serial-shift cycles are left out of the CSV unless LOG_SERIAL = 1.
//
// DATA LAYOUT (what the checker assumes)
//   weight SRAM word k : lane j = W[k][j]      (array row k)
//   act    SRAM word m : lane k = A[m][k]      (activation vector m)
//   act    SRAM words N..2N-1 = 0              (FSM reads addr 0..2N-2)
//   bias   SRAM word bias_base+m : lane j = B[m][j]
//   out    SRAM word m : lane j = out[m][j]
//
// ASSUMPTIONS ON MODULES NOT PROVIDED (edit if your RTL differs)
//   - config_regs map: CFG_* localparams below; start = write 1 to CFG_CTRL bit 0
//   - activation_unit: 0 = bypass, 1 = ReLU, 2 = leaky (x<0 : x >>> leaky_shift)
//   - pooling_unit   : POOL_MODE = 1 (default): 2x2 max-pool. Rows (2r, 2r+1)
//                      x lanes (2p, 2p+1) -> SRAM word r, lane p; N/2 writes.
//                      (matches output_valid on every 2nd row in the RTL log)
//                      POOL_MODE = 0: 2:1 max over adjacent lanes, N writes.
//   Set TEST_POOLING = 0 if the pooling semantics differ.
// =============================================================================

module tb_systolic_nmu_wrapper;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    parameter ARRAY_SIZE     = 8;
    parameter DATA_WIDTH     = 8;
    parameter ACC_WIDTH      = 32;
    parameter OUTPUT_WIDTH   = 16;
    parameter SRAM_DEPTH     = 256;
    parameter ADDR_WIDTH     = 8;
    parameter CFG_ADDR_WIDTH = 4;
    parameter NUM_RANDOM     = 10;   // random regression iterations
    parameter VERBOSE        = 1;    // 1 = frames + every RUN cycle, 2 = also PE weight grid
    parameter VERBOSE_RANDOM = 0;    // console verbosity during random loop
    parameter LOG_SERIAL     = 0;    // 1 = CSV also holds every serial shift cycle (large)
    parameter TEST_POOLING   = 1;
    parameter POOL_MODE      = 1;    // 1 = 2x2 max-pool, 0 = 2:1 lane pool
    parameter CHECK_CFG_READ = 1;    // check cfg_rdata readback of CFG_M
    parameter RUN_TIMEOUT    = 4000; // max cycles from start to done
    parameter CLK_PERIOD     = 10;

    localparam N  = ARRAY_SIZE;
    localparam DW = DATA_WIDTH;
    localparam AW = ACC_WIDTH;
    localparam OW = OUTPUT_WIDTH;
    localparam AD = ADDR_WIDTH;
    localparam CW = CFG_ADDR_WIDTH;

    localparam IN_BITS  = 1 + CW + 32 + CW + AD + N*DW + 1 + 1 + AD + N*AW + 1 + AD;
    localparam OUT_BITS = 32 + N*OW;

    // ---- config register map: EDIT to match config_regs.v ----
    localparam [CW-1:0] CFG_CTRL   = 4'd0,
                        CFG_M      = 4'd1,
                        CFG_K      = 4'd2,
                        CFG_N      = 4'd3,
                        CFG_ACT    = 4'd4,
                        CFG_POOL   = 4'd5,
                        CFG_QSHIFT = 4'd6,
                        CFG_LSHIFT = 4'd7,
                        CFG_BBASE  = 4'd8;
    localparam START_BIT = 0;

    // ---- controller_fsm state encoding ----
    localparam [2:0] S_IDLE = 3'd0, S_LOAD_WEIGHTS = 3'd1, S_LOAD_BIAS = 3'd2,
                     S_LOAD_ACT = 3'd3, S_COMPUTE = 3'd4, S_DRAIN = 3'd5;

    localparam signed [AW-1:0] QMAX =  (1 << (OW-1)) - 1;
    localparam signed [AW-1:0] QMIN = -(1 << (OW-1));

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg           clk;
    reg           rst_n;
    reg           serial_in;
    reg           serial_in_valid;
    reg           serial_out_load;
    wire          serial_out;
    wire          busy;
    wire          done;
    wire          output_valid;
    wire [AD-1:0] output_count;

    systolic_nmu_top_wrapper #(
        .ARRAY_SIZE    (ARRAY_SIZE),
        .DATA_WIDTH    (DATA_WIDTH),
        .ACC_WIDTH     (ACC_WIDTH),
        .OUTPUT_WIDTH  (OUTPUT_WIDTH),
        .SRAM_DEPTH    (SRAM_DEPTH),
        .ADDR_WIDTH    (ADDR_WIDTH),
        .CFG_ADDR_WIDTH(CFG_ADDR_WIDTH)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .serial_in      (serial_in),
        .serial_in_valid(serial_in_valid),
        .serial_out_load(serial_out_load),
        .serial_out     (serial_out),
        .busy           (busy),
        .done           (done),
        .output_valid   (output_valid),
        .output_count   (output_count)
    );

    // -------------------------------------------------------------------------
    // Clock
    // -------------------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // -------------------------------------------------------------------------
    // Debug taps (hierarchical)
    // -------------------------------------------------------------------------
    wire [2:0]  fsm_state  = dut.u_core.u_controller_fsm.state;
    wire        start_pulse= dut.u_core.start_pulse;
    wire        coll_valid = dut.u_core.collector_result_valid;
    wire [15:0] cfg_m      = dut.u_core.matrix_m_size;
    wire [15:0] cfg_k      = dut.u_core.matrix_k_size;
    wire [15:0] cfg_n      = dut.u_core.matrix_n_size;
    wire [1:0]  cfg_act    = dut.u_core.activation_mode;
    wire        cfg_pool   = dut.u_core.pooling_enable;
    wire [3:0]  cfg_qsh    = dut.u_core.quantize_shift;
    wire [3:0]  cfg_lsh    = dut.u_core.leaky_shift;
    wire [15:0] cfg_bbase  = dut.u_core.bias_base_addr;
    wire        stb_cfg_we = dut.core_cfg_we;
    wire        stb_wwe    = dut.core_ext_weight_we;
    wire        stb_awe    = dut.core_ext_act_we;
    wire        stb_bwe    = dut.core_bias_we;
    wire signed [N*AW-1:0] core_psum = dut.u_core.core_psum_out;

    wire signed [N*N*DW-1:0] pe_weight_flat;
    genvar gi, gj;
    generate
        for (gi = 0; gi < N; gi = gi + 1) begin : dbg_r
            for (gj = 0; gj < N; gj = gj + 1) begin : dbg_c
                assign pe_weight_flat[(gi*N+gj)*DW +: DW] =
                    dut.u_core.u_systolic_array_core.pe_row[gi].pe_col[gj].u_pe.weight_reg;
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Test data
    // -------------------------------------------------------------------------
    reg signed [DW-1:0] W_mat   [0:N-1][0:N-1];   // W[k][j]
    reg signed [DW-1:0] A_mat   [0:N-1][0:N-1];   // A[m][k]
    reg signed [AW-1:0] B_mat   [0:N-1][0:N-1];   // bias[m][j]
    reg signed [OW-1:0] Q_mat   [0:N-1][0:N-1];   // pre-pool reference
    reg signed [OW-1:0] EXP_mat [0:N-1][0:N-1];   // expected SRAM content
    reg signed [OW-1:0] OUT_mat [0:N-1][0:N-1];   // read back from DUT

    // Config shadow (what the next run programs)
    reg [1:0]    c_act;
    reg [3:0]    c_qsh;
    reg [3:0]    c_lsh;
    reg          c_pool;
    reg [AD-1:0] c_bbase;

    // Serial shadow of the sticky read addresses carried in every frame
    reg [CW-1:0] sh_cfg_raddr;
    reg [AD-1:0] sh_out_raddr;
    reg [OUT_BITS-1:0] rx_frame;

    // -------------------------------------------------------------------------
    // Bookkeeping
    // -------------------------------------------------------------------------
    integer       cycle_cnt, test_id;
    integer       err_cnt, chk_cnt, pass_tests, fail_tests, err_at_start;
    integer       seed, fd;
    integer       start_cnt, run_cycles;
    integer       exp_rows;        // output SRAM rows expected for this run
    reg           busy_seen;
    reg [8*3-1:0] phase;   // RST IDL SHF LAT WRD LOD SHO RUN
    reg           verbose;
    reg           prev_cfg_we, prev_wwe, prev_awe, prev_bwe, prev_done;

    // =========================================================================
    // Logging
    // =========================================================================
    function [8*4-1:0] st_name;
        input [2:0] s;
        case (s)
            S_IDLE:         st_name = "IDLE";
            S_LOAD_WEIGHTS: st_name = "LDW ";
            S_LOAD_BIAS:    st_name = "LDB ";
            S_LOAD_ACT:     st_name = "LDA ";
            S_COMPUTE:      st_name = "COMP";
            S_DRAIN:        st_name = "DRN ";
            default:        st_name = "????";
        endcase
    endfunction

    task write_csv_header;
        integer k;
        begin
            $fwrite(fd, "cycle,test,phase,rst_n,serial_in,sin_valid,sout_load,serial_out,");
            $fwrite(fd, "busy,done,output_valid,output_count,fsm_state,start_pulse,coll_valid");
            for (k = 0; k < N; k = k + 1) $fwrite(fd, ",psum_out%0d", k);
            $fdisplay(fd, "");
        end
    endtask

    task log_cycle;
        integer k;
        begin
            if (LOG_SERIAL || !((phase == "SHF") || (phase == "SHO"))) begin
                $fwrite(fd, "%0d,%0d,%s,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                        cycle_cnt, test_id, phase, rst_n, serial_in, serial_in_valid,
                        serial_out_load, serial_out, busy, done, output_valid,
                        output_count, fsm_state, start_pulse, coll_valid);
                for (k = 0; k < N; k = k + 1)
                    $fwrite(fd, ",%0d", $signed(core_psum[k*AW +: AW]));
                $fdisplay(fd, "");
            end
            if (verbose && (phase == "RUN")) begin
                $write("[cyc %7d] T%-3d RUN %s busy=%0d done=%0d st=%0d coll_v=%0d out_v=%0d cnt=%0d | psum_bot=[",
                       cycle_cnt, test_id, st_name(fsm_state), busy, done, start_pulse,
                       coll_valid, output_valid, output_count);
                for (k = 0; k < N; k = k + 1) $write(" %7d", $signed(core_psum[k*AW +: AW]));
                $display(" ]");
            end
        end
    endtask

    // Write strobes and done must be single-cycle pulses
    task check_pulses;
        begin
            if (rst_n) begin
                if ((stb_cfg_we && prev_cfg_we) || (stb_wwe && prev_wwe) ||
                    (stb_awe && prev_awe)       || (stb_bwe && prev_bwe)) begin
                    chk_cnt = chk_cnt + 1;
                    err_cnt = err_cnt + 1;
                    $display("ERROR [T%0d] cycle %0d: write strobe high > 1 cycle (cfg=%b w=%b a=%b b=%b)",
                             test_id, cycle_cnt, stb_cfg_we, stb_wwe, stb_awe, stb_bwe);
                end
                if (done && prev_done) begin
                    chk_cnt = chk_cnt + 1;
                    err_cnt = err_cnt + 1;
                    $display("ERROR [T%0d] cycle %0d: done high > 1 cycle", test_id, cycle_cnt);
                end
            end
            prev_cfg_we = stb_cfg_we;
            prev_wwe    = stb_wwe;
            prev_awe    = stb_awe;
            prev_bwe    = stb_bwe;
            prev_done   = done;
        end
    endtask

    task tick;
        begin
            @(posedge clk);
            #1;
            cycle_cnt = cycle_cnt + 1;
            if (start_pulse) start_cnt = start_cnt + 1;
            if (busy)        busy_seen = 1'b1;
            log_cycle;
            check_pulses;
        end
    endtask

    // =========================================================================
    // Basic control
    // =========================================================================
    task apply_reset;
        begin
            phase           = "RST";
            serial_in       = 0;
            serial_in_valid = 0;
            serial_out_load = 0;
            rst_n           = 0;
            tick;
            tick;
            rst_n           = 1;
            sh_cfg_raddr    = 0;   // wrapper latches reset to 0
            sh_out_raddr    = 0;
        end
    endtask

    task idle;
        input integer ncyc;
        integer n;
        begin
            phase           = "IDL";
            serial_in       = 0;
            serial_in_valid = 0;
            serial_out_load = 0;
            for (n = 0; n < ncyc; n = n + 1) tick;
        end
    endtask

    task test_start;
        input integer id;
        input integer do_rst;
        begin
            test_id      = id;
            err_at_start = err_cnt;
            if (do_rst != 0) apply_reset;
        end
    endtask

    task test_end;
        begin
            if (err_cnt == err_at_start) begin
                pass_tests = pass_tests + 1;
                $display("[TEST %0d] PASS", test_id);
            end else begin
                fail_tests = fail_tests + 1;
                $display("[TEST %0d] FAIL (%0d errors)", test_id, err_cnt - err_at_start);
            end
            $display("");
        end
    endtask

    task set_cfg;
        input [1:0]  act;
        input [3:0]  qsh;
        input [3:0]  lsh;
        input        pool;
        input [AD-1:0] bbase;
        begin
            c_act = act; c_qsh = qsh; c_lsh = lsh; c_pool = pool; c_bbase = bbase;
        end
    endtask

    // =========================================================================
    // Serial frame layer
    // =========================================================================
    function [IN_BITS-1:0] make_frame;
        input            f_cfg_we;
        input [CW-1:0]   f_cfg_addr;
        input [31:0]     f_cfg_wdata;
        input [AD-1:0]   f_ext_waddr;
        input [N*DW-1:0] f_ext_wdata;
        input            f_wwe;
        input            f_awe;
        input [AD-1:0]   f_bias_waddr;
        input [N*AW-1:0] f_bias_wdata;
        input            f_bwe;
        begin
            make_frame = {f_cfg_we, f_cfg_addr, f_cfg_wdata, sh_cfg_raddr,
                          f_ext_waddr, f_ext_wdata, f_wwe, f_awe,
                          f_bias_waddr, f_bias_wdata, f_bwe, sh_out_raddr};
        end
    endfunction

    // Shift IN_BITS bits MSB-first, then one serial_in_valid cycle
    task shift_frame;
        input [IN_BITS-1:0] f;
        integer b;
        begin
            phase = "SHF";
            for (b = IN_BITS-1; b >= 0; b = b - 1) begin
                serial_in = f[b];
                tick;
            end
            serial_in       = 1'b0;
            phase           = "LAT";
            serial_in_valid = 1'b1;
            tick;
            serial_in_valid = 1'b0;
        end
    endtask

    task cfg_write;
        input [CW-1:0] a;
        input [31:0]   d;
        begin
            if (verbose) $display("[cyc %7d] T%-3d FRAME cfg_write  addr=%0d data=0x%08h",
                                  cycle_cnt, test_id, a, d);
            shift_frame(make_frame(1'b1, a, d, {AD{1'b0}}, {(N*DW){1'b0}}, 1'b0, 1'b0,
                                   {AD{1'b0}}, {(N*AW){1'b0}}, 1'b0));
        end
    endtask

    task write_weight_row;
        input integer k;
        integer j;
        reg [N*DW-1:0] d;
        reg [AD-1:0]   a;
        begin
            a = k;
            for (j = 0; j < N; j = j + 1) d[j*DW +: DW] = W_mat[k][j];
            if (verbose) $display("[cyc %7d] T%-3d FRAME weight_wr  addr=%0d", cycle_cnt, test_id, a);
            shift_frame(make_frame(1'b0, {CW{1'b0}}, 32'd0, a, d, 1'b1, 1'b0,
                                   {AD{1'b0}}, {(N*AW){1'b0}}, 1'b0));
        end
    endtask

    // m < 0 -> write an all-zero padding word
    task write_act_row;
        input integer addr;
        input integer m;
        integer k;
        reg [N*DW-1:0] d;
        reg [AD-1:0]   a;
        begin
            a = addr;
            d = {(N*DW){1'b0}};
            if (m >= 0)
                for (k = 0; k < N; k = k + 1) d[k*DW +: DW] = A_mat[m][k];
            if (verbose) $display("[cyc %7d] T%-3d FRAME act_wr     addr=%0d %s",
                                  cycle_cnt, test_id, a, (m < 0) ? "(zero pad)" : "");
            shift_frame(make_frame(1'b0, {CW{1'b0}}, 32'd0, a, d, 1'b0, 1'b1,
                                   {AD{1'b0}}, {(N*AW){1'b0}}, 1'b0));
        end
    endtask

    task write_bias_raw;
        input integer    addr;
        input [N*AW-1:0] d;
        reg [AD-1:0] a;
        begin
            a = addr;
            if (verbose) $display("[cyc %7d] T%-3d FRAME bias_wr    addr=%0d", cycle_cnt, test_id, a);
            shift_frame(make_frame(1'b0, {CW{1'b0}}, 32'd0, {AD{1'b0}}, {(N*DW){1'b0}},
                                   1'b0, 1'b0, a, d, 1'b1));
        end
    endtask

    task write_bias_row;
        input integer addr;
        input integer m;
        integer j;
        reg [N*AW-1:0] d;
        begin
            for (j = 0; j < N; j = j + 1) d[j*AW +: AW] = B_mat[m][j];
            write_bias_raw(addr, d);
        end
    endtask

    // Set read addresses, wait read latency, load and shift out OUT_BITS bits
    task read_frame;
        input [AD-1:0] oaddr;
        input [CW-1:0] craddr;
        integer b;
        begin
            sh_out_raddr = oaddr;
            sh_cfg_raddr = craddr;
            shift_frame(make_frame(1'b0, {CW{1'b0}}, 32'd0, {AD{1'b0}}, {(N*DW){1'b0}},
                                   1'b0, 1'b0, {AD{1'b0}}, {(N*AW){1'b0}}, 1'b0));
            phase = "WRD";
            tick;
            tick;
            phase           = "LOD";
            serial_out_load = 1'b1;
            tick;
            serial_out_load = 1'b0;
            phase = "SHO";
            for (b = OUT_BITS-1; b >= 0; b = b - 1) begin
                rx_frame[b] = serial_out;
                tick;
            end
        end
    endtask

    // =========================================================================
    // Data generation
    // =========================================================================
    // mode: 0=const(val) 1=identity 2=random 3=counting(k*N+j+1) 4=sparse random
    task fill_W;
        input integer mode;
        input integer val;
        integer i, j, r;
        begin
            for (i = 0; i < N; i = i + 1)
                for (j = 0; j < N; j = j + 1) begin
                    case (mode)
                        0: W_mat[i][j] = val;
                        1: W_mat[i][j] = (i == j) ? 1 : 0;
                        2: W_mat[i][j] = $random(seed);
                        3: W_mat[i][j] = i*N + j + 1;
                        4: begin
                               r = $random(seed);
                               if (r[1:0] < 2) W_mat[i][j] = 0;
                               else            W_mat[i][j] = $random(seed);
                           end
                        default: W_mat[i][j] = 0;
                    endcase
                end
        end
    endtask

    // mode: 0=const(val) 1=random 2=counting(m*N+k+1) 3=sparse random 4=alternate zero vectors
    task fill_A;
        input integer mode;
        input integer val;
        integer mi, k, r;
        begin
            for (mi = 0; mi < N; mi = mi + 1)
                for (k = 0; k < N; k = k + 1) begin
                    case (mode)
                        0: A_mat[mi][k] = val;
                        1: A_mat[mi][k] = $random(seed);
                        2: A_mat[mi][k] = mi*N + k + 1;
                        3: begin
                               r = $random(seed);
                               if (r[1:0] < 2) A_mat[mi][k] = 0;
                               else            A_mat[mi][k] = $random(seed);
                           end
                        4: A_mat[mi][k] = (mi % 2) ? 0 : $random(seed);
                        default: A_mat[mi][k] = 0;
                    endcase
                end
        end
    endtask

    // mode: 0=const(val) 1=random in +/-65535 2=row-index pattern (m*1000 + j)
    task fill_B;
        input integer mode;
        input integer val;
        integer mi, j;
        begin
            for (mi = 0; mi < N; mi = mi + 1)
                for (j = 0; j < N; j = j + 1) begin
                    case (mode)
                        0: B_mat[mi][j] = val;
                        1: B_mat[mi][j] = $random(seed) % 65536;
                        2: B_mat[mi][j] = mi*1000 + j;
                        default: B_mat[mi][j] = 0;
                    endcase
                end
        end
    endtask

    // =========================================================================
    // Reference model: bias -> activation -> quantize(sat) -> pool
    // =========================================================================
    task compute_expected;
        integer mi, j, k;
        reg signed [AW-1:0] s, v;
        begin
            for (mi = 0; mi < N; mi = mi + 1)
                for (j = 0; j < N; j = j + 1) begin
                    s = 0;
                    for (k = 0; k < N; k = k + 1)
                        s = s + A_mat[mi][k] * W_mat[k][j];
                    s = s + B_mat[mi][j];
                    case (c_act)
                        2'd1:    v = (s < 0) ? 0 : s;
                        2'd2:    v = (s < 0) ? (s >>> c_lsh) : s;
                        default: v = s;
                    endcase
                    v = v >>> c_qsh;
                    if      (v > QMAX) Q_mat[mi][j] = QMAX[OW-1:0];
                    else if (v < QMIN) Q_mat[mi][j] = QMIN[OW-1:0];
                    else               Q_mat[mi][j] = v[OW-1:0];
                end
            exp_rows = (c_pool && POOL_MODE == 1) ? N/2 : N;
            for (mi = 0; mi < N; mi = mi + 1)
                for (j = 0; j < N; j = j + 1) begin
                    if (!c_pool) begin
                        EXP_mat[mi][j] = Q_mat[mi][j];
                    end else if (j >= N/2) begin
                        EXP_mat[mi][j] = 0;
                    end else if (POOL_MODE == 1) begin
                        if (mi < N/2)
                            EXP_mat[mi][j] = max4(Q_mat[2*mi][2*j],   Q_mat[2*mi][2*j+1],
                                                  Q_mat[2*mi+1][2*j], Q_mat[2*mi+1][2*j+1]);
                        else
                            EXP_mat[mi][j] = 0;
                    end else begin
                        EXP_mat[mi][j] = (Q_mat[mi][2*j] > Q_mat[mi][2*j+1]) ?
                                         Q_mat[mi][2*j] : Q_mat[mi][2*j+1];
                    end
                end
        end
    endtask

    function signed [OW-1:0] max4;
        input signed [OW-1:0] a, b, c, d;
        reg   signed [OW-1:0] x, y;
        begin
            x = (a > b) ? a : b;
            y = (c > d) ? c : d;
            max4 = (x > y) ? x : y;
        end
    endfunction

    // =========================================================================
    // Checks
    // =========================================================================
    task check_cfg_decode;
        begin
            chk_cnt = chk_cnt + 1;
            if ((cfg_m !== N) || (cfg_k !== N) || (cfg_n !== N) ||
                (cfg_act !== c_act) || (cfg_pool !== c_pool) ||
                (cfg_qsh !== c_qsh) || (cfg_lsh !== c_lsh) ||
                (cfg_bbase[AD-1:0] !== c_bbase)) begin
                err_cnt = err_cnt + 1;
                $display("ERROR [T%0d] cycle %0d: config decode mismatch", test_id, cycle_cnt);
                $display("    got M=%0d K=%0d N=%0d act=%0d pool=%0d qsh=%0d lsh=%0d bbase=%0d",
                         cfg_m, cfg_k, cfg_n, cfg_act, cfg_pool, cfg_qsh, cfg_lsh, cfg_bbase);
                $display("    exp M=%0d K=%0d N=%0d act=%0d pool=%0d qsh=%0d lsh=%0d bbase=%0d",
                         N, N, N, c_act, c_pool, c_qsh, c_lsh, c_bbase);
                $display("    -> check serial frame packing or the CFG_* map in this TB");
            end
        end
    endtask

    task check_weights;
        integer i, j, bad;
        reg signed [DW-1:0] got;
        begin
            bad = 0;
            for (i = 0; i < N; i = i + 1)
                for (j = 0; j < N; j = j + 1) begin
                    got = pe_weight_flat[(i*N+j)*DW +: DW];
                    chk_cnt = chk_cnt + 1;
                    if (got !== W_mat[i][j]) begin
                        err_cnt = err_cnt + 1;
                        bad = bad + 1;
                        if (bad <= 4)
                            $display("ERROR [T%0d] cycle %0d: PE[%0d][%0d].weight_reg got %0d expected %0d",
                                     test_id, cycle_cnt, i, j, got, W_mat[i][j]);
                    end
                end
            if (bad > 0 || VERBOSE >= 2) begin
                $display("    PE weight grid after load (%0d mismatches):", bad);
                for (i = 0; i < N; i = i + 1) begin
                    $write("      row %0d:", i);
                    for (j = 0; j < N; j = j + 1)
                        $write(" %4d", $signed(pe_weight_flat[(i*N+j)*DW +: DW]));
                    $display("");
                end
            end
        end
    endtask

    task check_idle_state;
        begin
            chk_cnt = chk_cnt + 1;
            if (busy !== 1'b0 || done !== 1'b0 || output_valid !== 1'b0 || fsm_state !== S_IDLE) begin
                err_cnt = err_cnt + 1;
                $display("ERROR [T%0d] cycle %0d: expected idle (busy=%b done=%b out_v=%b state=%0d)",
                         test_id, cycle_cnt, busy, done, output_valid, fsm_state);
            end
        end
    endtask

    task check_weights_zero;
        begin
            chk_cnt = chk_cnt + 1;
            if (pe_weight_flat !== {(N*N*DW){1'b0}}) begin
                err_cnt = err_cnt + 1;
                $display("ERROR [T%0d] cycle %0d: PE weights not cleared by reset", test_id, cycle_cnt);
            end
        end
    endtask

    // =========================================================================
    // Run phases
    // =========================================================================
    task program_tile;
        integer i;
        begin
            for (i = 0; i < N; i = i + 1) write_weight_row(i);
            for (i = 0; i < N; i = i + 1) write_act_row(i, i);
            for (i = N; (i < 2*N) && (i < SRAM_DEPTH); i = i + 1) write_act_row(i, -1);
            for (i = 0; i < N; i = i + 1) write_bias_row(c_bbase + i, i);
            cfg_write(CFG_M,      N);
            cfg_write(CFG_K,      N);
            cfg_write(CFG_N,      N);
            cfg_write(CFG_ACT,    c_act);
            cfg_write(CFG_POOL,   c_pool);
            cfg_write(CFG_QSHIFT, c_qsh);
            cfg_write(CFG_LSHIFT, c_lsh);
            cfg_write(CFG_BBASE,  c_bbase);
            idle(2);                       // let the last cfg write land
            check_cfg_decode;
        end
    endtask

    task start_and_wait;
        integer t, wchk, ov_cnt, got_cnt, seen_done;
        begin
            start_cnt = 0;
            cfg_write(CFG_CTRL, 32'd1 << START_BIT);
            phase     = "RUN";
            t = 0; wchk = 0; ov_cnt = 0; got_cnt = -1; seen_done = 0;
            while (!seen_done && (t < RUN_TIMEOUT)) begin
                tick;
                t = t + 1;
                if (!wchk && (fsm_state == S_LOAD_ACT)) begin
                    check_weights;
                    wchk = 1;
                end
                if (output_valid) ov_cnt = ov_cnt + 1;
                if (done) begin
                    seen_done = 1;
                    got_cnt   = output_count;
                end
            end
            run_cycles = t;

            chk_cnt = chk_cnt + 4;
            if (!seen_done) begin
                err_cnt = err_cnt + 1;
                $display("ERROR [T%0d] cycle %0d: done not seen within %0d cycles", test_id, cycle_cnt, RUN_TIMEOUT);
            end
            if (!wchk) begin
                err_cnt = err_cnt + 1;
                $display("ERROR [T%0d]: FSM never reached LOAD_ACT", test_id);
            end
            if (ov_cnt != exp_rows) begin
                err_cnt = err_cnt + 1;
                $display("ERROR [T%0d]: output_valid pulses = %0d, expected %0d", test_id, ov_cnt, exp_rows);
            end
            if (got_cnt != exp_rows) begin
                err_cnt = err_cnt + 1;
                $display("ERROR [T%0d]: output_count at done = %0d, expected %0d", test_id, got_cnt, exp_rows);
            end

            idle(4);
            check_idle_state;
            chk_cnt = chk_cnt + 1;
            if (start_cnt != 1) begin
                err_cnt = err_cnt + 1;
                $display("ERROR [T%0d]: start_pulse high for %0d cycles (expected a 1-cycle pulse) -> core restarts after done",
                         test_id, start_cnt);
            end
            if (verbose) $display("    run: %0d cycles start->done, %0d output rows", run_cycles, ov_cnt);
        end
    endtask

    task read_results;
        integer mi, j;
        reg [31:0] crd;
        begin
            for (mi = 0; mi < exp_rows; mi = mi + 1) begin
                read_frame(mi, CFG_M);
                crd = rx_frame[OUT_BITS-1 -: 32];
                if (CHECK_CFG_READ && (mi == 0)) begin
                    chk_cnt = chk_cnt + 1;
                    if (crd[15:0] !== N) begin
                        err_cnt = err_cnt + 1;
                        $display("ERROR [T%0d]: cfg_rdata(CFG_M) = 0x%08h, expected %0d", test_id, crd, N);
                    end
                end
                for (j = 0; j < N; j = j + 1) begin
                    OUT_mat[mi][j] = rx_frame[j*OW +: OW];
                    chk_cnt = chk_cnt + 1;
                    if (OUT_mat[mi][j] !== EXP_mat[mi][j]) begin
                        err_cnt = err_cnt + 1;
                        $display("ERROR [T%0d]: out[%0d][%0d] got %0d expected %0d",
                                 test_id, mi, j, OUT_mat[mi][j], EXP_mat[mi][j]);
                    end
                end
            end
        end
    endtask

    task print_matrices;
        integer i, j;
        begin
            if (verbose) begin
                $display("    cfg: act=%0d qshift=%0d lshift=%0d pool=%0d bias_base=%0d",
                         c_act, c_qsh, c_lsh, c_pool, c_bbase);
                $display("    Weights W[k][j]:");
                for (i = 0; i < N; i = i + 1) begin
                    $write("      ");
                    for (j = 0; j < N; j = j + 1) $write("%6d", W_mat[i][j]);
                    $display("");
                end
                $display("    Activations A[m][k]:");
                for (i = 0; i < N; i = i + 1) begin
                    $write("      ");
                    for (j = 0; j < N; j = j + 1) $write("%6d", A_mat[i][j]);
                    $display("");
                end
                $display("    Bias B[m][j]:");
                for (i = 0; i < N; i = i + 1) begin
                    $write("      ");
                    for (j = 0; j < N; j = j + 1) $write("%8d", B_mat[i][j]);
                    $display("");
                end
                $display("    Expected out[m][j]   |   DUT out[m][j]:");
                for (i = 0; i < exp_rows; i = i + 1) begin
                    $write("      ");
                    for (j = 0; j < N; j = j + 1) $write("%7d", EXP_mat[i][j]);
                    $write("   |");
                    for (j = 0; j < N; j = j + 1) $write("%7d", OUT_mat[i][j]);
                    $display("");
                end
            end
        end
    endtask

    // program SRAMs + config, start, wait, read back, check
    task run_tile;
        begin
            compute_expected;
            program_tile;
            start_and_wait;
            read_results;
            print_matrices;
            idle(2);
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    integer it, kk, bb;
    reg [N*AW-1:0] garbage;

    initial begin
        cycle_cnt = 0; test_id = 0; err_cnt = 0; chk_cnt = 0;
        pass_tests = 0; fail_tests = 0; err_at_start = 0;
        start_cnt = 0; run_cycles = 0; busy_seen = 0; exp_rows = N;
        prev_cfg_we = 0; prev_wwe = 0; prev_awe = 0; prev_bwe = 0; prev_done = 0;
        seed = 32'h1234_ABCD;
        verbose = VERBOSE;
        phase = "RST";
        rst_n = 0; serial_in = 0; serial_in_valid = 0; serial_out_load = 0;
        sh_cfg_raddr = 0; sh_out_raddr = 0;
        set_cfg(0, 0, 0, 0, 0);
        fill_B(0, 0);

        fd = $fopen("nmu_log.csv", "w");
        write_csv_header;

        $display("=====================================================");
        $display(" NMU wrapper TB : N=%0d DW=%0d AW=%0d OW=%0d", N, DW, AW, OW);
        $display(" input frame = %0d bits, output frame = %0d bits", IN_BITS, OUT_BITS);
        $display("=====================================================");

        // ---------------------------------------------------------------
        // T1: reset state
        // ---------------------------------------------------------------
        test_start(1, 1);
        $display("TEST 1: reset clears FSM, status pins, PE weights, serial_out");
        check_idle_state;
        check_weights_zero;
        chk_cnt = chk_cnt + 1;
        if (serial_out !== 1'b0 || output_count !== {AD{1'b0}}) begin
            err_cnt = err_cnt + 1;
            $display("ERROR [T1]: serial_out=%b output_count=%0d after reset", serial_out, output_count);
        end
        idle(3);
        check_idle_state;
        test_end;

        // ---------------------------------------------------------------
        // T2: serial interface, config decode, single-cycle strobes, no ghost start
        // ---------------------------------------------------------------
        test_start(2, 1);
        $display("TEST 2: serial config path, strobe width, no restart without start");
        set_cfg(2, 5, 3, 1, 17);
        start_cnt = 0; busy_seen = 0;
        cfg_write(CFG_M, N);   cfg_write(CFG_K, N);   cfg_write(CFG_N, N);
        cfg_write(CFG_ACT, c_act);    cfg_write(CFG_POOL, c_pool);
        cfg_write(CFG_QSHIFT, c_qsh); cfg_write(CFG_LSHIFT, c_lsh);
        cfg_write(CFG_BBASE, c_bbase);
        idle(2);
        check_cfg_decode;
        if (CHECK_CFG_READ) begin
            read_frame(0, CFG_K);
            chk_cnt = chk_cnt + 1;
            if (rx_frame[OUT_BITS-17 -: 16] !== N) begin
                err_cnt = err_cnt + 1;
                $display("ERROR [T2]: cfg_rdata(CFG_K) = 0x%08h, expected %0d",
                         rx_frame[OUT_BITS-1 -: 32], N);
            end
        end
        idle(2*IN_BITS);
        chk_cnt = chk_cnt + 1;
        if (start_cnt != 0 || busy_seen) begin
            err_cnt = err_cnt + 1;
            $display("ERROR [T2]: core started without a start write (start=%0d busy_seen=%b)",
                     start_cnt, busy_seen);
        end
        test_end;

        // ---------------------------------------------------------------
        // T3: identity weights, bypass post-processing -> out == A
        // ---------------------------------------------------------------
        test_start(3, 1);
        $display("TEST 3: identity weights, no bias/act/shift/pool, out must equal A");
        set_cfg(0, 0, 0, 0, 0);
        fill_W(1, 0); fill_A(1, 0); fill_B(0, 0);
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T4: all ones -> every output == N
        // ---------------------------------------------------------------
        test_start(4, 1);
        $display("TEST 4: all-ones weights and activations, every output = N");
        set_cfg(0, 0, 0, 0, 0);
        fill_W(0, 1); fill_A(0, 1); fill_B(0, 0);
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T5: counting pattern
        // ---------------------------------------------------------------
        test_start(5, 1);
        $display("TEST 5: counting-pattern weights and activations (qshift=2)");
        set_cfg(0, 2, 0, 0, 0);
        fill_W(3, 0); fill_A(2, 0); fill_B(0, 0);
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T6: random signed, with and without saturation
        // ---------------------------------------------------------------
        test_start(6, 1);
        $display("TEST 6a: random signed, qshift=0 (saturation active)");
        set_cfg(0, 0, 0, 0, 0);
        fill_W(2, 0); fill_A(1, 0); fill_B(0, 0);
        run_tile;
        $display("TEST 6b: random signed, qshift=4");
        set_cfg(0, 4, 0, 0, 0);
        fill_W(2, 0); fill_A(1, 0);
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T7: extreme values
        // ---------------------------------------------------------------
        test_start(7, 1);
        $display("TEST 7: extreme values (min*min, max*max, min*max) with/without saturation");
        fill_B(0, 0);
        set_cfg(0, 0, 0, 0, 0);
        fill_W(0, -(1 << (DW-1)));   fill_A(0, -(1 << (DW-1)));   run_tile;
        fill_W(0,  (1 << (DW-1))-1); fill_A(0,  (1 << (DW-1))-1); run_tile;
        fill_W(0, -(1 << (DW-1)));   fill_A(0,  (1 << (DW-1))-1); run_tile;
        set_cfg(0, 3, 0, 0, 0);
        fill_W(0, -(1 << (DW-1)));   fill_A(0, -(1 << (DW-1)));   run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T8: bias (row-distinct pattern exposes bias/row misalignment)
        // ---------------------------------------------------------------
        test_start(8, 1);
        $display("TEST 8a: identity W, bias = m*1000 + j (bias/row alignment)");
        set_cfg(0, 0, 0, 0, 0);
        fill_W(1, 0); fill_A(0, 0); fill_B(2, 0);
        run_tile;
        $display("TEST 8b: random data + random bias, qshift=4");
        set_cfg(0, 4, 0, 0, 0);
        fill_W(2, 0); fill_A(1, 0); fill_B(1, 0);
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T9: ReLU
        // ---------------------------------------------------------------
        test_start(9, 1);
        $display("TEST 9: ReLU (mode 1)");
        set_cfg(1, 3, 0, 0, 0);
        fill_W(2, 0); fill_A(1, 0); fill_B(1, 0);
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T10: leaky ReLU
        // ---------------------------------------------------------------
        test_start(10, 1);
        $display("TEST 10: leaky ReLU (mode 2), leaky_shift = 2 and 5");
        set_cfg(2, 3, 2, 0, 0);
        fill_W(2, 0); fill_A(1, 0); fill_B(1, 0);
        run_tile;
        set_cfg(2, 0, 5, 0, 0);
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T11: quantize shift sweep
        // ---------------------------------------------------------------
        test_start(11, 1);
        $display("TEST 11: quantize shift sweep 0,1,8,15");
        fill_W(2, 0); fill_A(1, 0); fill_B(1, 0);
        set_cfg(0, 0,  0, 0, 0); run_tile;
        set_cfg(0, 1,  0, 0, 0); run_tile;
        set_cfg(0, 8,  0, 0, 0); run_tile;
        set_cfg(0, 15, 0, 0, 0); run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T12: pooling
        // ---------------------------------------------------------------
        if (TEST_POOLING) begin
            test_start(12, 1);
            $display("TEST 12: pooling enabled (POOL_MODE=%0d)", POOL_MODE);
            set_cfg(1, 2, 0, 1, 0);
            fill_W(2, 0); fill_A(1, 0); fill_B(1, 0);
            run_tile;
            test_end;
        end

        // ---------------------------------------------------------------
        // T13: sparse / zero data
        // ---------------------------------------------------------------
        test_start(13, 1);
        $display("TEST 13a: sparse activations with dense weights");
        set_cfg(0, 2, 0, 0, 0);
        fill_W(2, 0); fill_A(3, 0); fill_B(0, 0);
        run_tile;
        $display("TEST 13b: sparse weights, alternate zero vectors");
        fill_W(4, 0); fill_A(4, 0);
        run_tile;
        $display("TEST 13c: all-zero activations, bias only");
        fill_W(2, 0); fill_A(0, 0); fill_B(1, 0);
        run_tile;
        $display("TEST 13d: all-zero weights");
        fill_W(0, 0); fill_A(1, 0); fill_B(0, 0);
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T14: back-to-back runs without reset (stale state, counter reset)
        // ---------------------------------------------------------------
        test_start(14, 1);
        $display("TEST 14: three runs back-to-back without reset");
        set_cfg(0, 3, 0, 0, 0); fill_W(2, 0); fill_A(1, 0); fill_B(1, 0); run_tile;
        set_cfg(1, 0, 0, 0, 0); fill_W(1, 0); fill_A(2, 0); fill_B(0, 0); run_tile;
        set_cfg(2, 4, 1, 0, 0); fill_W(3, 0); fill_A(1, 0); fill_B(1, 0); run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T15: non-zero bias_base with guard rows around the bias window
        // ---------------------------------------------------------------
        test_start(15, 1);
        bb = (SRAM_DEPTH > N + 40) ? 37 : 1;
        $display("TEST 15: bias_base = %0d, garbage in rows base-1 and base+N", bb);
        for (kk = 0; kk < N; kk = kk + 1) garbage[kk*AW +: AW] = 32'h5A5A_0000 + kk;
        write_bias_raw(bb - 1, garbage);
        if (bb + N < SRAM_DEPTH) write_bias_raw(bb + N, garbage);
        set_cfg(0, 2, 0, 0, bb);
        fill_W(2, 0); fill_A(1, 0); fill_B(2, 0);
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T16: reset during computation, then recovery
        // ---------------------------------------------------------------
        test_start(16, 1);
        $display("TEST 16: reset asserted in COMPUTE, then full recovery run");
        set_cfg(0, 0, 0, 0, 0);
        fill_W(3, 0); fill_A(2, 0); fill_B(0, 0);
        compute_expected;
        program_tile;
        cfg_write(CFG_CTRL, 32'd1 << START_BIT);
        phase = "RUN";
        kk = 0;
        while ((fsm_state != S_COMPUTE) && (kk < RUN_TIMEOUT)) begin tick; kk = kk + 1; end
        tick; tick;
        phase = "RST";
        rst_n = 0; tick; tick; rst_n = 1;
        sh_cfg_raddr = 0; sh_out_raddr = 0;
        check_idle_state;
        check_weights_zero;
        chk_cnt = chk_cnt + 1;
        if (output_count !== {AD{1'b0}}) begin
            err_cnt = err_cnt + 1;
            $display("ERROR [T16]: output_count=%0d after reset", output_count);
        end
        busy_seen = 0;
        idle(20);
        chk_cnt = chk_cnt + 1;
        if (busy_seen) begin
            err_cnt = err_cnt + 1;
            $display("ERROR [T16]: core restarted by itself after reset");
        end
        run_tile;
        test_end;

        // ---------------------------------------------------------------
        // T100+: random regression
        // ---------------------------------------------------------------
        $display("RANDOM REGRESSION: %0d iterations", NUM_RANDOM);
        verbose = VERBOSE_RANDOM;
        for (it = 0; it < NUM_RANDOM; it = it + 1) begin
            test_start(100 + it, it % 2);
            set_cfg({$random(seed)} % 3,
                    {$random(seed)} % 9,
                    {$random(seed)} % 5,
                    TEST_POOLING ? ({$random(seed)} % 2) : 0,
                    {$random(seed)} % (SRAM_DEPTH - N));
            fill_W(2 + ({$random(seed)} % 3), 0);
            fill_A(1 + ({$random(seed)} % 4), 0);
            fill_B({$random(seed)} % 3, 0);
            run_tile;
            test_end;
        end
        verbose = VERBOSE;

        // ---------------------------------------------------------------
        // Summary
        // ---------------------------------------------------------------
        $display("=====================================================");
        $display(" SUMMARY");
        $display("   Tests passed : %0d / %0d", pass_tests, pass_tests + fail_tests);
        $display("   Checks       : %0d", chk_cnt);
        $display("   Errors       : %0d", err_cnt);
        $display("   Total cycles : %0d", cycle_cnt);
        if (err_cnt == 0) $display("   *** ALL TESTS PASSED ***");
        else              $display("   *** TEST FAILED ***");
        $display("   Per-cycle log: nmu_log.csv");
        $display("=====================================================");
        $fclose(fd);
        $finish;
    end

    // Watchdog
    initial begin
        #(CLK_PERIOD * 20000000);
        $display("ERROR: simulation timeout");
        $fclose(fd);
        $finish;
    end

endmodule