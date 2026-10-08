# Systolic Array Accelerator with Near-Memory Unit (NMU)

An 8×8 weight-stationary systolic array in Verilog RTL for CNN inference, with a
Near-Memory Unit that applies bias, activation, quantisation and pooling to the
array output on its way into on-chip memory. This avoids a separate pass over
memory for each post-processing step.

- **Language / tools:** Verilog-2001, Xilinx Vivado, Icarus Verilog
- **Target device:** Zynq-7000 XC7Z020 (`xc7z020clg400-1`, PYNQ-Z2)
- **Verification:** self-checking testbench with a golden reference model.
  26 / 26 tests and 4,892 output checks pass, with 0 errors.

---

## Architecture

```
            serial_in / serial_out (17 pins)
                         │
            ┌────────────▼─────────────┐
            │ systolic_nmu_top_wrapper │  serial-shift I/O wrapper
            └────────────┬─────────────┘
                         │
┌────────────────────────▼─────────────────────────────────────┐
│                      systolic_nmu_top                        │
│                                                              │
│  config_regs ──▶ controller_fsm (6 states)                   │
│                        │ addresses / enables                 │
│  ┌─────────────────────▼────────────────────────┐            │
│  │ NMU-In: Weight BRAM, Activation BRAM,        │            │
│  │         zero detection                       │            │
│  └───────────┬───────────────────┬──────────────┘            │
│              │ weights           │ activations               │
│              │            ┌──────▼──────┐                    │
│              │            │ input skew  │                    │
│  ┌───────────▼────────────▼─────────────┐   Bias BRAM        │
│  │ systolic_array_core: 8×8 PEs         │       │            │
│  │ INT8 × INT8 → 32-bit MAC, zero-skip  │       │            │
│  └──────────────────┬───────────────────┘       │            │
│              output_collector (de-skew)         │            │
│  ┌──────────────────▼───────────────────────────▼─────────┐  │
│  │ NMU-Out: Bias ▶ ReLU/LeakyReLU ▶ Quantise ▶ 2×2 Pool   │  │
│  │          ▶ Output BRAM                                 │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

### Dataflow

1. **Load weights** (8 cycles). The FSM reads the weight BRAM one row at a time
   and shifts each row down into the PE grid. Each PE then holds its weight.
2. **Load bias** (8 cycles). The bias read pointer is set to `BIAS_BASE_ADDR`.
3. **Stream activations** (15 cycles). Activation vectors are read from BRAM and
   skewed diagonally (row *i* is delayed by *i* cycles). They move left to
   right, and partial sums move top to bottom.
4. **Compute / drain** (8 + 15 cycles). The output collector de-skews the
   anti-diagonal results, and each row goes through the NMU-Out pipeline into
   the output BRAM.

One 8×8 × 8×8 tile (512 MACs) takes **56 clock cycles** from start to done.

### Near-memory post-processing pipeline

```
psum (32b) → +bias (32b, saturating) → ReLU / Leaky-ReLU (32b) → >>> shift, saturate (16b) → 2×2 max-pool (16b) → Output BRAM
```

| Stage | Detail |
|---|---|
| Bias add | Per-element 32-bit add with saturation; one bias row per output row |
| Activation | 0 = bypass, 1 = ReLU, 2 = Leaky ReLU (negative slope = 2^-`LEAKY_SHIFT`) |
| Quantiser | Arithmetic right-shift by `QUANTIZE_SHIFT` (0–15), saturating clip 32 → 16 bit |
| Pooling | Optional 2×2 max-pool, stride 2: 8×8 → 4×4 (4 output rows) |

Each stage is a 1-cycle registered pipeline stage.

---

## On-chip memory

Each memory is written as a Verilog array with `(* ram_style = "block" *)`. Vivado
maps it to Block RAM.

| Memory | Location | Organisation | Contents |
|---|---|---|---|
| Weight BRAM | `near_memory_input.v` | 256 × 64 bit | word *k* = 8 INT8 weights of row *k* |
| Activation BRAM | `near_memory_input.v` | 256 × 64 bit | word *m* = 8 INT8 activations |
| Bias BRAM | `systolic_nmu_top.v` | 256 × 256 bit | word *m* = 8 × 32-bit biases |
| Output BRAM | `near_memory_output.v` | 256 × 128 bit | word *m* = 8 × 16-bit results |

All reads are synchronous, with 1-cycle latency.

---

## Module hierarchy

| Module | File | Description |
|---|---|---|
| `systolic_nmu_top_wrapper` | `rtl/systolic_nmu_top_wrapper.v` | Serial-shift I/O wrapper (~560 core signals → 17 pins) |
| `systolic_nmu_top` | `rtl/systolic_nmu_top.v` | Top-level integration, input skew, bias BRAM |
| `config_regs` | `rtl/config_regs.v` | Memory-mapped configuration registers |
| `controller_fsm` | `rtl/controller_fsm.v` | 6-state control FSM, BRAM address generation |
| `near_memory_input` | `rtl/near_memory_input.v` | Weight + activation BRAM, zero detection |
| `sram_bank` | `rtl/sram_bank.v` | Parameterised simple-dual-port BRAM |
| `systolic_array_core` | `rtl/systolic_array_core.v` | N×N PE grid |
| `pe` | `rtl/pe.v` | Weight-stationary MAC with zero-skip |
| `output_collector` | `rtl/output_collector.v` | Anti-diagonal de-skew |
| `near_memory_output` | `rtl/near_memory_output.v` | Post-processing pipeline + output BRAM |
| `bias_adder` | `rtl/bias_adder.v` | Saturating bias addition |
| `activation_unit` | `rtl/activation_unit.v` | ReLU / Leaky ReLU |
| `quantizer` | `rtl/quantizer.v` | Shift + saturating 32 → 16-bit quantisation |
| `pooling_unit` | `rtl/pooling_unit.v` | 2×2 max-pooling, stride 2 |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `ARRAY_SIZE` | 8 | N×N array dimension |
| `DATA_WIDTH` | 8 | Signed operand width |
| `ACC_WIDTH` | 32 | Accumulator width |
| `OUTPUT_WIDTH` | 16 | Quantised output width |
| `SRAM_DEPTH` | 256 | Words per BRAM |
| `ADDR_WIDTH` | 8 | BRAM address width |

## Configuration registers

| Addr | Name | Bits | Function |
|---|---|---|---|
| 0 | `START` | [0] | Write 1 to start a run (single-cycle pulse) |
| 1–3 | `MATRIX_M/K/N_SIZE` | [15:0] | Stored and readable; each run currently computes one 8×8 tile |
| 4 | `ACTIVATION_MODE` | [1:0] | 0 = bypass, 1 = ReLU, 2 = Leaky ReLU |
| 5 | `POOLING_ENABLE` | [0] | Enable 2×2 max-pooling |
| 6 | `QUANTIZE_SHIFT` | [3:0] | Right-shift before 16-bit saturation |
| 7 | `LEAKY_SHIFT` | [3:0] | Leaky-ReLU slope = 2^-shift |
| 8 | `BIAS_BASE_ADDR` | [15:0] | First bias BRAM row for this run |

## FSM

```
IDLE → LOAD_WEIGHTS → LOAD_BIAS → LOAD_ACT → COMPUTE → DRAIN → IDLE
```

---

## Verification

`tb/tb_systolic_nmu_wrapper.v` drives the design through the serial wrapper.
It checks every output against a behavioural reference model:

```
out[m][j] = pool( sat16( act( Σk A[m][k]·W[k][j] + B[m][j] ) >>> qshift ) )
```

It also checks PE weight registers, config decoding, single-cycle strobes,
`done` / `output_valid` counts and that the design never restarts on its own.

| Test | Covers |
|---|---|
| 1–2 | Reset state, serial config path, strobe width |
| 3–5 | Identity, all-ones and counting-pattern matrices |
| 6–7 | Random signed data, extreme values, saturation |
| 8 | Bias/row alignment, random bias |
| 9–10 | ReLU, Leaky ReLU (two slopes) |
| 11 | Quantisation shift sweep (0, 1, 8, 15) |
| 12 | 2×2 max-pooling |
| 13 | Sparse / all-zero activations and weights (zero-skip) |
| 14 | Three back-to-back runs without reset |
| 15 | Non-zero bias base address with guard rows |
| 16 | Reset asserted mid-compute, then recovery |
| 100–109 | Randomised regression (random activation, shifts, pooling, bias base) |

**Result:** 26 / 26 tests passed, 4,892 checks, 0 errors.

### Run with Icarus Verilog

```bash
iverilog -g2012 -o sim.vvp rtl/*.v tb/tb_systolic_nmu_wrapper.v
vvp sim.vvp
```

### Run with Vivado

```bash
vivado -mode batch -source vivado_create_project.tcl   # creates vivado_project/
```

Or on Windows, run `open_in_vivado.bat`. The script targets `xc7z020clg400-1`
and sets `systolic_nmu_top_wrapper` as the top-level design and
`tb_systolic_nmu_wrapper` as the simulation top.

---

## Project structure

```
├── rtl/                        14 Verilog modules
├── tb/tb_systolic_nmu_wrapper.v
├── constraints/systolic_nmu.xdc   100 MHz clock + I/O delay constraints
├── vivado_create_project.tcl
└── open_in_vivado.bat
```
