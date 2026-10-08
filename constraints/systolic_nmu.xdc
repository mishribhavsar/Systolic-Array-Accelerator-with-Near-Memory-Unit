## ==============================================================================
## Timing & Pin Constraints for Systolic Array with NMU
## Target FPGA: Zynq-7000 XC7Z020 (xc7z020clg400-1)
## Top module : systolic_nmu_top_wrapper (pin locations not assigned)
## ==============================================================================

## Primary Clock Definition (100 MHz -> 10 ns period)
create_clock -period 10.000 -name clk -waveform {0.000 5.000} [get_ports clk]

## Input Delay Constraints
set_input_delay -clock [get_clocks clk] -max 2.500 [all_inputs]
set_input_delay -clock [get_clocks clk] -min 1.000 [all_inputs]

## Output Delay Constraints
set_output_delay -clock [get_clocks clk] -max 2.500 [all_outputs]
set_output_delay -clock [get_clocks clk] -min 1.000 [all_outputs]
