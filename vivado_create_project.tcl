# ==============================================================================
# Vivado Automated Project Creation Script
# Project: Systolic Array with Near-Memory Unit (NMU)
# ==============================================================================

# Define Project Parameters
set proj_name "SystolicArray_NMU"
set target_part "xc7z020clg400-1" ;# Zynq-7000 XC7Z020 (PYNQ-Z2)
set script_dir [file normalize [file dirname [info script]]]
set proj_dir "$script_dir/vivado_project"

puts "----------------------------------------------------------------------"
puts "  SystolicArray_NMU Vivado Project Setup"
puts "  Target Part: $target_part"
puts "  Root Path  : $script_dir"
puts "  Project Dir: $proj_dir"
puts "----------------------------------------------------------------------"

# 1. Create Vivado Project (Overwrites existing project if any)
create_project $proj_name $proj_dir -part $target_part -force

# 2. Set Project Properties
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]
set_property default_lib xil_defaultlib [current_project]

# 3. Add RTL Sources (Design Files)
puts "--> Adding RTL sources from /rtl..."
set rtl_files [glob -nocomplain "$script_dir/rtl/*.v"]
if {[llength $rtl_files] > 0} {
    add_files -fileset sources_1 $rtl_files
    puts "    Added [llength $rtl_files] RTL files."
} else {
    puts "    WARNING: No .v files found in $script_dir/rtl"
}

# Set Top Module for RTL
set_property top systolic_nmu_top_wrapper [get_filesets sources_1]

# 4. Add Testbench Sources (Simulation Files)
puts "--> Adding Testbench sources from /tb..."
set tb_files [glob -nocomplain "$script_dir/tb/*.v"]
if {[llength $tb_files] > 0} {
    add_files -fileset sim_1 $tb_files
    puts "    Added [llength $tb_files] Simulation files."
} else {
    puts "    WARNING: No .v files found in $script_dir/tb"
}

# Set Top Module for Simulation
set_property top tb_systolic_nmu_wrapper [get_filesets sim_1]

# Configure Simulation Properties
set_property xsim.simulate.runtime {ALL} [get_filesets sim_1]
set_property xsim.simulate.log_all_signals true [get_filesets sim_1]

# 5. Add Constraint Files
puts "--> Adding Constraint sources from /constraints..."
set constr_files [glob -nocomplain "$script_dir/constraints/*.xdc"]
if {[llength $constr_files] > 0} {
    add_files -fileset constrs_1 $constr_files
    puts "    Added [llength $constr_files] Constraint files."
} else {
    puts "    No .xdc constraint files found."
}

# 6. Update Hierarchy and Compile Order
puts "--> Updating Compile Order..."
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

puts "----------------------------------------------------------------------"
puts "  SUCCESS: Vivado project created successfully!"
puts "  Project File: $proj_dir/$proj_name.xpr"
puts "----------------------------------------------------------------------"
