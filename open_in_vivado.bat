@echo off
echo ====================================================================
echo  Creating and Opening Vivado Project for SystolicArray_NMU
echo ====================================================================

where vivado >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [WARNING] Vivado executable not found directly in current PATH.
    echo Running Vivado Tcl script if available or opening GUI...
)

vivado -mode batch -source vivado_create_project.tcl
if exist vivado_project\SystolicArray_NMU.xpr (
    echo Launching Vivado GUI...
    start vivado vivado_project\SystolicArray_NMU.xpr
)
