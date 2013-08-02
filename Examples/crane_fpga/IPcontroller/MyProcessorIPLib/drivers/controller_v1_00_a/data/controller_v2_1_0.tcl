##############################################################################
## Filename:          C:\Users\asuardi\UK_Imperial\Research\Algorithms\Hardware_optimal_MPC\FPGA_implementatation\IPcontroller\MyProcessorIPLib/drivers/controller_v1_00_a/data/controller_v2_1_0.tcl
## Description:       Microprocess Driver Command (tcl)
## Date:              Mon Feb 13 16:04:08 2012 (by Create and Import Peripheral Wizard)
##############################################################################

#uses "xillib.tcl"

proc generate {drv_handle} {
  xdefine_include_file $drv_handle "xparameters.h" "controller" "NUM_INSTANCES" "DEVICE_ID" "C_BASEADDR" "C_HIGHADDR" 
}
