Example of linking EMBOCON supervisor interface with MPT generated code

Martin Herceg, Automatic Control Laboratory, ETH Zurich 2012
herceg@control.ee.ethz.ch


Requirements for simulation

Matlab R2011a or later with compatible C-compiler.
MPT toolbox must be installed.


Simulation instructions

1) The script for synthesizing an explicit controller using MPT is given in
"crane_mpt_design.m". Run the script from Matlab to see the design procedure 
and first simulation outputs. The script generates the code that is stored
in the directory "mpt_explicit_controller".

2) Compile the generated code with the GEMS supervisor by running
"crane_mpt_compile.m".

3) Run the simulation script "crane_embocon_sim.mdl" in Simulink.

4) Plot the results using the script "plot_datasim_mpt.m".
