
if exist('mpt_explicit_controller','dir')~=7
    error('The directory "mpt_explicit_controller" must be present in order to compile the S-function.');
end
mex -largeArrayDims -IEMBOCON_Interfaces -Impt_explicit_controller SGems_SFunc.c SGems.c observer.c optimizer.c