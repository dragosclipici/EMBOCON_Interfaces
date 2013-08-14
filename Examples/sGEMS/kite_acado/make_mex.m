%% kite_acado_make_mex.m

% Output file name, and also function name
fileOUT = 'SGems_SFunc';
% Root folder of code generation
CGRoot = './kite_nmpc_export';
% List of all source files to be compiled
CGSources = 'CGRoot/integrator.c CGRoot/condensing.c CGRoot/gauss_newton_method.c CGRoot/qpoases/SRC/QProblem.cpp CGRoot/qpoases/SRC/QProblemB.cpp CGRoot/qpoases/SRC/Bounds.cpp CGRoot/qpoases/SRC/Constraints.cpp CGRoot/qpoases/SRC/SubjectTo.cpp CGRoot/qpoases/SRC/Indexlist.cpp CGRoot/qpoases/SRC/CyclingManager.cpp CGRoot/qpoases/SRC/Utils.cpp CGRoot/qpoases/SRC/MessageHandling.cpp CGRoot/qpoases/solver.cpp';
% Gems related stuff
GemsInterfaceSources = 'SGems_SFunc.c SGems.c observer.c optimizer.c';
% Recipe for compilation
CGRecipe = 'mex -O -I. -I../../../supervisor/Interfaces/ -I''CGRoot'' -I''CGRoot/qpoases/INCLUDE'' -I''CGRoot/qpoases/SRC''  -D__MATLAB__ -O CGSources GemsInterfaceSources -output %s.%s';

% Compilation
CGSources = regexprep( CGSources, 'CGRoot', CGRoot);
CGRecipe = regexprep( CGRecipe, 'CGRoot', CGRoot);
CGRecipe = regexprep( CGRecipe, 'GemsInterfaceSources', GemsInterfaceSources);
eval( sprintf( regexprep( CGRecipe, 'CGSources', CGSources ), fileOUT, mexext ) )
