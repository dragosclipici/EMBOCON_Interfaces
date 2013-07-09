EMBOCON_Interfaces
==================

The set of EMBOCON standard interfaces

The basis for the interaction between the different modules of the online part of the software platform
is dened by a set of standardized interfaces. These interfaces specify sets of callable functions for the
dierent modules to trigger computation routines in the corresponding modules. The set of specified
functions of an interface includes standard functions which have to be implemented by the module and
optional functions which are used in special problem classes or for debugging purpose. The input and
output parameters of the interface functions dene the data content and the data structures which
are exchanged when calling a function.

The advantages of using common interfaces for the communication between the dierent modules
of the online part of the platform are flexibility and applicability. Flexibility is offered by the easy
substitution of any module with another module implementing the same interface, e.g. the simulation
environment can be exchanged by an application after running necessary test cases, or the optimizer
can be replaced by another optimizer for benchmarking performance using the same application.
Applicability is oered by the set of common interfaces which, allows on the one hand, industrial
users to implement a needed interface in a simulation environment and/or an application to evaluate
the performance of dierent state-of-the-art numerical algorithms supported by the platform, and on
the other hand allows algorithm developers to test their developed numerical algorithms on a set of
different problems in form of models, simulations, or applications from dierent target sectors.

The structure and the set of interfaces of the software platform contains the following interfaces: 
The Model Interface, the Optimizer Interface, the Observer Interface, and the SimulationApplication Interface. 
The Optimizer Interface and the Observer Interface are common for all optimization algorithms used by the 
platform, independent whether the development platform is used to implement the problem, or C code generated
by the Matlab by-pass is used. Both are used to connect the elements of the optimization routine to the
supervisor. The Model Interface is used for the interaction between the elements of the optimization
algorithm and C code models generated by the Modelica-Optimica-based oine part of the platform. 
The SimulationApplication Interface is used for the interaction between a simulation environment or 
an application and the supervisor. It contains two layers, one common information layer which defines 
what should be exchanged via the interface and a tailored implementation layer which includes the 
actual implementation of the connection to an application, or a simulation environment.

The implementation language of all interfaces is C. The interfaces specied in this report consist of a
number of function definitions and possibly type definitions.

The reader should note that C passes all function arguments that are arrays by reference. 
Therefore, calling a function does not in itself incur a copy of any argument that is an
array. Any other arguments of interface functions but arrays are also passed by reference. The actual
implementation of the function may or may not involve copying the contents of the array or any other
argument.

C allows the last dimension of a multidimensional array to be unspecied if the array is a function
parameter, but the leading dimensions must have a constant size that is known during compilation.
This does not t the need for parameters that represent a matrix of arbitrary dimensions. Where
a matrix is to be passed to or from a function, a one-dimensional array will be passed. The matrix
elements are stored in the array in row major order, which is the standard C ordering: rst come all
the elements of the rst row, then comes the second row, and so on.

When an array is passed into a function as an input argument, the function may not assume that the
memory of the array remains available beyond the function call.

The source code of the interfaces and empty library implementations are released as open-source at the
end of the project under the MIT license using the GitHub repository system. The respository location
of the interfaces is the following: https://github.com/EMBOCONcs/EMBOCON Interfaces.git. All
libraries are contained in single projects for the Eclipse IDE.
