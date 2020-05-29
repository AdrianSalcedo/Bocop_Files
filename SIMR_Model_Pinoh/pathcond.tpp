// Function for the path constraints of the problem
// a <= g(t,y,u,z,p) <= b

// The following are the input and output available variables 
// for the path constraints of your optimal control problem.

// Input :
// dim_path_constraints : number of path constraints
// time : current time (t)
// initial_time : time value on the first discretization point
// final_time : time value on the last discretization point
// dim_* is the dimension of next vector in the declaration
// state : vector of state variables
// control : vector of control variables
// algebraicvars : vector of algebraic variables
// optimvars : vector of optimization vector of optimization parameters
// constants : vector of constants

// Output :
// path_constraints : vector of path constraints expressions ("g" in the example above)

// The functions of your problem have to be written in C++ code
// Remember that the vectors numbering in C++ starts from 0
// (ex: the first component of the vector state is state[0])

// Tdouble variables correspond to values that can change during optimization:
// states, controls, algebraic variables and optimization parameters.
// Values that remain constant during optimization use standard types (double, int, ...).

#include "header_pathcond"
{
	// HERE : description of the path constraints
	// Please give a function or a value for each path constraint
double b = constants[0];
double c = constants[1];
double a_1 = constants[2];
double a_2 = constants[3];
double eta = constants[4];
double g_1 = constants[5];
double g_2 = constants[6];
double A = constants[7];
double B = constants[8];
double C = constants[9];

Tdouble s = state[0];
Tdouble i = state[1];
Tdouble m = state[2];
Tdouble j = state[3];

	path_constraints[0] = -s;
	path_constraints[1] = -i;
	path_constraints[2] = -m;
	
}


