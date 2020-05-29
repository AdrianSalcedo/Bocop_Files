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
Tdouble S = state[0];
Tdouble I_a = state[1];
Tdouble I_s = state[2];
Tdouble R_a = state[3];
Tdouble R_s = state[4];
Tdouble D = state[5];
Tdouble V = state[6];
Tdouble x_aux = state[7];

	path_constraints[0] = S+I_a+I_s+R_a+R_s+V+D-1;
	path_constraints[1] = -S;
	path_constraints[2] = -I_a;
	path_constraints[3] = -I_s;//-0.00027802233;
	path_constraints[4] = -R_a;
	path_constraints[5] = -R_s;
	path_constraints[6] = -D;
	path_constraints[7] = -V;
	path_constraints[8] = -x_aux;
}


