// Function for the dynamics of the problem
// dy/dt = dynamics(y,u,z,p)

// The following are the input and output available variables 
// for the dynamics of your optimal control problem.

// Input :
// time : current time (t)
// normalized_time: t renormalized in [0,1]
// initial_time : time value on the first discretization point
// final_time : time value on the last discretization point
// dim_* is the dimension of next vector in the declaration
// state : vector of state variables
// control : vector of control variables
// algebraicvars : vector of algebraic variables
// optimvars : vector of optimization parameters
// constants : vector of constants

// Output :
// state_dynamics : vector giving the expression of the dynamic of each state variable.

// The functions of your problem have to be written in C++ code
// Remember that the vectors numbering in C++ starts from 0
// (ex: the first component of the vector state is state[0])

// Tdouble variables correspond to values that can change during optimization:
// states, controls, algebraic variables and optimization parameters.
// Values that remain constant during optimization use standard types (double, int, ...).

#include "header_dynamics"
{
	// HERE : description of the function for the dynamics
	// Please give a function or a value for the dynamics of each state variable
	// dynamics for the lenhart example
	// dx1/dt = x2
	// dx2/dt = -r*u
	// dx3/dt = 0.5*(x1*x1 +u*u)

	// constants
double r = constants[0];
double s = constants[1];
//double theta = constants[2];

	// variables
Tdouble x1 = state[0];
Tdouble x2 = state[1];
Tdouble x3 = state[2];

Tdouble u = control[0];

	// dynamics
state_dynamics[0] = x2;
state_dynamics[1] = -r*u;
state_dynamics[2] = 0.5*(x1*x1 +u*u);
}


