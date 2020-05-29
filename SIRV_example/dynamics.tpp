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
double beta = constants[0];
double gamma = constants[1];
double d = constants[2];
double a = constants[3];
double B = constants[4];

Tdouble s = state[0];
Tdouble i = state[1];
Tdouble r = state[2];
Tdouble v = state[3];
Tdouble x_aux = state[4];

Tdouble u = control[0];

state_dynamics[0] = -beta*s*i-u;
state_dynamics[1] = beta*s*i-gamma*i;
state_dynamics[2] = gamma*i;
state_dynamics[3] = u;
state_dynamics[4] = i+d*u;
}


