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
double b = constants[0];
double c = constants[1];
double a = constants[2];
double e = constants[3];
double g = constants[4];
double B = constants[5];
double d = constants[6];

Tdouble S = state[0];
Tdouble E = state[1];
Tdouble I = state[2];
Tdouble N = state[3];
Tdouble R = state[4];
Tdouble W = state[5];
Tdouble J = state[6];

Tdouble u = control[0];

	state_dynamics[0] = b-b*S-(c-a)*S*I-u*S;
	state_dynamics[1] = c*S*I-(e+b)*S+a*E*I;
	state_dynamics[2] = e*E-(g+a+b)*I-a*I*I;
	state_dynamics[3] = 0;
	state_dynamics[4] = g*I+u*S-b*R+a*I*R;
	state_dynamics[5] = u*S-(b-d)*W+a*I*W;
	state_dynamics[6] = I +B*u/N-(b-d)*J+a*I*J;
}


