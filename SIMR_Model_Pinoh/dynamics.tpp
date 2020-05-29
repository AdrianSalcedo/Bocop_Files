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

Tdouble u = control[0];
Tdouble v = control[1];

	state_dynamics[0] = b-b*s-c*s*i+(a_1*i+a_2*m)*s-eta*u*s;
	state_dynamics[1] = c*s*i-b*i-(g_1+a_1)*i+(a_1*i+a_2*m)*i-i*v;
	state_dynamics[2] = -(a_2+g_2+b)*m+(a_1*i+a_2*m)*m+i*v;
	state_dynamics[3] = A*i+B*u+C*v;
}


