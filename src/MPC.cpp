#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

/***********************************************************************
* Initialize variables
************************************************************************/

// Set the timestep length and duration.
const size_t N = 15;
const double dt = 0.12;

// Initialize error variables.
const double cte = 0;
const double epsi = 0;

// Initialize velocity variable.
const double v = 95;

// Initialize state and sctuator variables.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_psi_start = epsi_start + N;
const size_t a_start = delta_psi_start + N - 1;

// This value assumes the model presented in the classroom is used.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients.
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    
    /****************************************************************************************************
     * MPC Implementation
     * `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
     ****************************************************************************************************/
    
    // Initialize cost constraint.
    fg[0] = 0;
    
    // cte and epsi panalties.
    for (int i = 0; i < N; i++) {
      fg[0] += 2000 * CppAD::pow(vars[cte_start+ i] - cte, 2);
      fg[0] += 1500 * CppAD::pow(vars[epsi_start + i] - epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + i] - v, 2);
    }
    
    // large delta value and actuator penalties.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 2 * CppAD::pow(vars[delta_psi_start + i + 1] - vars[delta_psi_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
    
    // Build `vars` constraints.
    fg[x_start + 1] = vars[x_start];
    fg[y_start + 1] = vars[y_start];
    fg[psi_start + 1] = vars[psi_start];
    fg[v_start + 1] = vars[v_start];
    fg[cte_start + 1] = vars[cte_start];
    fg[epsi_start + 1] = vars[epsi_start];
    
    for (int i = 0; i < N - 1; i++) {
      AD<double> x_0 = vars[x_start+ 1];
      AD<double> x_1 = vars[x_start + i + 1];
      AD<double> y_0 = vars[y_start + 1];
      AD<double> y_1 = vars[y_start + i + 1];
      AD<double> psi_0 = vars[psi_start + 1];
      AD<double> psi_1 = vars[psi_start + i + 1];
      AD<double> v_0 = vars[v_start + 1];
      AD<double> v_1 = vars[v_start + i + 1];
      AD<double> delta_0 = vars[delta_psi_start + 1];
      AD<double> a_0 = vars[a_start + 1];
      AD<double> epsi_0 = vars[epsi_start + 1];
      AD<double> epsi_1 = vars[epsi_start + i + 1];
      AD<double> cte_1 = vars[cte_start + i + 1];
      AD<double> fx_0;
      AD<double> psides_0;
      
      // switch between polynomials.
      if (coeffs.size() == 4) {
        fx_0 = coeffs[0] + coeffs[1] * x_0 + coeffs[2] * CppAD::pow(x_0, 2) + coeffs[3] * CppAD::pow(x_0, 3);
        psides_0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x_0 + 3 * coeffs[3] * CppAD::pow(x_0, 2));
      } else {
        fx_0 = coeffs[0] + coeffs[1] * x_0 + coeffs[2] * CppAD::pow(x_0, 2);
        psides_0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x_0);
      }
      
      // compute derivatives to contrain the values to 0.
      fg[2 + x_start + i]    = x_1 - (x_0 + v_0 * CppAD::cos(psi_0) * dt);
      fg[2 + y_start + i]    = y_1 - (y_0 + v_0 * CppAD::sin(psi_0) * dt);
      fg[2 + psi_start + i]  = psi_1 - (psi_0 + v_0 / Lf * delta_0 * dt);
      fg[2 + v_start + i]    = v_1 - (v_0 + a_0 * dt);
      fg[2 + cte_start + i]  = cte_1 - ((fx_0 - y_0) + v_0 * CppAD::sin(epsi_0) * dt);
      fg[2 + epsi_start + i] = epsi_1 - ((psi_0 - psides_0) + v_0 * delta_0 / Lf * dt);
    }
  }
};

/***************************************************************************************
* MPC class definition implementation
****************************************************************************************/

MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  // Declare state variables.
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];
  
  // Model variables vector size.
  const int state_vec = 6;
  const int act_vec = 2;
  const size_t n_vars = N * state_vec + (N - 1) * act_vec;
  const size_t n_constraints = N * state_vec;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  // Initialize variables.
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // Set lower and upper limits for variables.
  for (int i = 0; i < delta_psi_start; i++) {
    vars_lowerbound[i] = -1 * numeric_limits<double>::max();
    vars_upperbound[i] = numeric_limits<double>::max();
  }
  
  // Steering constraints.
  const double steering = 25 * M_PI /180;
  for (int i = delta_psi_start; i < a_start; i++) {
    vars_lowerbound[i] = -steering;
    vars_upperbound[i] = steering;
  }
  
  // Acceleration constraints.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints.
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  // Initial state constraints for x.
  constraints_lowerbound[x_start] = x;
  constraints_upperbound[x_start] = x;
  
  // Initial state constraints for y;
  constraints_lowerbound[y_start] = y;
  constraints_upperbound[y_start] = y;
  
  // Initial state constraints for psi.
  constraints_lowerbound[psi_start] = psi;
  constraints_upperbound[psi_start] = psi;
  
  // Initial state constraints for v.
  constraints_lowerbound[v_start] = v;
  constraints_upperbound[v_start] = v;
  
  // Initial state constraints for epsi.
  constraints_lowerbound[epsi_start] = epsi;
  constraints_upperbound[epsi_start] = epsi;
  
  // Initial state constraints for cte.
  constraints_lowerbound[cte_start] = cte;
  constraints_upperbound[cte_start] = cte;

  // Object that computes objective and constraints.
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /***************************************************************************************
  * Return the first actuator values. The variables can be accessed with `solution.x[i]`.
  ****************************************************************************************/
  
  vector<double> result(N * 2 -14);
  result[0] += solution.x[delta_psi_start];
  result[1] += solution.x[a_start];
  int waypoints = N - 1 - 7;
  for (int i = 0; i < waypoints; i++) {
    result[i + 2] = solution.x[x_start + i];
    result[waypoints + i + 2] = solution.x[y_start + i];
  }
  return result;
}






























