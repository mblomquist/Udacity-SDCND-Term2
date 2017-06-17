#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
// Set number of timesteps (N) and length of time per timestep (dt)
size_t N = 8;
double dt = 0.17;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Set error references values.
double ref_cte = 0.0;  // Cross-track error (CTE)
double ref_epsi = 0.0; //  Orientation Error
double ref_vel = 16.0; // Reference velocity

// Define Cost coefficients (Set to 1.0)
const double c_cte = 0.19; // weights to CTE
const double c_epsi = 17.0;
const double c_vel = 0.261;
const double c_throttle = 0.7;
const double c_steering = 5.0; // higher is less turning
const double c_t_seq = 10.0;
const double c_s_seq = 1200.0; // Note: Reviewer noted this as the most important value to mod.

// Create an index scheme for the optimization solver as the input
// is a single vector.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
      // Fitted polynomial coefficients
      Eigen::VectorXd coeffs;
      FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

      typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
      void operator()(ADvector& fg, const ADvector& vars) {
            // TODO: implement MPC
            // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
            // NOTE: You'll probably go back and forth between this function and
            // the Solver function below.

            // From classroom...
            fg[0] = 0;

            // The part of the cost based on the reference state.
            for (int t = 0; t < N; t++) {
                  fg[0] += c_cte*CppAD::pow(vars[cte_start + t] - ref_cte, 2);
                  fg[0] += c_epsi*CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
                  fg[0] += c_vel*CppAD::pow(vars[v_start + t] - ref_vel, 2);
            }

            // Minimize the use of actuators.
            for (int t = 0; t < N - 1; t++) {
                  fg[0] += c_steering*CppAD::pow(vars[delta_start + t], 2);
                  fg[0] += c_throttle*CppAD::pow(vars[a_start + t], 2);
            }

            // Minimize the value gap between sequential actuations.
            for (int t = 0; t < N - 2; t++) {
                  fg[0] += c_s_seq*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
                  fg[0] += c_t_seq*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
            }
            //
            // Setup Constraints
            //
            // NOTE: In this section you'll setup the model constraints.

            // Initial constraints
            //
            // We add 1 to each of the starting indices due to cost being located at
            // index 0 of `fg`.
            // This bumps up the position of all the other values.
            fg[1 + x_start] = vars[x_start];
            fg[1 + y_start] = vars[y_start];
            fg[1 + psi_start] = vars[psi_start];
            fg[1 + v_start] = vars[v_start];
            fg[1 + cte_start] = vars[cte_start];
            fg[1 + epsi_start] = vars[epsi_start];

            // The rest of the constraints
            for (int i = 0; i < N - 1; i++) {
                  // The state at time t+1 .
                  AD<double> x1 = vars[x_start + i + 1];
                  AD<double> y1 = vars[y_start + i + 1];
                  AD<double> psi1 = vars[psi_start + i + 1];
                  AD<double> v1 = vars[v_start + i + 1];
                  AD<double> cte1 = vars[cte_start + i + 1];
                  AD<double> epsi1 = vars[epsi_start + i + 1];

                  // The state at time t.
                  AD<double> x0 = vars[x_start + i];
                  AD<double> y0 = vars[y_start + i];
                  AD<double> psi0 = vars[psi_start + i];
                  AD<double> v0 = vars[v_start + i];
                  AD<double> cte0 = vars[cte_start + i];
                  AD<double> epsi0 = vars[epsi_start + i];

                  // Only consider the actuation at time t.
                  AD<double> delta0 = vars[delta_start + i];
                  AD<double> a0 = vars[a_start + i];

                  // Create 3rd Order Polynomial
                  AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
                  AD<double> psides0 = CppAD::atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3] * (x0*x0)));

                  // Calculate Residual from State Equations.
                  // The idea here is to constraint this value to be 0.
                  fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
                  fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
                  fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
                  fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
                  fg[2 + cte_start + i] =
                        cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
                  fg[2 + epsi_start + i] =
                        epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
            }
      }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
      bool ok = true;
      size_t i;
      typedef CPPAD_TESTVECTOR(double) Dvector;

      // TODO: Set the number of model variables (includes both states and inputs).
      // For example: If the state is a 4 element vector, the actuators is a 2
      // element vector and there are 10 timesteps. The number of variables is:
      //
      // 4 * 10 + 2 * 9
      size_t n_vars = 6 * N + (N - 1) * 2;
      // TODO: Set the number of constraints
      size_t n_constraints = N * 6;

      // Initialize model variables
      double x = state[0];
      double y = state[1];
      double psi = state[2];
      double v = state[3];
      double cte = state[4];
      double epsi = state[5];

      // Initial value of the independent variables.
      // SHOULD BE 0 besides initial state.
      Dvector vars(n_vars);
      for (int i = 0; i < n_vars; i++) {
            vars[i] = 0;
      }

      // TODO: Set lower and upper limits for variables.
      Dvector vars_lowerbound(n_vars);
      Dvector vars_upperbound(n_vars);

      for (int i = 0; i < delta_start; i++) {
            vars_lowerbound[i] = -1.0e19;
            vars_upperbound[i] = 1.0e19;
      }

      // Set the steering control bounds to -25 and +25 degrees
      // NOTE: Values are set in radians
      for (int i = delta_start; i < a_start; i++) {
            vars_lowerbound[i] = -0.436332;
            vars_upperbound[i] = 0.436332;
      }

      // Set the acceleration control bounds to -1 and +1
      for (int i = a_start; i < n_vars; i++) {
            vars_lowerbound[i] = -1.0;
            vars_upperbound[i] = 1.0;
      }

      // Lower and upper limits for the constraints
      // Should be 0 besides initial state.
      Dvector constraints_lowerbound(n_constraints);
      Dvector constraints_upperbound(n_constraints);
      for (int i = 0; i < n_constraints; i++) {
            constraints_lowerbound[i] = 0;
            constraints_upperbound[i] = 0;
      }

      // Set constrain bounds for initial states
      constraints_lowerbound[x_start] = x;
      constraints_lowerbound[y_start] = y;
      constraints_lowerbound[psi_start] = psi;
      constraints_lowerbound[v_start] = v;
      constraints_lowerbound[cte_start] = cte;
      constraints_lowerbound[epsi_start] = epsi;

      constraints_upperbound[x_start] = x;
      constraints_upperbound[y_start] = y;
      constraints_upperbound[psi_start] = psi;
      constraints_upperbound[v_start] = v;
      constraints_upperbound[cte_start] = cte;
      constraints_upperbound[epsi_start] = epsi;

      // object that computes objective and constraints
      FG_eval fg_eval(coeffs);

      //
      // NOTE: You don't have to worry about these options
      //
      // options for IPOPT solver
      std::string options;
      // Uncomment this if you'd like more print information
      options += "Integer print_level  12\n";

      // Add new solver
      options += "String linear_solver mumps\n";

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

      // TODO: Return the first actuator values. The variables can be accessed with
      // `solution.x[i]`.
      //
      // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
      // creates a 2 element double vector.

      // Clear old MPC values
      mpc_x_vals.clear();
      mpc_y_vals.clear();

      // Save new MPC values
      for (int i = 0; i < N - 1; i++) {
            mpc_x_vals.push_back(solution.x[x_start + i + 1]);
            mpc_y_vals.push_back(solution.x[y_start + i + 1]);
      }

      // Return actuator values
      return{ solution.x[delta_start], solution.x[a_start]};
}
