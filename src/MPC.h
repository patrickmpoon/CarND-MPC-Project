#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


const double MPH_to_METERS_PER_SECOND = 0.44704;

// In degrees
const double MAX_STEERING_ANGLE = 25.0;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from the vehicle's front to its center-of-gravity that has a similar radius.
const double Lf = 2.67;

/*************************
 * Duration components
 *************************/
// Number of timesteps
const size_t N = 10;
// Time between steps in seconds: delta time
const double dt = 0.2;


class MPC {
 public:
  MPC();

  virtual ~MPC();

  double steer_value;
  double throttle_value;
  vector<double> x_predictions;
  vector<double> y_predictions;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
