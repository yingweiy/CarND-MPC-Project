#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
const double Lf = 2.67;

class MPC {
 public:
    const double max_steer_deg = 25;


  MPC();
  //constexpr double pi() { return M_PI; }
  double deg2rad(double x) { return x * M_PI / 180; }
  //double rad2deg(double x) { return x * 180 / M_PI; }

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
