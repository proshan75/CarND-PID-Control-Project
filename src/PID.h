#ifndef PID_H
#define PID_H

#include <iostream>     // std::cout
#include <limits>       // std::numeric_limits
#include <vector>
using namespace std;

class PID {
public:
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Twiddle tolerance
  */ 
  double Tolerance;
  int Current_index;
  int Current_iteration;
  const int MAX_TWIDDLE_ITERATIONS = 400;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double twiddle_tol = 0.2);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Perform twiddle correction.
  */
  void Twiddle(double error); 
  void AddError(double cte);
  double CalculateError(int num_of_errors);
  bool ShouldRunTwiddle();
  void ResetTwiddle();

private:
  vector<double> _dp, _p;
  double _best_error, _cte_total;
  bool _is_storing_best_error;
};

#endif /* PID_H */
