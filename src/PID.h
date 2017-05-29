#ifndef PID_H
#define PID_H

#include <vector>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * twiddle related variables
  */
  int twiddle_ind = 0; //index representing the parameter to change
  int num_ind_updates = 0; // number tracking changes to a parameter

  double best_err = 1e8;
  vector<double> d = { .1, .1, .1 };

  bool twiddle_flag = true; // flag to perform twiddle
  

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Twiddle.
  */
  double twiddle(double err);

  bool getTwiddleFlag();
};

#endif /* PID_H */
