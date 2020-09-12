#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void updateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double totalError();
 
  // Updates PID params
  void updateKp(double Kp);
  void updateKi(double Ki);
  void updateKd(double Kd);
  
  bool toleranceMet();
  
  void printParams();
  
  // Calculates params based on twiddle algorithm
  std::vector<double> twiddle(double cte, double tolerance);

 private:
  /**
   * PID Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;

  /**
   * PID Coefficients
   */ 
  double Kp_;
  double Ki_;
  double Kd_;
    
  // Twiddle params
  int iter_;
  double best_err_;
  short step_;
  double total_error_;
  bool tolerance_met_;
  short param_id_;
  std::vector<double> dp_;
};

#endif  // PID_H