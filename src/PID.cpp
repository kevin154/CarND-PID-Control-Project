#include <numeric>
#include <math.h>
#include <iostream>
#include <limits>
#include "PID.h"

// Max and min steering values
#define MAX_STEERING 1
#define MIN_STEERING -1

PID::PID() {}

PID::~PID() {}

// Initialise PID state
void PID::init(double Kp, double Ki, double Kd) 
{  
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
  
    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;
    
    // Twiddle params
    iter_ = 0;
    best_err_ = std::numeric_limits<double>::infinity();
    step_ = 0;
    total_error_ = 0.0;
    tolerance_met_ = false;
    param_id_ = 0;
    dp_ = {0.1 * Kp_, 0.1 * Ki_, 0.1 * Kd_};
}


void PID::printParams()
{
    std::cout << Kp_ << " " << Ki_ << " " << Kd_ << " " << best_err_ <<std::endl;
}


void PID::updateKp(double Kp)
{
    this->Kp_ = Kp;
}


void PID::updateKi(double Ki)
{
    this->Ki_ = Ki;
}


void PID::updateKd(double Kd)
{
    this->Kd_ = Kd;
}


bool PID::toleranceMet()
{
    return this->tolerance_met_;
}


void PID::updateError(double errorVal) 
{
    // Update PID errors based on error value
    d_error_ = errorVal - p_error_; 
    p_error_ = errorVal;
    i_error_ += errorVal;
}


double PID::totalError() 
{
    // Calculate and return the total error
    double total_error = -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
  
    if (total_error < 0.0)
        return fmax(total_error, MIN_STEERING);
    else
        return fmin(total_error, MAX_STEERING);
}


std::vector<double> PID::twiddle(double cte, double tolerance)
{
    // Copy of current params
    std::vector<double> p = {Kp_, Ki_, Kd_}; 
    
    // Check tolerance
    if(std::accumulate(dp_.begin(), dp_.end(), 0.0) < tolerance)
    {
        tolerance_met_ = true;
        return p;
    }
  
    // Allow the simulation to run 100 times prior to twiddling 
    iter_++;
    total_error_ += fabs(cte);
      
    if (iter_ % 100 != 0) 
        return p;
    
  
    // Take average error and reset error accumulation variable and iteration count
    double current_error = total_error_ / iter_;  
    total_error_ = 0.0;  
    iter_ = 0;
  
    
    // Flag to determine whether to twiddle next parameter
    bool next_param = false;
      
    if (step_ == 0)
    {
        p[param_id_] += dp_[param_id_];
        step_ = 1;
    }
    else if (step_ == 1)
    {
        if (current_error < best_err_)
        {
            best_err_ = current_error;
            dp_[param_id_] *= 1.1;
            step_ = 0;
            next_param = true;
        }
        else
        {   
            p[param_id_] -= 2 * dp_[param_id_];    
            step_ = 2;
        }
    }
    else if (step_ == 2)
    {
        if (current_error < best_err_)
        {
            best_err_ = current_error;
            dp_[param_id_] *= 1.1;
        }    
        else
        {
            p[param_id_] += dp_[param_id_];
            dp_[param_id_] *= 0.9;
        }
        step_ = 0;
        next_param = true;
    }
  
    if (next_param)
    {
        param_id_++; 
        if (param_id_ == 3)
            param_id_ = 0;
    }
    return p;
}