/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

    /*
    * Errors
    */
    double error_p;
    double error_i;
    double error_d;

    /*
    * Coefficients
    */
    double k_p;
    double k_i;
    double k_d;

    /*
    * Output limits
    */
    double lim_max_output;
    double lim_min_output;

    /*
    * Integral clamp bounds, derived from the output limits for anti-windup
    */
    double i_error_min;
    double i_error_max;

    /*
    * Delta time
    */
    double delta_t;
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
    void init_controller(double Kp, double Ki, double Kd, double output_lim_maxi, double output_lim_mini);

    /*
    * Update the PID error variables given cross track error.
    */
    void update_error(double cte);

    /*
    * Calculate the total PID error.
    */
    double total_error();
  
    /*
    * Update the delta time.
    */
    double update_delta_time(double new_delta_time);
};

#endif //PID_CONTROLLER_H


