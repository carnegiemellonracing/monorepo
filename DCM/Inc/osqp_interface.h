/**
 *  @file osqp_interface.h
 *  @author Carnegie Mellon Racing
 *  @brief uses the osqp solver to give desired results
 */

#ifndef _OSQP_INTERFACE_H_
#define _OSQP_INTERFACE_H_

void osqp_guess(float FL, float FR, float RL, float RR);

void osqp_init(int max_iters);

float solve_with_osqp(const double steering_angle,
           const double M_REQ,
           const double T_REQ,
           double *resFR,
           double *resFL,
           double *resRR,
           double *resRL
);

#endif  /* _OSQP_INTERFACE_H_ */
