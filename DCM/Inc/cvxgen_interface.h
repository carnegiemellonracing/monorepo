/**
 *  @file cvxgen_interface.h
 *  @author Carnegie Mellon Racing
 *  @brief uses the cvxgen solver to give desired torques
 */

#include "motors_helper.h"
#include "motors.h"

#ifndef _CVXGEN_INTERFACE_H_
#define _CVXGEN_INTERFACE_H_


void cvxgen_init(int max_iters, unsigned int max_time_ms);

bool load_data(
	float steering_angle_rad,
	float T_REQ_Nm,
	float M_REQ_Nm,
	float regen_torque_Nm,
	cmr_torqueDistributionNm_t * torque_max_Nm,
	cmr_torqueDistributionNm_t *result_torques_Nm
	);

#endif  /* _CVXGEN_INTERFACE_H_ */
