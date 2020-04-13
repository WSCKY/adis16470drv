/*
 * estimator.h
 *
 *  Created on: Apr 13, 2020
 *      Author: kychu
 */

#ifndef APPS_ESTIMATOR_H_
#define APPS_ESTIMATOR_H_

#include "adis16470drv.h"

void fusionQ_6dot(imu_6dof_t *unit, quaternion_t *q, float prop_gain, float intg_gain, float dt);
void Quat2Euler(quaternion_t* q, euler_t* eur);

#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#endif /* APPS_ESTIMATOR_H_ */
