/*
 * estimator.h
 *
 *  Created on: Apr 13, 2020
 *      Author: kychu
 */

#ifndef APPS_ESTIMATOR_H_
#define APPS_ESTIMATOR_H_

typedef struct {
  float gx, gy, gz, ax, ay, az;
} IMU_6DOF_T;

typedef struct {
  float qw, qx, qy, qz;
} Quat_T;

typedef struct {
  float pitch, roll, yaw;
} Euler_T;

void fusionQ_6dot(IMU_6DOF_T *unit, Quat_T *q, float prop_gain, float intg_gain, float dt);
void Quat2Euler(Quat_T* q, Euler_T* eur);

#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#endif /* APPS_ESTIMATOR_H_ */
