/*
 * estimator.c
 *
 *  Created on: Apr 13, 2020
 *      Author: kychu
 */

#include "estimator.h"

#include <math.h>

#pragma GCC push_options
#pragma GCC optimize ("O0")
static float fast_inverse_sqrt(float x) {
	float half = 0.5f * x;
	int i = *((int *)&x);
	i = 0x5f3759df - (i >> 1);
	x = *((float *)&i);
	x = x * (1.5f - (half * x * x));
	return x;
}
#pragma GCC pop_options

void fusionQ_6dot(IMU_6DOF_T *unit, Quat_T *q, float prop_gain, float intg_gain, float dt)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;

	float twoKp = prop_gain;    // 2 * proportional gain (Kp)
	float twoKi = intg_gain;    // 2 * integral gain (Ki)

	static float integralFBx = 0.0f;
	static float integralFBy = 0.0f;
	static float integralFBz = 0.0f;  // integral error terms scaled by Ki

	float qw, qx, qy, qz;
	float gx, gy, gz;
	float ax, ay, az;

	qw = q->qw;
	qx = q->qx;
	qy = q->qy;
	qz = q->qz;

	gx = unit->gx;
	gy = unit->gy;
	gz = unit->gz;

	ax = unit->ax;
	ay = unit->ay;
	az = unit->az;

	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		// Normalise accelerometer measurement
		recipNorm = fast_inverse_sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = qx * qz - qw * qy;
		halfvy = qw * qx + qy * qz;
		halfvz = qw * qw - 0.5f + qz * qz;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled

		integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
		integralFBy += twoKi * halfey * dt;
		integralFBz += twoKi * halfez * dt;
		gx += integralFBx;  // apply integral feedback
		gy += integralFBy;
		gz += integralFBz;

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	float delta2 = (gx*gx + gy*gy + gz*gz)*dt*dt;

	float qw_last = qw;
	float qx_last = qx;
	float qy_last = qy;
	float qz_last = qz;

	qw = qw_last*(1.0f-delta2*0.125f) + (-qx_last*gx - qy_last*gy - qz_last*gz)*0.5f * dt;
	qx = qx_last*(1.0f-delta2*0.125f) + (qw_last*gx + qy_last*gz - qz_last*gy)*0.5f * dt;
	qy = qy_last*(1.0f-delta2*0.125f) + (qw_last*gy - qx_last*gz + qz_last*gx)*0.5f * dt;
	qz = qz_last*(1.0f-delta2*0.125f) + (qw_last*gz + qx_last*gy - qy_last*gx)*0.5f * dt;

	// Normalise quaternion
	recipNorm = fast_inverse_sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
	qw *= recipNorm;
	qx *= recipNorm;
	qy *= recipNorm;
	qz *= recipNorm;

	q->qw = qw;
	q->qx = qx;
	q->qy = qy;
	q->qz = qz;
}

void Quat2Euler(Quat_T* q, Euler_T* eur)
{
	float qw = q->qw;
	float qx = q->qx;
	float qy = q->qy;
	float qz = q->qz;

	eur->roll    = atan2f(2 * (qw * qx + qy * qz) , 1 - 2 * (qx * qx + qy * qy))*RAD_TO_DEG;  //+-90
	eur->pitch   = asinf(2 * (qw * qy - qz * qx))*RAD_TO_DEG;                                 //+-180
	eur->yaw     = atan2f(2 * (qw * qz + qx * qy) , 1 - 2 * (qy * qy + qz * qz))*RAD_TO_DEG;  //+-180
}
