/**
 * @file    adis16470drv.h
 * @author  kyChu<kychu@qq.com>
 * @date    Apr 13, 2020
 * @version V1.0.0
 * @brief   API for ADIS16470 IMU driver.
 */

#ifndef _ADIS16470DRV_H_
#define _ADIS16470DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * type definitions
 */
/* sensor data in unit */
typedef struct {
  float gx, gy, gz; // (deg/s)
  float ax, ay, az; // (m/s^2)
} imu_6dof_t;

/* quaternion */
typedef struct {
  float qw, qx, qy, qz;
} quaternion_t;

/* euler angle */
typedef struct {
  float pitch, roll, yaw;
} euler_t;

/*
 * @brief  start adis16470 sensor.
 * @param  dev: device node, e.g. "/dev/ttyACM0".
 * @retval 0 is OK, or negative value means error.
 */
int adis16470_start(const char *dev);

/*
 * @brief  get SDK version string.
 * @param  none
 * @retval version string.
 */
const char *adis16470_sdk_version(void);

/*
 * @brief  read temperature form sensor.
 * @param  <float *>: pointer to a float number to hold temperature data.
 * @retval 0 always.
 */
int adis16470_get_temperature(float *);

/*
 * @brief  read temperature form sensor.
 * @param  <imu_6dof_t *>: pointer to a buffer to hold sensor data.
 * @retval 0 always.
 */
int adis16470_get_6dof_unit(imu_6dof_t *);

/*
 * @brief  read temperature form sensor.
 * @param  <quaternion_t *>: pointer to a quaternion structure to hold quaternion data.
 * @retval 0 always.
 */
int adis16470_get_quaternion(quaternion_t *);

/*
 * @brief  read temperature form sensor.
 * @param  <euler_t *>: pointer to a euler_t structure to hold EULER angle data.
 * @retval 0 always.
 */
int adis16470_get_euler_angle(euler_t *);

/*
 * @brief  start sensor calibration task.
 * @param  none
 * @retval 0 is OK, -1 means already started, other: error.
 */
int adis16470_calibrate_start(void);

/*
 * @brief  get calibration status.
 * @param  none
 * @retval 0: NOT start, 1: started.
 */
int adis16470_calibrate_status(void);

/*
 * @brief  exit safely.
 * @param  none
 * @retval none
 */
int adis16470_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* _ADIS16470DRV_H_ */

/******************** kyChu<kyChu@qq.com> **** END OF FILE ********************/
