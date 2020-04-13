/*
 * adis.c
 *
 *  Created on: Mar 25, 2020
 *      Author: kychu
 */

#include "adis16470drv.h"
#include "uart.h"
#include "sys_def.h"

#include "estimator.h"

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>

#define STR1(R) #R
#define STR2(R) STR1(R)

static pthread_t est_thread;
static pthread_t cal_thread;

static bool_t _should_exit = false;

static bool_t _calibrate_flag = false;

static pthread_mutex_t sensor_if_mtx = PTHREAD_MUTEX_INITIALIZER;

static float imu_temp;
static imu_6dof_t imu_6dof;
static pthread_mutex_t imu_6dof_mtx = PTHREAD_MUTEX_INITIALIZER;
static quaternion_t est_q;
static pthread_mutex_t est_q_mtx = PTHREAD_MUTEX_INITIALIZER;
static euler_t est_e;
static pthread_mutex_t est_e_mtx = PTHREAD_MUTEX_INITIALIZER;

static const uint8_t config_spi[] = {0x5A, 0x02, 0x93, 0x05}; /* MODE3 1MHz */

static void adis_imu_est_task(void);
static void adis_imu_cal_task(void);

int adis16470_start(const char *dev)
{
  if(uart_open(dev, (const char *)"9600") != EXIT_SUCCESS) {
    printf("\e[0;31mfailed to open device %s.\e[0m\n", dev);
    return -1;
  }
  if(uart_write((char *)config_spi, 4) <= 0) {
    printf("\e[0;31mfailed to config SPI bus.\e[0m\n");
    return -2;
  }
  uart_flush_read();
  uart_flush_write();
  if(pthread_create(&est_thread, NULL, (void *)adis_imu_est_task, NULL) != 0) {
    printf("\e[0;31mfailed to start estimate task.\e[0m\n");
    return -3;
  }
  return 0;
}

const char *adis16470_sdk_version(void)
{
  return "kyChu ADIS16470 SDK"" <"STR2(__VERSION_STR__)">" "@<"__DATE__ " " __TIME__ ">";
}

int adis16470_stop(void)
{
  _should_exit = true;
  pthread_join(est_thread, NULL);
  pthread_join(cal_thread, NULL);
  uart_close();
  return 0;
}

int adis16470_get_temperature(float *val)
{
  *val = imu_temp;
  return 0;
}

int adis16470_get_6dof_unit(imu_6dof_t *unit)
{
  pthread_mutex_lock(&imu_6dof_mtx);
  *unit = imu_6dof;
  pthread_mutex_unlock(&imu_6dof_mtx);
  return 0;
}

int adis16470_get_quaternion(quaternion_t *q)
{
  pthread_mutex_lock(&est_q_mtx);
  *q = est_q;
  pthread_mutex_unlock(&est_q_mtx);
  return 0;
}

int adis16470_get_euler_angle(euler_t *e)
{
  pthread_mutex_lock(&est_e_mtx);
  *e = est_e;
  pthread_mutex_unlock(&est_e_mtx);
  return 0;
}

int adis16470_calibrate_start(void)
{
  if(_calibrate_flag == true) return -1;
  _calibrate_flag = true;
  if(pthread_create(&cal_thread, NULL, (void *)adis_imu_cal_task, NULL) != 0) {
    printf("\e[0;31mfailed to start calibration task.\e[0m\n");
    return -2;
  }
  return 0;
}

int adis16470_calibrate_status(void)
{
  if(_calibrate_flag == true) return 1;
  return 0;
}

static void adis_write_register(uint8_t addr, uint16_t data)
{
  uint8_t buff[5] = {0x61, 0x00, 0x00, 0x00, 0x00};
  buff[1] = addr | 0x80;
  buff[3] = (addr + 1) | 0x80;
  buff[2] = data & 0xFF;
  buff[4] = data >> 8;
  pthread_mutex_lock(&sensor_if_mtx);
  uart_write((char *)buff, 5);
  uart_flush_write();
  uart_read((char *)buff, 5);
  uart_flush_read();
  pthread_mutex_unlock(&sensor_if_mtx);
}

static int16_t adis_combine_word(uint8_t* p)
{
  int16_t ret;
  if(p[0] & 0x80) {
    ret = - ~(((uint16_t)p[0] << 8) | p[1]);
  } else {
    ret = ((int16_t)p[0] << 8) | p[1];
  }
  return ret;
}

static uint16_t adis_conbine_uword(uint8_t *p)
{
  return ((uint16_t)p[0] << 8) | p[1];
}

#define ADIS_CALIBRATION_LOOP_TIME_S             (40)
static void adis_imu_cal_task(void)
{
  if(_calibrate_flag == true) {
    printf("\n calibrate started!\n");
    printf("!!!keep IMU motionless (%ds) !!!\n", ADIS_CALIBRATION_LOOP_TIME_S);
    adis_write_register(0x66, 0x070A);
    for(int i = 0; i < ADIS_CALIBRATION_LOOP_TIME_S; i ++) {
      sleep(1);
      printf("time remain %d s...   \r", ADIS_CALIBRATION_LOOP_TIME_S - i - 1);
      fflush(stdout);
      if(_should_exit == true) {
        printf("!!!calibration been terminated!!!");
        break;
      }
    }
    adis_write_register(0x68, 0x0001);
    _calibrate_flag = false;
    printf("\n calibrate done.\n");
  }
}

static void adis_imu_est_task(void)
{
  uint8_t test_cmd[28];
  uint8_t rx_cache[28];

  uint16_t current_cnt = 0, last_cnt = 0;
  float delta_ts_ms = 0;
  uint8_t checksum = 0;

  _calibrate_flag = true;
  if(pthread_create(&cal_thread, NULL, (void *)adis_imu_cal_task, NULL) != 0) {
    printf("\e[0;31mfailed to start calibration task.\e[0m\n");
    return;
  }

  est_q.qw = 1.0f;
  est_q.qx = 0.0f;
  est_q.qy = 0.0f;
  est_q.qz = 0.0f;
  est_e.roll = 0.0f;
  est_e.pitch = 0.0f;
  est_e.yaw = 0.0f;

  memset(test_cmd, 0, 28);
  memset(rx_cache, 0, 28);
  test_cmd[0] = 0x61;
  test_cmd[1] = 0x68;
  test_cmd[2] = 0x00;
  while(_should_exit == false) {
    if(_calibrate_flag == false) {
      pthread_mutex_lock(&sensor_if_mtx);
      uart_write((char *)test_cmd, 24);
      uart_flush_write();
      uart_read((char *)rx_cache, 24);
      pthread_mutex_unlock(&sensor_if_mtx);

      pthread_mutex_lock(&imu_6dof_mtx);
      imu_6dof.gx = adis_combine_word(rx_cache + 5) * 0.1f;
      imu_6dof.gy = adis_combine_word(rx_cache + 7) * 0.1f;
      imu_6dof.gz = adis_combine_word(rx_cache + 9) * 0.1f;
      imu_6dof.ax = adis_combine_word(rx_cache + 11) * 1.25f * 0.0098f;
      imu_6dof.ay = adis_combine_word(rx_cache + 13) * 1.25f * 0.0098f;
      imu_6dof.az = adis_combine_word(rx_cache + 15) * 1.25f * 0.0098f;
      pthread_mutex_unlock(&imu_6dof_mtx);
      imu_temp = adis_combine_word(rx_cache + 17) * 0.1f;
      current_cnt = adis_conbine_uword(rx_cache + 19);

      if(current_cnt < last_cnt)
        delta_ts_ms = (0x10000 - last_cnt + current_cnt) * 0.5f;
      else
        delta_ts_ms = (current_cnt - last_cnt) * 0.5f;//adis_conbine_uword(rx_cache + 19);
      last_cnt = current_cnt;

      checksum = 0;
      for(int i = 0; i < 18; i ++) {
        checksum += *(rx_cache + 3 + i);
      }
      if(checksum == rx_cache[22]) {
        if(delta_ts_ms != 0) {
          pthread_mutex_lock(&imu_6dof_mtx);
          pthread_mutex_lock(&est_q_mtx);
          fusionQ_6dot(&imu_6dof, &est_q, 1.0, 0.0, delta_ts_ms * 0.001f);
          pthread_mutex_unlock(&imu_6dof_mtx);
          pthread_mutex_lock(&est_e_mtx);
          Quat2Euler(&est_q, &est_e);
          pthread_mutex_unlock(&est_q_mtx);
          pthread_mutex_unlock(&est_e_mtx);
        }
      }
    }
    usleep(1000);
  }
}
