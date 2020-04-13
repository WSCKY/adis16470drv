/*
 * adis.c
 *
 *  Created on: Mar 25, 2020
 *      Author: kychu
 */

#include "adis.h"
#include "uart.h"

#include "kysocket.h"
#include "estimator.h"

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>

static pthread_t read_thread;
static pthread_t mesg_thread;
static pthread_t exit_thread;
static int exit_flag = 0;

static IMU_6DOF_T imu_6dof;
static Quat_T est_q;
static Euler_T est_e;

static const uint8_t config_spi[] = {0x5A, 0x02, 0x93, 0x05}; /* MODE3 1MHz */

struct imu_info_t {
  float gx, gy, gz, ax, ay, az, temp, cntr, chksum;
} imu_info;

//static const uint8_t get_temp_cmd[3] = {0x61, 0x1c, 0x1d};

static void adis_read_task(void);
static void adis_mesg_task(void);
static void adis_exit_task(void);

int adis_start(void)
{
  uart_flush_read();
  uart_flush_write();
  if(pthread_create(&read_thread, NULL, (void *)adis_read_task, NULL) != 0)
    return EXIT_FAILURE;
  if(pthread_create(&exit_thread, NULL, (void *)adis_exit_task, NULL) != 0)
    return EXIT_FAILURE;
  return EXIT_SUCCESS;
}

int adis_config(void)
{
  if(uart_write((char *)config_spi, 4) > 0) {
    uart_flush_read();
    return EXIT_SUCCESS;
  }
  return EXIT_FAILURE;
}

void adis_wait_exit(void)
{
	pthread_join(read_thread, NULL);
	pthread_join(mesg_thread, NULL);
	pthread_join(exit_thread, NULL);
}

static void adis_write_register(uint8_t addr, uint16_t data)
{
  uint8_t buff[5] = {0x61, 0x00, 0x00, 0x00, 0x00};
  buff[1] = addr | 0x80;
  buff[3] = (addr + 1) | 0x80;
  buff[2] = data & 0xFF;
  buff[4] = data >> 8;
  uart_write((char *)buff, 5);
  uart_flush_write();
  uart_read((char *)buff, 5);
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
static void adis_calib_loop(void)
{
  printf("\n calibrate started!\n");
  printf("!!!keep IMU motionless (%ds) !!!\n", ADIS_CALIBRATION_LOOP_TIME_S);
  adis_write_register(0x66, 0x070A);
  for(int i = 0; i < ADIS_CALIBRATION_LOOP_TIME_S; i ++) {
    sleep(1);
    printf("time remain %d s...   \r", ADIS_CALIBRATION_LOOP_TIME_S - i - 1);
    fflush(stdout);
  }
  adis_write_register(0x68, 0x0001);
  printf("\n calibrate done.\n");
}

static uint8_t test_cmd[28];
static uint8_t rx_cache[28];
static void adis_read_task(void)
{
  uint16_t current_cnt = 0, last_cnt = 0;
  uint8_t checksum = 0;
  memset(test_cmd, 0, 28);

  adis_calib_loop();

  est_q.qw = 1.0f;
  est_q.qx = 0.0f;
  est_q.qy = 0.0f;
  est_q.qz = 0.0f;
  est_e.roll = 0.0f;
  est_e.pitch = 0.0f;
  est_e.yaw = 0.0f;

  if(pthread_create(&mesg_thread, NULL, (void *)adis_mesg_task, NULL) != 0) {
    printf("mesg_task start failed!\n");
    return;
  }

  test_cmd[0] = 0x61;
  test_cmd[1] = 0x68;
  test_cmd[2] = 0x00;
  while(exit_flag == 0) {
    uart_write((char *)test_cmd, 24);
    uart_flush_write();
//    usleep(20);
    uart_read((char *)rx_cache, 24);

    imu_6dof.gx = imu_info.gx = adis_combine_word(rx_cache + 5) * 0.1f;
    imu_6dof.gy = imu_info.gy = adis_combine_word(rx_cache + 7) * 0.1f;
    imu_6dof.gz = imu_info.gz = adis_combine_word(rx_cache + 9) * 0.1f;
    imu_6dof.ax = imu_info.ax = adis_combine_word(rx_cache + 11) * 1.25f * 0.0098f;
    imu_6dof.ay = imu_info.ay = adis_combine_word(rx_cache + 13) * 1.25f * 0.0098f;
    imu_6dof.az = imu_info.az = adis_combine_word(rx_cache + 15) * 1.25f * 0.0098f;
    imu_info.temp = adis_combine_word(rx_cache + 17) * 0.1f;
    current_cnt = adis_conbine_uword(rx_cache + 19);
    if(current_cnt < last_cnt)
      imu_info.cntr = (0x10000 - last_cnt + current_cnt) * 0.5f;
    else
      imu_info.cntr = (current_cnt - last_cnt) * 0.5f;//adis_conbine_uword(rx_cache + 19);
    last_cnt = current_cnt;

    checksum = 0;
    for(int i = 0; i < 18; i ++) {
      checksum += *(rx_cache + 3 + i);
    }
    if(checksum != rx_cache[22]) {
      imu_info.chksum = 0;
    } else {
      imu_info.chksum = 1;
      if(imu_info.cntr != 0) {
        fusionQ_6dot(&imu_6dof, &est_q, 1.0, 0.0, imu_info.cntr * 0.001f);
        Quat2Euler(&est_q, &est_e);
      }
    }

//    sck_send_package(&imu_info, 0x19, sizeof(imu_info));
//    printf("g: %.2f, %.2f, %.2f     a: %.2f, %.2f, %.2f        \r", imu_info.gx, imu_info.gy, imu_info.gz, imu_info.ax, imu_info.ay, imu_info.az);
//    fflush(stdout);
    usleep(1000);
  }
}

static void adis_mesg_task(void)
{
  while(exit_flag == 0) {
    usleep(10000);
    sck_send_package(&imu_info, 0x19, sizeof(imu_info));
    usleep(10000);
//    sck_send_package(&est_q, 0x12, sizeof(Quat_T));
    sck_send_package(&est_e, 0x18, sizeof(Euler_T));
  }
}

static void adis_exit_task(void)
{
  while(exit_flag == 0) {
    usleep(10000);
    int v = getchar();
    switch(v) {
      case 0x1d: // detect for 'CTRL+['
        exit_flag = 1;
      break;
      case 0x09: // CTRL+i
        printf("'CTRL+I' pressed\n");
        adis_write_register(0x66, 0x070A);
        printf("set bias estimating time (GLOB_CMD)\n");
      break;
      case 0x0A: // CTRL+j
        printf("'CTRL+J' pressed\n");
      break;
      case 0x0B: // CTRL+k
        printf("'CTRL+K' pressed\n");
        adis_write_register(0x68, 0x0001);
        printf("Bias correction update (GLOB_CMD)\n");
      break;
      case 0x0C: // CTRL+l
        printf("'CTRL+L' pressed\n");
      break;
      default:
        printf("\nkey:0x%x\n", v);
      break;
    }
  }
}
