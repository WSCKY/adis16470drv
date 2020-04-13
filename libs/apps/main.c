/*
 * main.c
 *
 *  Created on: Feb 25, 2019
 *      Author: kychu
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "sys_def.h"

#include "adis16470drv.h"
#include "terminal.h"
#include "kysocket.h"

static bool_t _should_exit = false;

static pthread_t exit_thread;

static euler_t est_e;
static imu_6dof_t imu_6dof;

struct imu_info_t {
  float gx, gy, gz, ax, ay, az, temp, cntr, chksum;
} imu_info;

static void adis_exit_task(void);

int main(int argc, char *argv[]) {
  int ch;
  char *ip_addr = "127.0.0.1";
  const char *dev = "/dev/ttyACM0";

  terminal_config();

  printf("\e[0;31madis16470 SDK test\e[0m\n");
  while ((ch = getopt(argc, argv, "d:a:")) != -1) {
    switch (ch) {
    case 'a':
      ip_addr = optarg;
      printf("ip addr = %s", ip_addr);
    break;
    case 'd':
      dev = optarg;
    break;
    case '?':
      printf("Unknown option: %c\n", (char)optopt);
    break;
    }
  }

  if(pthread_create(&exit_thread, NULL, (void *)adis_exit_task, NULL) != 0) {
    printf("\e[0;31mfailed to start exit_task.\e[0m\n");
    goto exit;
  }

  if(sck_connect(ip_addr) != 0) {
    printf("\e[0;31mfailed to create socket %s.\e[0m\n", ip_addr);
    goto exit;
  }

  ch = adis16470_start(dev);
  if(ch < 0) {
    printf("\e[0;31mfailed to start adis16470 driver %d.\e[0m\n", ch);
    goto exit;
  }

  while(_should_exit == false) {
    usleep(10000);
    adis16470_get_6dof_unit(&imu_6dof);
    adis16470_get_temperature(&imu_info.temp);
    imu_info.ax = imu_6dof.ax;
    imu_info.ay = imu_6dof.ay;
    imu_info.az = imu_6dof.az;
    imu_info.gx = imu_6dof.gx;
    imu_info.gy = imu_6dof.gy;
    imu_info.gz = imu_6dof.gz;
    sck_send_package(&imu_info, 0x19, sizeof(imu_info));
    usleep(10000);
    adis16470_get_euler_angle(&est_e);
    sck_send_package(&est_e, 0x18, sizeof(euler_t));
  }

  adis16470_stop();

exit:
  _should_exit = true;
  pthread_join(exit_thread, NULL);
  printf("\n\e[0;31mTEST DONE\e[0m\n");

  terminal_config_restore();

  return EXIT_SUCCESS;
}

static void adis_exit_task(void)
{
  while(_should_exit == false) {
    usleep(10000);
    int v = getchar();
    switch(v) {
      case 0x1d: // detect for 'CTRL+['
        _should_exit = true;
      break;
      case 0x09: // CTRL+i
        printf("'CTRL+I' pressed\n");
//        adis_write_register(0x66, 0x070A);
//        printf("set bias estimating time (GLOB_CMD)\n");
      break;
      case 0x0A: // CTRL+j
        printf("'CTRL+J' pressed\n");
      break;
      case 0x0B: // CTRL+k
        printf("'CTRL+K' pressed\n");
//        adis_write_register(0x68, 0x0001);
//        printf("Bias correction update (GLOB_CMD)\n");
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
