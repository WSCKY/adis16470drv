/*
 * main.c
 *
 *  Created on: Feb 25, 2019
 *      Author: kychu
 */

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "uart.h"
#include "adis.h"
#include "terminal.h"
#include "kysocket.h"

int main(int argc, char *argv[]) {
	int ch;
	char *baud = "9600";
	char *ip_addr = "192.168.1.2";
	const char *dev = "/dev/ttyACM0";

	terminal_config();

	printf("\e[0;31mencoder test\e[0m\n");
	while ((ch = getopt(argc, argv, "f:d:b:a:")) != -1) {
		switch (ch) {
			case 'a':
				ip_addr = optarg;
				printf("ip addr = %s", ip_addr);
			break;
			case 'b':
				baud = optarg;
			break;
			case 'd':
				dev = optarg;
			break;
			case '?':
				printf("Unknown option: %c\n", (char)optopt);
			break;
		}
	}

	if(uart_open(dev, baud) != EXIT_SUCCESS) {
		printf("\e[0;31mfailed to open uart %s.\e[0m\n", dev);
		terminal_config_restore();
		return EXIT_FAILURE;
	}

	if(sck_connect(ip_addr) == 0) {
		adis_config();
		adis_start();
		adis_wait_exit();
	} else {
		printf("\e[0;31mfailed to create socket %s.\e[0m\n", ip_addr);
	}

	uart_close();

	printf("\n\e[0;31mTEST DONE\e[0m\n");

	terminal_config_restore();

	return EXIT_SUCCESS;
}
