/*
 * adis.h
 *
 *  Created on: Mar 25, 2020
 *      Author: kychu
 */

#ifndef APPS_ADIS_H_
#define APPS_ADIS_H_

#include <stdio.h>

int adis_start(void);
int adis_config(void);
void adis_wait_exit(void);

#endif /* APPS_ADIS_H_ */
