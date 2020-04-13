/*
 * terminal.h
 *
 *  Created on: Feb 18, 2019
 *      Author: kychu
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include <stdio.h>
#include <termios.h>

void terminal_config(void);
void terminal_config_restore(void);

#endif /* TERMINAL_H_ */
