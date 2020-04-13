/*
 * terminal.c
 *
 *  Created on: Feb 18, 2019
 *      Author: kychu
 */

#include "terminal.h"

static struct termios old_settings;
static struct termios new_settings;

void terminal_config(void)
{
	tcgetattr(fileno(stdin), &old_settings);
	new_settings = old_settings;
	new_settings.c_lflag &= ~ECHO; /* close terminal echo */
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(fileno(stdin),TCSANOW, &new_settings);
}

void terminal_config_restore(void)
{
	tcsetattr(fileno(stdin),TCSANOW, &old_settings);
}
