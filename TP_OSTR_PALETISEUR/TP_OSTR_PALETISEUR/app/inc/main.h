/*
 * main.h
 *
 *  Created on: 27 févr. 2020
 *      Author: delkeke
 */

#include "event.h"
#include "delay.h"
#include "bsp.h"
#include "stm32f0xx_it.h"
#include "stm32f0xx.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"

#ifndef APP_INC_MAIN_H_
#define APP_INC_MAIN_H_

/*
 * printf() and sprintf() from printf-stdarg.c
 */

int my_printf	(const char *format, ...);
int my_sprintf	(char *out, const char *format, ...);


#endif /* APP_INC_MAIN_H_ */
