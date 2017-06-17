/*
 * 		File: sleep.h
 *
 *  	Created on: Feb 8, 2017
 *      Author: Vignesh Jayaram
 *
 *      sleep():
 *      This routine is used to enter into the desired energy modes
 *
 *      blockSleepMode()
 *      This routine is used to set the minimum energy mode. Once set, the energy mode does not change.
 *
 *      unblockSleepMode()
 *      This routine is used to exit from the set energy mode back to EM0.
 */

#ifndef SLEEP_H
#define SLEEP_H

#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "em_letimer.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_adc.h"
#include "em_dma.h"
#include "em_common.h"
#include "dmactrl.h"

#define INT_CLR_ALL 0x11111111	// Macro to clear all interrupts

void sleep(void);
void blockSleepMode(int minMode);
void unblockSleepMode(int minMode);

#endif /* SRC_SLEEP_H_ */
