/*
 * 		File: sleep.c
 *
 *  	Created on: Feb 8, 2017
 *      Author: Vignesh Jayaram
 *
 *      Copyright: The code used in this file is taken from SiliconLabs.
 *
 *     	Permission is granted to anyone to use this software for any purpose,
 *  	including commercial applications, and to alter it and redistribute it
 *  	freely, subject to the following restrictions:
 *
 * 	 	1. The origin of this software must not be misrepresented; you must not
 *  	  claim that you wrote the original software.
 *  	2. Altered source versions must be plainly marked as such, and must not be
 *  	  misrepresented as being the original software.
 *  	3. This notice may not be removed or altered from any source distribution.
 *
 *  	DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 *  	obligation to support this Software. Silicon Labs is providing the
 *  	Software "AS IS", with no express or implied warranties of any kind,
 *  	including, but not limited to, any implied warranties of merchantability
 *  	or fitness for any particular purpose or warranties against infringement
 *  	of any proprietary rights of a third party.
 *
 *  	Silicon Labs will not be liable for any consequential, incidental, or
 *  	special damages, or any other relief, or for any claim by any third party,
 *  	arising from your use of this Software.
 *
 *      sleep():
 *      This routine is used to enter into the desired energy modes
 *
 *      blockSleepMode()
 *      This routine is used to set the minimum energy mode. Once set, the energy mode does not change.
 *
 *      unblockSleepMode()
 *      This routine is used to exit from the set energy mode back to EM0.
 *
 *
 */

#include "sleep.h"

int sleep_block_counter[3] = {0, 0, 0};		//Array to determine which sleep mode to enter

/* ****************************************************
 * Function is used to enter the desired energy mode
 *
 * Input variable:  none
 * Global variable: sleep_block_counter
 * Local variable: none
 * Return type: none
 *
 *******************************************************/
void sleep(void)
{
	if (sleep_block_counter[0] > 0){		//Go to Energy Mode 0
		return;
	}
	else if(sleep_block_counter[1] > 0){    //Go to Energy Mode 1
		EMU_EnterEM1();
	}
	else if(sleep_block_counter[2] > 0){	//Go to Energy Mode 2
		EMU_EnterEM2(true);
	}
	else{
		EMU_EnterEM3(true);					//Go to Energy Mode 3
	}
}


/* ******************************************************************************
 * Function to set the minimum energy mode, so that energy mode does not go lower
 * Thus allowing to enter any other energy mode as required
 *
 * Input variable:  minMode
 * Global variable: sleep_block_counter
 * Local variable: none
 * Return type: none
 *
 ********************************************************************************/
void blockSleepMode(int minMode)
{
	INT_Disable();
	sleep_block_counter[minMode]++;
	INT_Enable();
}


/* ******************************************************************************
 * This routine is used to exit from the set energy mode back to EM0.
 *
 * Input variable:  minMode
 * Global variable: sleep_block_counter
 * Local variable: none
 * Return type: none
 *
 ********************************************************************************/
void unblockSleepMode(int minMode)
{
	INT_Disable();
	if(sleep_block_counter[minMode] > 0)
	{
		sleep_block_counter[minMode]--;
	}
	INT_Enable();
}

