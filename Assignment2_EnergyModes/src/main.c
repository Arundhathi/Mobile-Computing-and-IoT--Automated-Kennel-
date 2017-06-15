/***************************************************************************//**
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_letimer.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/*---------------------------------------------------------------------------------------------------------*/
/*This section declares all the macros global variables required by the functions of this code*/
#define Min_EMODE 3 // macro defines the minimum energy mode the processor can enter
#define Cycle_Period 1.5  //defining cycle period in seconds
#define Cycle_On_Time 0.02  //defining on time in seconds
int sleep_block_counter[3] = {0, 0, 0}; //array that dictates the energy mode the processor can enter (EM0/EM1/EM2, default energy mode EM3)


/*---------------------------------------------------------------------------------------------------------*/
/*Routine to set up and enable all the required clocks and peripherals in accordance
 * to the peripherals being used.
	 * Inputs : None
	 * Outputs : None
	 * Macro: Min_EMODE
	 * Global Variables: None
	 * Return Type : Void
	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
*/

 	void CLOCK_TREE_SETUP()
	{

		/*Condition to enable clocks according to the energy mode the board is set in*/
		if (Min_EMODE == 3)
		{
		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true); //enables the ultra low frequency RC Oscillator
		CMU_ClockSelectSet (cmuClock_LFA, cmuSelect_ULFRCO); //enables the LFA clock branch of the ultra low frequency RC Oscillator
		}
		else
		{
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true); //enables low frequency crystal oscillator
		CMU_ClockSelectSet (cmuClock_LFA, cmuSelect_LFXO); //enables the LFA clcok branch of the low frequency crystal oscillator
		}

		CMU_ClockEnable (cmuClock_CORELE, true); //enables core clock for low energy peripherals
		CMU_ClockEnable (cmuClock_LETIMER0, true); //enables clock to LETIMER0
		CMU_ClockEnable (cmuClock_GPIO, true); //enables clock to the GPIO pin set
	}

/*---------------------------------------------------------------------------------------------------------*/
/*Routine to set up and enable features of the low energy timer LETIMER0
 	 * Inputs : None
 	 * Outputs : None
 	 * Macro: Min_EMODE
 	 * Global Variables: None
 	 * Return Type : Void
 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
*/

 	void LETIMER_SETUP()
	{
		const LETIMER_Init_TypeDef letimerInit =
		  {
		  .bufTop         = false,            // Don't load COMP1 into COMP0 when REP0 i.e repeat counter reaches 0
		  .comp0Top       = true,             // Load COMP0 register into CNT when counter underflows to keep timer running continuously only for a certain defined period.
		  .debugRun       = false,            // Counter is instructed to halt when program is stopped for debugging
		  .enable         = true,             // Start counting when initialization is completed
		  .out0Pol        = 0,                // Idle value for output 0
		  .out1Pol        = 0,                // Idle value for output 1
		  .repMode        = letimerRepeatFree, // Count until explicitly stopped
		  .rtcComp0Enable = false,            // Don't start counting on RTC COMP0 match
		  .rtcComp1Enable = false,            // Don't start counting on RTC COMP1 match
		  .ufoa0          = letimerUFOANone,  // Dont take any action on underflow in timer 0
		  .ufoa1          = letimerUFOANone,  // Dont take any action on underflow in timer 0
		  };
	/* Initialize LETIMER */
		LETIMER_Init(LETIMER0, &letimerInit);
	}

/*---------------------------------------------------------------------------------------------------------*/
/*Macro definition to set prescalar on and off*/
#define PRESCALAR_OFF
#ifdef PRESCALAR_ON
	CMU_ClockDivSet(cmuClock_LETIMER0, 2);
	LETIMER_Enable (LETIMER_SYNCBUSY, true);
#endif


/*---------------------------------------------------------------------------------------------------------*/
 /*This section enumerates all functions related to the sleep modes required by the process to switch between energy states. */

/* 1. Routine to limit the sleep() function from entering a lower energy mode according to the clock/peripherals required.
	 * Inputs : minMode
	 * Outputs : None
	 * Macros: none
	 * Return Type : void
	 * Credits: copyrights held by Silicon Labs.
*/
	 void blockSleepMode(int minMode)
	 {
	 	INT_Disable();
	 	sleep_block_counter[minMode]++;
	 	INT_Enable();
	 }

/* 2. Allows sleep() to enter another energy mode after the peripheral's utility is over.
	  * Inputs : minMode
	  * Outputs : None
	  * Macros: none
	  * return Type : void
	  * credit: copyrights held by Silicon Labs.
*/
	 void unblockSleepMode(int minMode)
	 {
	 	INT_Disable();
	 	if(sleep_block_counter[minMode] > 0)
	 	{
	 		sleep_block_counter[minMode]--;
	 	}
	 	INT_Enable();
	 }

/* 3. Routine to appropriate sleeping modes according to defined minimum energy mode
	  * inputs: none
	  * outputs: none
	  * macros : none
	  * global variables: sleep_block_counter[]
	  * return Type: void
	  * Credit: copyrights held by Silicon Labs.
*/
	 void sleep(void){
	 	if (sleep_block_counter[0] > 0){
	 		return;
	 	}
	 	else if(sleep_block_counter[1] > 0){
	 		EMU_EnterEM1();
	 	}
	 	else if(sleep_block_counter[2] > 0){
	 		EMU_EnterEM2(true);
	 	}
	 	else{
	 		EMU_EnterEM3(true);
	 	}
	 }


/*---------------------------------------------------------------------------------------------------------*/
/*This section sets up interrupt features of the LETIMER and defines the interrupt handler for LETIMER Interrupts.
 * The LETIMER for this function employs two interrupts for COMP1 and COMP0.
 * The COMP0 interrupt occurs on completion of every specified period (1.75 seconds).
 * COMP1 interrupts after the completion of every ON time specified (30 mS).

	  * Inputs : None
	  * Outputs : None
	  * Macro: Min_EMODE
	  * Global Variables: None
	  * Return Type : Void
	  * Credit: Copyrights held by Silicon Labs for functions used (emlib)
*/

	 void LETIMER_INTERRUPT_SETUP()
	 {
		 LETIMER0->IFC |= LETIMER_IFC_UF | LETIMER_IFC_REP1 | LETIMER_IFC_REP0 | LETIMER_IFC_COMP1 | LETIMER_IFC_COMP0; //clearing all interrupts in the register
		 LETIMER0->IEN |= LETIMER_IEN_COMP0 | LETIMER_IEN_COMP1 ; //enables comp0 and comp1 interrupts
		 blockSleepMode(Min_EMODE); //restricts the sleep mode to EM3
		 NVIC_EnableIRQ(LETIMER0_IRQn); //instructs NVIC to enable interrupts related to the LETIMER peripheral
		 LETIMER_Enable(LETIMER0, true); // Enables the LETIMER peripheral
	 }

/*Interrupt Handler Routine for LETIMER0
	   * inputs: none
	   * outputs: none
	   * Macros: none
	   * global variables: gpioPortE (from emlib)
	   * return Type: void
	   * credit: Copyrights held by Silicon Labs for certain functions used (emlib)
*/

	 void LETIMER0_IRQHandler(void)
	 {
		INT_Disable();
		LETIMER0->IFC |= LETIMER_IFC_COMP1 | LETIMER_IFC_COMP0;
		GPIO_PinOutToggle(gpioPortE, 2);
		INT_Enable();
	 }


/*---------------------------------------------------------------------------------------------------------*/
/*Routine to set up the GPIO requirements for switching the LED on and off
   * inputs: none
   * outputs: none
   * Macros: none
   * global variables: gpioPortE, pin number, gpioDriveModeStandard (from emlib)
   * return Type: void
   * credit: Copyrights held by Silicon Labs for certain functions used (emlib)
*/
	 void GPIO_CONFIG ()
	 {
		 GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 1); // Configure Port E Pin 2 as a push-pull output pin
		 GPIO_DriveModeSet(gpioPortE, gpioDriveModeStandard); //sets a standard drive mode (6 mA) on port E
	 }


/*---------------------------------------------------------------------------------------------------------*/
/*Routine to calculate the ON time and Total Period for the duty cycle of operation required and to set up the timers to compare those values.
    * inputs: Total_Period, On_Time
    * outputs: none
    * global variables: none
    * return Type: void
    * credit: Copyrights held by Silicon Labs for functions used (emlib)
 */

	 void DUTY_CYCLE_SETUP (float Total_Period, float On_Time )
	 {
        	uint16_t Period_EM3 = (Total_Period*1157); //calculates the total cycle period for ULFRCO frequency
        	uint16_t Period = (Total_Period*32768); //calculates the total cycle period for LFA frequency
        	uint16_t OnTime_EM3 = (On_Time*1157); //calculates the total cycle period for ULFRCO frequency
        	uint16_t OnTime = (On_Time*32768); //calculates the total cycle period for LFA frequency

        	if (Min_EMODE == 3) //conditional to set the duty cycle values according to the respective clock frequencies
        	{
        		LETIMER_CompareSet(LETIMER0, 0,Period_EM3 );//Gives value of 1750 to the compare 0 reg of LETIMER0
        		LETIMER_CompareSet(LETIMER0, 1,OnTime_EM3  );//Gives value of 30 to the compare 1 reg of LETIMER0
        	}
        	else
        	{
        		LETIMER_CompareSet(LETIMER0, 0, Period );//Gives value of 57343 to the compare 0 reg of LETIMER0
        		LETIMER_CompareSet(LETIMER0, 1, OnTime);//Gives value of 983 to the compare 1 reg of LETIMER0
        	}

	}


/*---------------------------------------------------------------------------------------------------------*/
/*Main Routine*/
void main(void)
{
/* Chip errata */
  CHIP_Init();
  CLOCK_TREE_SETUP(); //enables oscillators and clocks to all the required peripherals
  LETIMER_SETUP(); //sets up the timer and count registers for all required characteristics
  DUTY_CYCLE_SETUP (Cycle_Period, Cycle_On_Time); // sets up the duty cycle for blinking the LEDs
  LETIMER_INTERRUPT_SETUP(); //sets up the COMP1 and COMP0 in LETIMER to interrupt
  GPIO_CONFIG (); // Configures the GPIO port as required for the LED

 /* Infinite loop */
  while (1)
  {
	  sleep(); //enters sleep mode according to the energy mode specified.
  }
}
