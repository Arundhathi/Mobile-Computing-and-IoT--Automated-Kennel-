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
#include "em_timer.h"
#include "em_acmp.h"
#include "em_adc.h"
#include "dmactrl.h"
#include "em_dma.h"

/*---------------------------------------------------------------------------------------------------------*/
/*This section declares all the macros global variables required by the functions of this code*/
/*Variables and macros related to sleep modes and LETIMER module, TIMER0 and TIMER 1 modules*/
int sleep_block_counter[3] = {0, 0, 0}; //array that dictates the energy mode the processor can enter (EM0/EM1/EM2, default energy mode EM3)

/*Compile Time switches and values*/
#define CALIBRATION true //Macro to define utility of calibration
#define DMA_REQUIREMENT true// Macro to switch requirement of DMA within the program
#define Cycle_Period 5.5  //defining cycle period in seconds
#define LED_On_Time 0.004  //defining on time in seconds
#define Min_EMODE 3 //Macro defining minimum energy mode for LETIMER



/*defining an enum to set clock types for prescalar and calibration*/
typedef enum clock_type {
	LFXO_c = 0,
	ULFRCO_c = 1,
} clock_c;

uint32_t T0_Count = 0; //variable to set timer 0 count value
uint32_t T1_Count = 0; //variable to set timer 1 count value
uint32_t Reference_Count = 0; //variable to set reference count
uint32_t Caliberted_ULFRCO; //variable to set corrected ULFRCO value
uint32_t ULFRCO_Period = 0; //variable to set ULFRCO time period
uint32_t ULFRCO_On_Time = 0; //variable to set ULFRCO on time
uint32_t ULFRCO_Time_Unit_c =0; //variable to set required time unit for caliberated ULFRCO
uint32_t Final_LETIMER_Value_Cycle = 0; //variable to set calibrated period time
uint32_t Final_LETIMER_On_Time_Value = 0;//variable to set calibrated on time
uint32_t Prescalar;//variable to set prescalar value for presco register
#define LFXO_Time_Unit 32768  //Macro to define LFXO counts for 1 s
#define ULFRCO_Time_Unit 1000 //Macro to define ULFRCO counts for 1 s
#define LETIMER0_Max_Count 65535 //Macro to define LETIMER CNT registers max count


/*General Macros*/
#define CLR_INTERRUPTS 0x11111111 ///Macro to define register clear byte
#define CLEAR_REG 0x00000000



/*Gloablvariables and macros related to peripherals*/
/*1 - LEDS*/
#define LED_Port gpioPortE //Macro to define LED port
#define LED0_Pin 2 //Macro to define pin of LED0
#define LED1_Pin 3 // Macro to define pin of LED1
#define LED0 0
#define LED1 1

/*2. - ACMP */
uint32_t flag; //general purpose flag
uint32_t acmp_diff  = 0; //variable to get vale of ACMP
#define acmp_VDD acmpChannelVDD
#define acmp_excite_input acmpChannel6

/*3. - LESENSE*/
#define LESENSE_Sensor_Port gpioPortC //Macro to define Light sensor port
#define LESENSE_Sensor_Pin 6 //Macro to define light sensor pin
#define LESENSE_Excite_Port gpioPortD //Macro to define light sensor excite port
#define LESENSE_Excite_Pin 6 //Macro to define light sensor excite pin
#define High_Threshold_Light 61 //Macro to define light level
#define Low_Threshold_Dark  2 //Macro to define darkness level

/*4. - Temperature sensor */
#define TEMP_LOW_THRESHOLD 15//Macro to define low threshold of temperature
#define TEMP_HIGH_THRESHOLD 35 //Macro to define high threshold of temperature

/*5. - ADC */
uint32_t no_of_samples_nd; //Macro to define no of samples without dma
uint32_t no_of_samples_d; //Macro to define no of samples with dma
#define Sleep_Mode_ADC 1 //Macro to define energy mode of ADC
#define MAX_SAMPLES 750 //Macro to define maximum number of samples that adc needs to read
#define Single_Mode adcStartSingle //Macro to define ADC to start in single measurement mode
#define Prescalar_ADC 49   //Value to set prescalar of ADC
#define Acquisition_Time adcAcqTime2
#define Reference_Voltage adcRef1V25
#define Resolution adcRes12Bit

/*6. - DMA */
#define DMA_Arbitration_Rate dmaArbitrate1 //arbitrates after every single two-bye transfer
#define Destination_Increment_Byte dmaDataInc2 //increments register by 2 bytes after every data transfer
#define HPROT_Privilege 0 //macro that defines privilege mode of dma for source read accesses
#define DATA_Transfer_Size dmaDataSize2 //transfers two bytes of data in every transfer
#define Source_Increment_Byte dmaDataIncNone //doesn't increment source of dma data transfers
#define DMA_Channel_ADC 0 //Macro to define which channel to use
#define TOTAL_SAMPLES 750 //Macro to define maximum bytes to be transferred
#define Enable_CalBack_Functions true //Macro to define enabling call back function im lieu of interrupt handler
#define Set_High_Priority true //Macro to change priority of channel
#define Use_Primary_Desciptor true //Macro to define use of primary descriptor
#define Use_Busrt_Mode false //Macro to define burst mode transfer eligibility
volatile uint16_t ramBufferAdcTempData[TOTAL_SAMPLES]; //Array to store the ADC values


/* user defined structure to set attributes of ACMP */
static ACMP_Init_TypeDef acmpInit =
	  {
		    .fullBias                 = false, /* The lightsensor is slow acting,but accurate */
		    .halfBias                 = true,  /* comparator bias current can be set to lowest setting thus conserving energy*/
		    .biasProg                 = 0x0,   /* Analog comparator will still be fast enough */
		    .interruptOnFallingEdge   = false, /* No comparator interrupt, light sense will issue interrupts. */
		    .interruptOnRisingEdge    = false,
		    .warmTime                 = acmpWarmTime256, /*warm up time wrt HFPER clock. */
		    .hysteresisLevel          = acmpHysteresisLevel4, /* Some hysteresis will prevent unwanted values from being picked up. */
		    .inactiveValue            = false, /* Not applicable */
		    .lowPowerReferenceEnabled = false, /* Can be enabled for even lower powerig set */
		    .vddLevel                 = Low_Threshold_Dark,  /*sets .acmpThreshold default value. */
		    .enable                   = false  /* chooses not to enable acmp immediately */
	  };
/*---------------------------------------------------------------------------------------------------------*/
 /*This section enumerates all functions related to the sleep modes required by the process to switch between energy states. */
/**************************************************************************//**
 * @file
 * @brief blockSleep() routine, unblockSleep() routine and sleep() routine for EFM32LG_STK3600
 ******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
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
/*--------------------------------------------------------------------------------------*/
/*Routine to blink LED
	 	  	 * Inputs : LED_ID
	 	 	 * Outputs : None
	 	 	 * Macro: LED_Port, LED0_Pin, LED1_Pin
	 	 	 * Global Variables: None
	 	 	 * Return Type : Void
	 	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
void LED_BLINK (uint8_t LED_ID)
{
	if(LED_ID == 0)
	{
		GPIO_PinOutSet(LED_Port,LED0_Pin);
	}

	else
	{
		GPIO_PinOutSet(LED_Port,LED1_Pin);
	}

}
/*--------------------------------------------------------------------------------------*/
/*Routine to switch off LED
 	 * Inputs : LED_ID
	 * Outputs : None
	 * Macro: LED_Port, LED0_Pin, LED1_Pin
	 * Global Variables: None
	 * Return Type : Void
	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
void LED_OFF (uint8_t LED_ID)
{
	if (LED_ID == 0)
	{
		GPIO_PinOutClear(LED_Port,LED0_Pin);
	}
	else
	{
		GPIO_PinOutClear(LED_Port,LED1_Pin);
	}
}

/*Routine to set up and enable all the required clocks and peripherals in accordance
	  * to the peripherals being used.
	 	 * Inputs : None
	 	 * Outputs : None
	 	 * Macro: Min_EMODE
	 	 * Global Variables: F_Clock
	 	 * Return Type : Void
	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
*/

void OSCILLATOR_SETUP(clock_c F_Clock)
	 {
	 	CMU_ClockEnable(cmuClock_HFPER, true);

	 	 if (F_Clock == 0)
	 	 	{
	 		 	 CMU_OscillatorEnable(cmuOsc_LFXO, true, true); //enables low frequency crystal oscillator
	 		 	 CMU_ClockSelectSet (cmuClock_LFA, cmuSelect_LFXO); //enables the LFA clcok branch of the low frequency crystal oscillator
	 	 	}
	 	 	else
	 	 	{
	 	 		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true); //enables low frequency crystal oscillator
	 	 		CMU_ClockSelectSet (cmuClock_LFA, cmuSelect_ULFRCO); //enables the LFA clcok branch of the low frequency crystal oscillator
	 	 	}

	 	 		CMU_ClockEnable(cmuClock_CORELE, true);//enables core clock for low energy peripherals
	 	 		CMU_ClockEnable(cmuClock_LETIMER0, true);//enables low energy timer
	 	 	 }

#if CALIBRATION == 1
/*ALl functions within the defined barriers are used to perform calibration of ULFRCO in EM3*/
/*-----------------------------------------------------------------------------*/
/*Routine to set up and enable all the required timer attributes for both timer 1 and timer 0
	 	 * Inputs : None
	 	 * Outputs : None
	 	 * Macro: None
	 	 * Global Variables:
	 	 * Return Type : Void
	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
void TIMER_SETUP()
	 	{
	 	 	 CMU_ClockEnable(cmuClock_TIMER0, true); //enables clock to timer 0
		 	 CMU_ClockEnable(cmuClock_TIMER1, true); //enables clock to timer1
		 	 TIMER_Init_TypeDef timer0Init =
	 		{
	 			.enable     = true,
	 			.debugRun   = false,
	 			.prescale   = timerPrescale1, //sets prescale to 1
	 			.clkSel     = timerClkSelHFPerClk, //clocked from HFPER clock
	 			.count2x    = false,
	 			.ati        = false,
	 			.fallAction = timerInputActionNone,
	 			.riseAction = timerInputActionNone,
	 			.mode       = timerModeUp, //upcounting mode
	 			.dmaClrAct  = false,
	 			.quadModeX4 = false ,
	 			.oneShot    = false,
	 			.sync       = true,
	 		};

	 		TIMER_Init(TIMER0, &timer0Init); //initialise timer

	 		TIMER_Init_TypeDef timer1Init =
	 		{
	 					.enable     = true,
	 					.debugRun   = false,
	 					.prescale   = timerPrescale1, //prescaled by 1 power
	 					.clkSel     = timerClkSelCascade, //is cascaded to timer 0
	 					.count2x    = false,
	 					.ati        = false,
	 					.fallAction = timerInputActionNone,
	 					.riseAction = timerInputActionNone,
	 					.mode       = timerModeUp, //up counting mode
	 					.dmaClrAct  = false,
	 					.quadModeX4 = false ,
	 					.oneShot    = false,
	 					.sync       = true,

	 		};

	 		TIMER_Init(TIMER1, &timer1Init); //initialise timer

	 	}

/*--------------------------------------------------------------------------------------------------------*/
/*Routine to set up and enable all the required timer attributes for LETIMER
	 	 * Inputs : None
	 	 * Outputs : None
	 	 * Macro: None
	 	 * Global Variables: C_Clock
	 	 * Return Type : Void
	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
void LF_LETIMER_SETUP(clock_c C_Clock)
	 	{

			LETIMER_Enable(LETIMER0, false);
	 		const LETIMER_Init_TypeDef letimerInit =
	 				  {
	 				  .bufTop         = false,            // Don't load COMP1 into COMP0 when REP0 i.e repeat counter reaches 0
	 				  .comp0Top       = false,             // Load COMP0 register into CNT when counter underflows to keep timer running continuously only for a certain defined period.
	 				  .debugRun       = false,            // Counter is instructed to halt when program is stopped for debugging
	 				  .enable         = false,             // Start counting when initialization is completed
	 				  .out0Pol        = 0,                // Idle value for output 0
	 				  .out1Pol        = 0,                // Idle value for output 1
	 				  .repMode        = letimerRepeatOneshot, // Count once till set value
	 				  .rtcComp0Enable = false,            // Don't start counting on RTC COMP0 match
	 				  .rtcComp1Enable = false,            // Don't start counting on RTC COMP1 match
	 				  .ufoa0          = letimerUFOANone,  // Dont take any action on underflow in timer 0
	 				  .ufoa1          = letimerUFOANone,  // Dont take any action on underflow in timer 0
	 				  };
	 			/* Initialize LETIMER */

	 		 LETIMER0->CNT = CLEAR_REG;
	 		 LETIMER_Init(LETIMER0, &letimerInit);
	 				if(C_Clock == 0)
	 				{
	 					LETIMER0->CNT = LFXO_Time_Unit; //sets 32768 for LFXO
	 				}
	 				else
	 				{
	 					LETIMER0->CNT = ULFRCO_Time_Unit; //sets 1000 for ULFRCO
	 				}

	 		 LETIMER_Enable(LETIMER0, true);//enables LETIMER with set attributes
	 	}

/*-----------------------------------------------------------------------------------------*/

/*Routine to set up and perform calibration of ulfrco in em3
	 	 * Inputs : None
	 	 * Outputs : None
	 	 * Macro: None
	 	 * Global Variables: None
	 	 * Return Type : uint32_t
	 	 * Credit: none
	 */
uint32_t ULFRCO_Calibration()
	 	{
	 		uint32_t LFXO_Calibrated_Count_A = 0; //local variable to hold lfxo count from timers
	 		uint32_t ULFRCO_Calibrated_Count_B = 0; //local variable to hold ulfrco count from timers


	 		OSCILLATOR_SETUP(LFXO_c); //calls oscillator set up lfxo
	 		TIMER_SETUP(); //sets up timers
	 		LF_LETIMER_SETUP(LFXO_c); //calls letimer setup for lfxo

	 		while(LETIMER0->CNT != 0) //polls letimer coutn register for 0
	 		{};

	 		TIMER_Enable(TIMER0, false); //disables timers
	 		TIMER_Enable(TIMER1, false);

	 		LFXO_Calibrated_Count_A = (TIMER1->CNT << 16) | TIMER0->CNT; //stores timer value

	 	/*-------------------------------------------------------------------------------*/
	 		LETIMER_Enable(LETIMER0, false); //calls oscillator set up lfxo
	 		TIMER0->CNT = CLEAR_REG; //clearing timer regs
	 		TIMER1->CNT = CLEAR_REG; // clearing timer regs

	 		OSCILLATOR_SETUP(ULFRCO_c);//calls oscillator set up ulfrco
	 		TIMER_SETUP();//sets up timers
	 		LF_LETIMER_SETUP(ULFRCO_c);//calls letimer setup for ulfrco

	 		while(LETIMER0->CNT != 0)//polls letimer coutn register for 0
	 		{};
	 		ULFRCO_Calibrated_Count_B = (TIMER1->CNT << 16) | TIMER0->CNT;//stores timer value

/******************Calculating the correction ratio for ulfrco******************************************************/
	 		float Correction_Ratio = 0;
	 		uint32_t Corrected_Count = 0;
	 		Correction_Ratio = (float)LFXO_Calibrated_Count_A / (float)ULFRCO_Calibrated_Count_B;
	 		Corrected_Count = (uint32_t) (Correction_Ratio * 1000);
	 		return Corrected_Count;
	 	}

#endif
/*---------------------------------------------------------------------------------------------------*/
/*Routine to set up features of gpio pins of LED0, Light Sensor and Light Excite pins
	 	 * Inputs : None
	 	 * Outputs : None
	 	 * Macro: None
	 	 * Global Variables: None
	 	 * Return Type : none
	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
void GPIO_SETUP()
	 {
	    CMU_ClockEnable (cmuClock_GPIO, true);
		GPIO_PinModeSet(LED_Port, LED0_Pin, gpioModePushPull, 1);		// Configure Pe2 as push pull and output*/
	 	GPIO_DriveModeSet(LED_Port, gpioDriveModeStandard);	// Standard drive mode
	 	GPIO_PinModeSet(LED_Port, LED1_Pin, gpioModePushPull, 1);		// Configure Pe3 as push pull and output*/
	 	GPIO_DriveModeSet(LED_Port, gpioDriveModeStandard);	// Standard drive mode

	 	GPIO_DriveModeSet(LESENSE_Sensor_Port, gpioDriveModeStandard);	// Standard drive mode
	 	GPIO_PinModeSet(LESENSE_Sensor_Port, LESENSE_Sensor_Pin, gpioModeInput, 1);		// Configure Light sensor as input

	 	GPIO_DriveModeSet(LESENSE_Excite_Port, gpioDriveModeStandard);	// Standard drive mode
	 	GPIO_PinModeSet(LESENSE_Excite_Port, LESENSE_Excite_Pin, gpioModePushPull, 1);	//Configure Excitation as pushpull
	 }
/*--------------------------------------------------------------------------------------------------------*/
/*Routine to calculate the prescalar value required to enable a time interval of 2.5 s
	 	 * Inputs : clock_c type P_Clock
	 	 * Outputs : Period_Prescalar
	 	 * Macro: None
	 	 * Global Variables: P_Clock, Cycle_Period, LFXO_Time_Unit, LED_On_Time, LETIMER0_Max_Count, Final_LETIMER_Value_Cycle, Final_LETIMER_On_Time_Value
	 	 * Return Type : uint32_t
	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
uint32_t PRESCALAR_CALCULATOR(clock_c P_Clock)
	{
	uint32_t Period_Presacalar = 0;

	if (P_Clock == 0) //checking if LFXO is being calibrated
		{

		uint32_t Required_LETIMER_Count_Cycle = 0;
		uint32_t Required_LETIMER_Count_On_Time = 0;

		Required_LETIMER_Count_Cycle = Cycle_Period * LFXO_Time_Unit; //calculating required period count for comp0 in LFXO
		Required_LETIMER_Count_On_Time = LED_On_Time * LFXO_Time_Unit;//calculating required on time count for comp1 in LFXO

		while (Required_LETIMER_Count_Cycle > LETIMER0_Max_Count)  //checking if prescaled time is less than max count of letimer0 vnt reg i.e, 65535
			{
			Period_Presacalar++; //ioncrementing prescalar
			Required_LETIMER_Count_Cycle =  Required_LETIMER_Count_Cycle / 2; //calculating prescaled period count
			Required_LETIMER_Count_On_Time = Required_LETIMER_Count_On_Time / 2; //calculating prescaled on time count
			Final_LETIMER_Value_Cycle = Required_LETIMER_Count_Cycle;
			Final_LETIMER_On_Time_Value = Required_LETIMER_Count_On_Time;
			}

		}
		return Period_Presacalar;
	}
/*-----------------------------------------------------------------------------------------------------------*/
/*Routine to set features of LETIMER to get interrupts at appropriate times
	 	 * Inputs : none
	 	 * Outputs : none
	 	 * Macro: ULFRCO_Period
	 	 * Global Variables: none
	 	 * Return Type : void
	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
void LETIMER_SETUP()
	{
		const LETIMER_Init_TypeDef letimerInit =
				  {
				  .bufTop         = false,            // Don't load COMP1 into COMP0 when REP0 i.e repeat counter reaches 0
				  .comp0Top       = true,             // Load COMP0 register into CNT when counter underflows to keep timer running continuously only for a certain defined period.
				  .debugRun       = false,            // Counter is instructed to halt when program is stopped for debugging
				  .enable         = false,             // Start counting when initialization is completed
				  .out0Pol        = 0,                // Idle value for output 0
				  .out1Pol        = 0,                // Idle value for output 1
				  .repMode        = letimerRepeatFree, // Count until explicitly stopped
				  .rtcComp0Enable = false,            // Don't start counting on RTC COMP0 match
				  .rtcComp1Enable = false,            // Don't start counting on RTC COMP1 match
				  .ufoa0          = letimerUFOANone,  // Dont take any action on underflow in timer 0
				  .ufoa1          = letimerUFOANone,  // Dont take any action on underflow in timer 0
				  };
			/* Initialize LETIMER */


	        LETIMER0->IFC |= LETIMER_IFC_UF | LETIMER_IFC_REP1 | LETIMER_IFC_REP0 | LETIMER_IFC_COMP1 | LETIMER_IFC_COMP0; //clearing all interrupts in the register
	        LETIMER0->IEN |= LETIMER_IEN_COMP0 | LETIMER_IEN_COMP1 ; //enables comp0 and comp1 interrupts
	        LETIMER0->CNT = ULFRCO_Period;//setting count reg with ulfrco period
	        LETIMER_Init(LETIMER0, &letimerInit); //initialising letimer values with above features
	        NVIC_EnableIRQ(LETIMER0_IRQn); //instructs NVIC to enable interrupts related to the LETIMER peripheral
	        blockSleepMode(Min_EMODE); //setting block mode to minimum specified mode

	        LETIMER_Enable(LETIMER0, true); // Enables the LETIMER peripheral


	}
/*-----------------------------------------------------------------------------------------------------------*/
/*Routine to set times at which interrupts will occur
	 	 * Inputs : none
	 	 * Outputs : none
	 	 * Macro: ULFRCO_Period, LED_On_Time, Cycle_Period
	 	 * Global Variables: ULFRCO_Time_Unit_c, ULFRCO_Period, ULFRCO_On_Time, Final_LETIMER_Value_Cycle, Final_LETIMER_On_Time_Value
	 	 * Return Type : void
	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
void DUTY_CYCLE_SETUP ()
	{
	        if (Min_EMODE == 3) //conditional to set the duty cycle values according to the respective clock frequencies
	        {
	            if (CALIBRATION == true)
	            {
	            ULFRCO_Period = (Cycle_Period*ULFRCO_Time_Unit_c);
				ULFRCO_On_Time = (LED_On_Time*ULFRCO_Time_Unit_c);
	            LETIMER_CompareSet(LETIMER0, 0,ULFRCO_Period);//Gives value of 1750 to the compare 0 reg of LETIMER0
	            LETIMER_CompareSet(LETIMER0, 1,ULFRCO_On_Time);//Gives value of 30 to the compare 1 reg of LETIMER0
	            }
	            else
	            {
	            ULFRCO_Period = (Cycle_Period*ULFRCO_Time_Unit);
				ULFRCO_On_Time = (LED_On_Time*ULFRCO_Time_Unit);
	            LETIMER_CompareSet(LETIMER0, 0,ULFRCO_Period);//Gives value of 1750 to the compare 0 reg of LETIMER0
	            LETIMER_CompareSet(LETIMER0, 1,ULFRCO_On_Time);//Gives value of 30 to the compare 1 reg of LETIMER0
	            }
	        }
	         else
	         {
	        	 CMU->LFAPRESC0 = (Prescalar << 8); // sets prescalar for lfxo clock
	        	 LETIMER_CompareSet(LETIMER0, 0, Final_LETIMER_Value_Cycle );//Gives prescaled to the compare 0 reg of LETIMER0
	             LETIMER_CompareSet(LETIMER0, 1, Final_LETIMER_On_Time_Value);//Gives prescaled value of to the compare 1 reg of LETIMER0
	         }


	}
/*------------------------------------------------------------------------------------------------------------*/
/*Routine to set features of ACMP to get interrupts at appropriate times
	 	 * Inputs : none
	 	 * Outputs : none
	 	 * Macro: acmp_VDD, acmp_excite_input
	 	 * Global Variables: none
	 	 * Return Type : void
	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
void ACMP_SETUP()
	{
	    CMU_ClockEnable(cmuClock_ACMP0, true);
		ACMP_Init(ACMP0, &acmpInit);
		ACMP_ChannelSet(ACMP0, acmp_VDD, acmp_excite_input);
		ACMP_Enable(ACMP0);
	}


/*------------------------------------------------------------------------------------------------------------*/
/*Routine to set up ADC
  * Inputs : none
	* Outputs : none
	* Macro: Prescalar_ADC, Acquisition_Time, Reference_Voltage, Resolution
	* Global Variables: none
	* Return Type : void
	* Credit: Copyrights held by Silicon Labs for functions used (emlib)
	*/
void ADC_SETUP ()
{
		CMU_ClockEnable(cmuClock_ADC0, true); //enables clock to the ADC module
		uint8_t time_base = 0; //variable to store time_base value
		uint32_t freq =0; // variable to store frequency of the HFPER clock
		freq = CMU_ClockFreqGet(cmuClock_HFPER); //gets frequency of HFPER
		time_base = ADC_TimebaseCalc(freq); //function to alculate time base required by ADC
		const ADC_Init_TypeDef adc0 = //type def to initialise ADC attributes
		{
			.ovsRateSel = adcOvsRateSel2, //oversampling rate set to 0
			.lpfMode  = adcLPFilterBypass, //bypasses low pass filter within ADC
			.warmUpMode = adcWarmupNormal, //normal warm up mode activated
			.prescale = Prescalar_ADC, //sets prescalar to 49 effectively dividing frquency by 50
			.tailgate = false,
			.timebase = time_base, //sets time base to calculated value
		};
		ADC_Init(ADC0, &adc0); //initializes ADC) with above set attributes

		const ADC_InitSingle_TypeDef adcs = //typedef to set attributes of  single sample mode of ADC
		{
				.acqTime = Acquisition_Time, //Sample ascquisition time set to 1
				.diff = false, //doesnt take diffenrential input
				.input = adcSingleInputTemp, //routes channel to temperature sensor i.e. channel 8
				.leftAdjust = false, //
				.prsEnable = false, // disables the peripheral reflex system
				.reference = Reference_Voltage, //sets refernce voltage to 1.25 V
				.rep = true, //repetition mode is enabled
				.resolution = Resolution, //sets sample resolution to 12 bits
		};

		ADC_InitSingle(ADC0, &adcs); //initialises single mode adc operations with above attributes
		ADC0->IFC = CLR_INTERRUPTS; //clears all interrupts
		ADC_IntEnable(ADC0, ADC_IEN_SINGLE); //enables SINGLE interrupt that produces an interrupt every time a single sample is collected
		//NVIC_EnableIRQ(ADC0_IRQn);
}


/*---------------------------------------------------------------------------------------------------------------*/
/*Routine to convert adc samped analog values into deg celcius
 * IP owned by SILICON LABS
 * @file
 * @brief convertToCelius for EFM32LG_STK3600
 ******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/
float convertToCelcius (int32_t adcSample)
	{
		/*Factory Calibration manual*/
	float temp;
	/* Factory calibration temperature from device information page. */
	float cal_temp_0 = (float)((DEVINFO->CAL& _DEVINFO_CAL_TEMP_MASK)>> _DEVINFO_CAL_TEMP_SHIFT);
	float cal_value_0 = (float)((DEVINFO->ADC0CAL2& _DEVINFO_ADC0CAL2_TEMP1V25_MASK)>> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
	/* Temperature gradient(from datasheet) */
	float t_grad = -6.27;
	temp = (cal_temp_0 -((cal_value_0 -adcSample)  / t_grad));
	return temp;
	}

/*-------------------------------------------------------------------------------------------------------------*/
/*Routine to pass ADC samples to the conversion function and collect temperature values
 * * Inputs : none
   * Outputs :
   * Macro: MAX_SAMPLES, Sleep_Mode_ADC
   * Global Variables: no_of_samples_nd
   * Return Type : float temp_CValue
   * Credit: Copyrights held by Silicon Labs for functions used (emlib)
 */
float GetTemp ()
	{
		uint32_t total_nd = 0; //stores the sum of samples converted by the ADC
		no_of_samples_nd = 0; //stores the number of samples
		int32_t avg_sample_value_nd = 0; //stores average value of all samples taken
		float temp_CValue_nd = 0; //stores converted temperature values
		blockSleepMode(Sleep_Mode_ADC); //limits processor from going in any energy mode below EM1
		ADC_Start(ADC0,Single_Mode); //starts the ADC
		while (no_of_samples_nd != MAX_SAMPLES) //collects samples till number of samples reaches 750
		{
			while (!(ADC0->IF & ADC_IFS_SINGLE)); //polls interrupts bit of ADC singe mode
			total_nd += ADC0->SINGLEDATA; //collects analog value from the adc single data reg instantaneously
			no_of_samples_nd++;
			ADC_IntClear(ADC0, ADC_IFC_SINGLE); //clears flag

		}
		ADC0->CMD = ADC_CMD_SINGLESTOP; //stops ADC operation from CMD reg
		unblockSleepMode(Sleep_Mode_ADC); //allows processor to go to energy modes below EM1
		avg_sample_value_nd = (total_nd / MAX_SAMPLES); //calculates avg
		temp_CValue_nd = convertToCelcius(avg_sample_value_nd);//converts to readable deg celcius format
		return temp_CValue_nd; //returns temperature value

 	}

/*--------------------------------------------------------------------------------------------------------------*/
/*Call back routine serviced every time DMA interrupts
   * Inputs : channel, primary, user(ptr)
   * Outputs : None
   * Macro: MAX_SAMPLES, Sleep_Mode_ADC, LED0, LED1
   * Global Variables: no_of_samples_d, ramBufferAdcTempData[]
   * Return Type : none
   * Credit: Copyrights held by Silicon Labs for functions used (emlib)
 */
void DMA_ADC_Call_Back_Function(unsigned int channel, bool primary, void *user)
{
	INT_Disable();
	ADC0->CMD = ADC_CMD_SINGLESTOP;					//Disable ADC
	unblockSleepMode(Sleep_Mode_ADC);
	CMU_ClockEnable(cmuClock_ADC0, false);
									//Disable interrupts
	DMA_IntClear(DMA_IFC_CH0DONE);					//clear the DMA interrupts


	uint32_t total_d = 0; //stores the sum of samples converted by the ADC
	no_of_samples_d = 0; //stores the number of samples
	int32_t avg_sample_value_d = 0; //stores average value of all samples taken
	float temp_CValue_d = 0; //stores converted temperature values

	for (uint16_t i=0; i < TOTAL_SAMPLES ; i++)		//Add all the 750 samples
	    {
	            total_d += ramBufferAdcTempData[i];
	    }
	    avg_sample_value_d = (total_d / TOTAL_SAMPLES);				//Take average of the samples

	temp_CValue_d = convertToCelcius(avg_sample_value_d);	//Calculate the temperature in celsius

	if (temp_CValue_d < TEMP_LOW_THRESHOLD || temp_CValue_d > TEMP_HIGH_THRESHOLD)
	    {
		GPIO_PinOutSet(LED_Port,LED1_Pin);
		//LED_BLINK(LED1);
	    }
	    else
	    {
	    	GPIO_PinOutClear(LED_Port,LED1_Pin);
	    	//LED_OFF(LED1);
	    }

	INT_Enable();
	}

/*Structure to set attributes of DMA Call Back Routine*/
DMA_CB_TypeDef adc0_cb =								// DMA callback structure
{
		.cbFunc = DMA_ADC_Call_Back_Function, //defines call back function
		.primary = true, //sets attribute to primary
		.userPtr = NULL
};


/*-----------------------------------------------------------------------------------------------------------------*/
/*Call back routine to set up DMA wrt ADC
   * Inputs : None
   * Outputs : None
   * Macro: DMA_Arbitration_Rate, Destination_Increment_Byte, HPROT_Privilege, DATA_Transfer_Size, Source_Increment_Byte, Set_High_Priority, Enable_CalBack_Functions
   * Global Variables: no_of_samples_d, ramBufferAdcTempData[]
   * Return Type : none
   * Credit: Copyrights held by Silicon Labs for functions used (emlib)
 */
void ADC0_DMA_SETUP(void)
{
	const DMA_CfgDescr_TypeDef ADC0_DMA_cfg =
		{
			.arbRate = DMA_Arbitration_Rate, //no of DMA transfers after which re-arbitration takes place
			.dstInc = Destination_Increment_Byte, //increments destination by two bytes as data size transferred is two bytes at a time
			.hprot = HPROT_Privilege, //setting control for source read accesses
			.size =  DATA_Transfer_Size, // NO of bytes transferred at one time by dma is set to 2 as 750 is divisible by 2
			.srcInc = Source_Increment_Byte //set to not increment source as the source register will be the same for all data transfers
		};
	DMA_CfgDescr(DMA_Channel_ADC, true, &ADC0_DMA_cfg);	//Configure the descriptorwith above set typedef values

	/*Contruct to set attributes of dma channel 0*/
	DMA_CfgChannel_TypeDef DMA_Channel_Config =
		{
			.cb = &adc0_cb, //specifies the call back function that needs to be called after the dma transfers
			.enableInt = Enable_CalBack_Functions, //enabled if a call back routine has to be implemented after data transfers
			.highPri = Set_High_Priority, //dma channel 0 has been given higher priority
			.select =  DMAREQ_ADC0_SINGLE //selects single mode of operation for ADc wrt DMA
		};

	DMA_CfgChannel(DMA_Channel_ADC,&DMA_Channel_Config);

	DMA_IntClear(DMA_IFC_CH0DONE); //clear interrupts
	DMA_IntEnable(DMA_IFC_CH0DONE); // enable interrupt for channel 0
}

/*------------------------------------------------------------------------------------------------------------------*/
/*Call back routine to set up DMA
   * Inputs : None
   * Outputs : None
   * Macro: HPROT_Privilege
   * Global Variables: None
   * Return Type : none
   * Credit: Copyrights held by Silicon Labs for functions used (emlib)
 */

void DMA_SETUP()
{
	 CMU_ClockEnable(cmuClock_DMA, true); //enables clock to the DMA module
	 const DMA_Init_TypeDef DMA_INIT =
	 {
			 .controlBlock = dmaControlBlock,
			 .hprot = HPROT_Privilege //sets privilege mode for all src accesses
	 };
	 DMA_Init(&DMA_INIT);    /* initial DMA dma Control Block descriptor location */
	 ADC0_DMA_SETUP();
	 NVIC_EnableIRQ(DMA_IRQn);
}

/*IRQ handler routine for LETIMER interrupts comp0 and copm1
	 	 * Inputs : none
	 	 * Outputs : none
	 	 * Macro: acmp_VDD, acmp_excite_input
	 	 * Global Variables: none
	 	 * Return Type : void
	 	 * Credit: Copyrights held by Silicon Labs for functions used (emlib)
	 */
/*NOTES:
 *When the voltage on the positive input is higher than the voltage on the negative input, the digital output is high and vice versa.
 *Lesense is negative coefficient, i.e. resistance and voltage decrease with increase in light intensity
*/
void LETIMER0_IRQHandler(void)
{
	INT_Disable();							//Disable interrupts
		flag = LETIMER_IntGet(LETIMER0);		//Check if COMP0 and COMP1 interrupt
		if(flag & LETIMER_IF_COMP0)
		{
			acmp_diff = (ACMP0->STATUS & ACMP_STATUS_ACMPOUT);
			GPIO_PinOutClear(LESENSE_Excite_Port,LESENSE_Excite_Pin);	//Disable the excitation

			ACMP0->CTRL &= ~ACMP_CTRL_EN;		//Disable ACMP0

			LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0);		//Clear COMP interrupt

			if(acmp_diff)
			{
				if(acmpInit.vddLevel == Low_Threshold_Dark)
				{
					acmpInit.vddLevel = High_Threshold_Light; //switch threshold to detect light
					ACMP_Init(ACMP0,&acmpInit); //reinitialise acmp0 with new value of VDD
					ACMP_ChannelSet(ACMP0, acmp_VDD , acmp_excite_input); //set negsel and possel channels
					//LED_BLINK(LED0); //set led
					GPIO_PinOutSet(LED_Port,LED0_Pin);
				}
				else
				{
					acmpInit.vddLevel = Low_Threshold_Dark; ////switch threshold to detect darkness
					ACMP_Init(ACMP0,&acmpInit); //reinitialise acmp0 with new value of VDD
					ACMP_ChannelSet(ACMP0, acmp_excite_input, acmp_VDD);//switch negSel and posSel channels
					//LED_OFF(LED0);//switch off led
					GPIO_PinOutClear(LED_Port,LED0_Pin);

				}
			 }
/*Compile time switch to switch between using the dma and calculating temperature straight from the ADC values*/
			#if DMA_REQUIREMENT == 1
			DMA_ActivateBasic(
						DMA_Channel_ADC, //sets which channel to use i.e. 0
						Use_Primary_Desciptor, 			// activates DMA using primary desciptor for channel
						Use_Busrt_Mode,  //disables useBurst that is upported only by certain peripherals
						(void *)ramBufferAdcTempData,		//destination address
						(void *)&(ADC0->SINGLEDATA),	//source address i.e. register of ADC used to store sampled digital values
						 (TOTAL_SAMPLES - 1)//for n-1 samples where n is the number of data bytes required
						 );			//No of ADC reads required
			CMU_ClockEnable(cmuClock_ADC0, true);
			blockSleepMode(Sleep_Mode_ADC);							//Enter Block Sleep Mode 1
			ADC_Start(ADC0, Single_Mode);					//Enable ADC

			#else
			float temperature = 0;
			temperature = GetTemp(); //function to get temperature
			if (temperature < TEMP_LOW_THRESHOLD || temperature > TEMP_HIGH_THRESHOLD)
			{
				//LED_BLINK(LED1); //glows LED
				GPIO_PinOutSet(LED_Port,LED1_Pin);
			}
			else
			{
				//LED_OFF(LED1); //switches off LEd
				GPIO_PinOutClear(LED_Port,LED1_Pin);
			}
            #endif
		 }
		else /*Routine for interrupt comp1 after 2.5 s*/
		{
			ACMP0->CTRL |= ACMP_CTRL_EN;		//Enable ACMP0
			GPIO_PinOutSet(LESENSE_Excite_Port,LESENSE_Excite_Pin);	//Enable the excitation
			LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP1);		//Clear COMP1 interrupt
			while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;	//Check if warm-up is complete
		}
		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0 | LETIMER_IFC_COMP1);		//Clear the interrupts
		INT_Enable();							//Enable interrupts
}


/*--------------------MAIN ROUTINE------------------------------------------------------------------------*/
int main(void)

{
	 //int  i;
	 /* Chip errata */
    CHIP_Init();

#if CALIBRATION == true //call calibration routine
    ULFRCO_Time_Unit_c = ULFRCO_Calibration();
    CMU_ClockEnable(cmuClock_HFPER, false); //switch off all clocks to conserve energy
    CMU_OscillatorEnable(cmuOsc_LFXO, false, false);
    CMU_OscillatorEnable(cmuOsc_ULFRCO, false, false);
    CMU_ClockEnable(cmuClock_CORELE, false);
    CMU_ClockEnable(cmuClock_LETIMER0, false);
    CMU_ClockEnable(cmuClock_TIMER0, false); //disables clock to timer 0
    CMU_ClockEnable(cmuClock_TIMER1, false); //disables clock to timer1
 #endif

    Prescalar = PRESCALAR_CALCULATOR (LFXO_c); //calculate prescalar for lfxo
    if (Min_EMODE == 3) //initilaise ulfrco for min energy mode 3
    {
    	OSCILLATOR_SETUP(ULFRCO_c);
    	CMU_OscillatorEnable(cmuOsc_LFXO, false, false);
    }
    else //initilaise lfxo for general energy modes
    {
    	OSCILLATOR_SETUP(LFXO_c);
    	CMU_OscillatorEnable(cmuOsc_ULFRCO, false, false);
    }
    DUTY_CYCLE_SETUP(); //adjust comp0 and comp1 values
    ACMP_SETUP(); //set up acmp
    GPIO_SETUP(); //set up gpio pins
    ADC_SETUP(); //sets up ADC with required attributes
    DMA_SETUP();
    LED_OFF(0);
    LED_OFF(0);//default led to switch off to conserve energy

    LETIMER_Enable(LETIMER0, false); //disable timer
    while (LETIMER0->SYNCBUSY & LETIMER_SYNCBUSY_CMD); //wait for it to sync with new clock
    LETIMER_SETUP(); //set letimer features and enable running
            /* Infinite loop */


  while (1) {
	  sleep();
  }
}

