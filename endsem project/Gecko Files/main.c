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
#include "em_leuart.h"
#include "em_common.h"
#include "dmactrl.h"
#include "sleep.h"
#include "circbuff.h"
#include "em_i2c.h"

#define ENERGY_MIN 3			// Macro to define minimum energy mode
#define INT_CLR_ALL 0x11111111	// Macro to clear all interrupts					//Macro to define the ADC sleep mode

#define LFXO_COUNT  32768		//ideal Count for 1 sec for LFXO
#define ULFRCO_COUNT  1000		//ideal count for 1 sec for ULFRCO

#define ONE_CYCLE 5.5			//one time cycle is 1.75 seconds
#define ON_TIME 0.004     	//LED shld be on for 4 milliseconds
#define LETIMER_MAX_COUNT 65535 //Max count of Letimer

#define LED0_PORT gpioPortE		//Macro to define LED0 PORT
#define LED0_PIN 2				//Macro to define LED0 PIN
#define LED1_PORT gpioPortE		//Macro to define LED1 PORT
#define LED1_PIN 3				//Macro to define LED1 PIN

#define CALIBRATION 1 					//Macro for calibration

#define WATER_LEVEL_INPUT_PORT gpioPortD
#define WATER_LEVEL_INPUT_PIN 3

#define TMP36_PORT gpioPortD
#define TMP36_PIN 7

#define PIR_INPUT_PORT gpioPortD
#define PIR_INPUT_PIN 6

#define LEUART_LOC    0
#define LEUART_TXPORT 	   gpioPortD        //Defining macro for LEUART transmission port
#define LEUART_TXPIN       4                //Defining macro for LEUART transmission pin
#define LEUART_RXPORT      gpioPortD        //Defining macro for LEUART reception port
#define LEUART_RXPIN       5                //Defining macro for LEUART reception pin
#define LEUART_BAUD 9600					//Defining macro to set the LEUART baud rate
#define LEUART_PARITY leuartNoParity 		//Defining macro to set the LEUART parity
#define LEUART_refFreq 0 					//Defining macro to select the reference clock
#define LEUART_DISABLE leuartDisable		//Defining macro to disable LEUART
#define LEUART_DATA_BIT leuartDatabits8		//Defining macro to set the LEUART data
#define LEUART_STOP_BIT	leuartStopbits1 	//Defining macro to set the LEUART stop
#define LEUART_ENABLE 1
#define LEUART_EM 2
#define Buffer_Enable true

#define DMA_REQD
#define DMA_CHANNEL_LEUART 0   	      	//DMA channel 0
#define LEUART0_DMA_ARBITRATE dmaArbitrate1 //Macro to set arbitration to 1024
#define LEUART0_DESTINATION_INCREMENT dmaDataIncNone//Macro for destination increment
#define LEUART0_SOURCE_INCREMENT dmaDataInc1   //Macro for source increment
#define LEUART0_DMA_TRANSFER_SIZE dmaDataSize1	   //Macro for hwo many bytes DMA should transfer

#define ADC_MODE adcStartSingle			// Macro to define ADC scan mode
#define ADC_PRESCALER 49				// Macro to define ADC Prescaler
#define ADC_ACQ_TIME adcAcqTime2		//Macro to set the acquisition time clock cycles
#define ADC_INPUT_TMP36 adcSingleInputCh7 	//Macro to define ADC input
#define ADC_REFERENCE_VOLTAGE adcRef1V25 //Macro to define ADC reference voltage
#define ADC_RESOLUTION adcRes12Bit		//Macro to define ADC Resolution
#define ADC_EM 1						//Macro to define the ADC sleep mode

#define DMA_CHANNEL_ADC 0   	      	//DMA channel 0
#define ADC0_DMA_ARBITRATE dmaArbitrate1024 //Macro to set arbitration to 1024
#define ADC0_DESTINATION_INCREMENT dmaDataInc2 //Macro for destination increment
#define ADC0_SOURCE_INCREMENT dmaDataIncNone   //Macro for source increment
#define ADC_DMA_TRANSFER_SIZE dmaDataSize2	   //Macro for hwo many bytes DMA should transfer

#define LOW_TEMP_LIMIT_C 5				//Macro to define lower temperature limit
#define HIGH_TEMP_LIMIT_C 45			//Macro to define higher temperature limit

#define TOTAL_CONVERSION 750			//Macro to define total number of conversions required

#define LOCATION_ROUTE 0				//Macro to define ROUTE FOR I2C1
#define SLAVE_ADDR	0x39				//Macro to define SLAVE ADDRESS
#define I2C_WRITE	0						//Macro to define WRITE
#define I2C_READ	1						//Macro to define READ

#define TSL2651_PWR_PORT gpioPortD		//Macro to define POWER PORT FOR SENSOR TSL2651
#define TSL2651_PWR_PIN 0				//Macro to define POWER PIN FOR SENSOR TSL2651
#define TSL2651_INT_PORT gpioPortD		//Macro to define INTERRUPT PORT FOR SENSOR TSL2651
#define TSL2651_INT_PIN 1				//Macro to define INTERRUPT PIN FOR SENSOR TSL2651

#define I2C1_SCL_PORT gpioPortC			//Macro to define SCL port for I2C 1
#define I2C1_SDA_PORT gpioPortC			//Macro to define SDA port for I2C 1
#define I2C1_SCL_PIN 5					//Macro to define SCL pin for I2C 1
#define I2C1_SDA_PIN 4					//Macro to define SDA pin for I2C 1

#define THRESHOLD_LOW_LOW	0x82		//Macro to set the register for low threshold [low byte]
#define THRESHOLD_LOW_HIGH	0x83		//Macro to set the register for low threshold [high byte]
#define THRESHOLD_HIGH_LOW	0x84    	//Macro to set the register for high threshold [low byte]
#define THRESHOLD_HIGH_HIGH	0x85		//Macro to set the register for high threshold [high byte]
#define INTERRUPT 0x86					//Macro to set persistence register
#define TIMING_REG	0x81					//Macro to set the timing register
#define CONTROL	0x80					//Macro to define the control register

//#define PASSIVE_LIGHT_SENSOR

#define SENSOR_THRESHOLD_LOW	0x000f	 //Macro to set the low threshold
#define SENSOR_THRESHOLD_HIGH	0x0800   //Macro to sete the high threshold
#define SENSOR_ADC0_LOW	0xAC			 //Macro to set the ADC0 data low register
#define SENSOR_ADC0_HIGH 0xAD			 //Macro to set the ADC0 data high register
#define	PERSISTENCE	0x04				 //Macro to set persistence to 4
#define INTR	0x01					 //Macro to select interrupt to level mode
#define SENSOR_THRESHOLD_LOW_LOW	0x0f //Macro to set the low threshold[low byte]
#define SENSOR_THRESHOLD_LOW_HIGH	0x00 //Macro to set the low threshold[high byte]
#define SENSOR_THRESHOLD_HIGH_LOW	0x00 //Macro to set the high threshold[low byte]
#define SENSOR_THRESHOLD_HIGH_HIGH	0x08 //Macro to set the high threshold[high byte]
#define INTEGRATION_TIME	0x01		 //Macro to set integration time to 101 ms
#define GAIN	0x00					 //Macro to set LOW GAIN
#define POWER_UP	0x03				 //Macro to give the power to the sensor
#define POWER_DOWN	0x00				 //Macro to disable the power
#define INTERRUPT_CLEAR	0xC6			//Macro to clear the interrupts in control
#define INTERRUPT_DISABLE	0x00		//Macro to disable interrupts
#define TIMING_REG_VAL	GAIN<<4|INTEGRATION_TIME //Macro to give the value to TIMING register
#define	INTERRUPT_REG_VAL	INTR<<4|PERSISTENCE	 //Macro to give the value to INTERRUPT register
#define I2C1_EM 1						//Macro to define the I2C1 sleep mode

#define MIC_PORT gpioPortD
#define MIC_PIN 3
#define ADC_INPUT_MIC adcSingleInputCh3

#define ACMP_CHANNEL acmpChannel6		//Macro to define ACMP0 channel
#define ACMP_REF acmpChannelVDD			//Macro to define ACMP0 reference

#define FSR_PORT gpioPortD
#define FSR_PIN 2
#define ADC_INPUT_FSR adcSingleInputCh2



#if ( ( DMA_CHAN_COUNT > 0 ) && ( DMA_CHAN_COUNT <= 4 ) )
#define DMACTRL_CH_CNT      4
#define DMACTRL_ALIGNMENT   128

#elif ( ( DMA_CHAN_COUNT > 4 ) && ( DMA_CHAN_COUNT <= 8 ) )
#define DMACTRL_CH_CNT      8
#define DMACTRL_ALIGNMENT   256

#elif ( ( DMA_CHAN_COUNT > 8 ) && ( DMA_CHAN_COUNT <= 12 ) )
#define DMACTRL_CH_CNT      16
#define DMACTRL_ALIGNMENT   256

#else
#error "Unsupported DMA channel count (dmactrl.c)."
#endif

/** DMA control block array, requires proper alignment. */
SL_ALIGN(DMACTRL_ALIGNMENT)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMACTRL_CH_CNT * 2] SL_ATTRIBUTE_ALIGN(DMACTRL_ALIGNMENT);

volatile uint16_t ramBufferAdcData[TOTAL_CONVERSION];		//Array to store the ADC values

uint8_t array[20] = {0};
uint8_t *ptr;

cirbuff_t circular_buffer_structure;
cirbuff_t *cbuff;


/*********************GLOBAL VARIABLES********************************************/
uint32_t total_count_LFXO = 0;			//variable to store TIMER count for LFXO
uint32_t total_count_ULFRCO = 0;		//variable to store TIMER count for ULFCRO
float calib_ratio=0;					//variable for calibration ratio
uint32_t on_period_EM3;					//variable for on period for EM3
uint32_t period_EM3;					//variable for period for EM3
uint32_t period;						//variable for period
uint32_t on_period;						//variable for on period
unsigned int letimer_interrupt_flag;	//variable to store the letimer interrupt
unsigned int acmp_value;				//variable to store ACMP value
uint8_t letimer_prescaler;				//variable to store calculate prescaler value
uint32_t frequency_value =0;			//variable to store frequency of clock
float temp_celsius = 0;					//variable to store the temperature
uint8_t pir_value;
int water_level_value;
uint8_t interrupt_cnt=0;
unsigned int leuart_interrupt_flag;
uint8_t recd_data[100] = {0};
uint8_t recd_data_index = 0;
/* ****************************************************
 * Function to set up the clock tree
 *
 * Input variable:  CMU_Osc_TypeDef osc
 * Local variable: none
 * Global variable: none
 * Return type: none
 * macro: ENERGY_MIN
 *******************************************************/
void CLOCK_SETUP(CMU_Osc_TypeDef osc)
{

	CMU_OscillatorEnable(osc, true, true);  	    		//Enabling low frequeuncy
	CMU_ClockSelectSet(cmuClock_LFA, osc);					//Selecting low frequency clock for LFA tree
	CMU_ClockEnable(cmuClock_CORELE, true);					//Enabling Low frequency Clock tree
	CMU_ClockEnable(cmuClock_LETIMER0, true);				//Enabling LFA clock tree to LETIMER0
	CMU_ClockEnable(cmuClock_GPIO, true);					//Enabling Clock to GPIO
	CMU_ClockEnable(cmuClock_ACMP0, true);					//Enabling Clock to ACMP
	CMU_ClockEnable(cmuClock_ADC0, true);					//Enabling Clock to ADC0
	CMU_ClockEnable(cmuClock_DMA, true);					//Enabling Clock to DMA
}

/* ****************************************************
 * Function to calculate PRESCALAR
 *
 * Input variable: none
 * Local variable: none
 * Global variable: period, on_period, letimer_prescaler
 * Return type: none
 * macro: ONE_CYCLE,LFXO_COUNT, LETIMER_MAX_COUNT,ON_TIME
 ********************************************************/
void prescaler_set(void)
{
	letimer_prescaler = 0;
	period = (ONE_CYCLE * LFXO_COUNT);						//calculate desired period
	on_period = (ON_TIME* LFXO_COUNT);						//calculate on_time
	while(period > LETIMER_MAX_COUNT)						//check if prescaler required
	{
		letimer_prescaler++;								//increment prescaler count
		period/=2;											//divide desired period by 2
		on_period/=2;										//divide on period by 2
	}
}

/* ****************************************************
 * Function to set up TIMER0
 *
 * Input variable:  none
 * Local variable: none
 * Global variable: none
 * Return type: none
 * macro: none
 *******************************************************/
void TIMER0_SETUP()
{
	CMU_ClockEnable(cmuClock_HFPER,true);				// Enabling High Frequency[14MHz] clock
	CMU_ClockEnable(cmuClock_TIMER0,true);				// Enabling clock to TIMER0

	/* Creating struct to set the TIMER0 parameters */
	TIMER_Init_TypeDef timer0Init =
	  {
	    .enable     = true, 							//Start counting when init is completed
	    .debugRun   = false, 							//Counter stops counting during debug
	    .prescale   = timerPrescale1,					//No prescaler required
	    .mode       = timerModeUp, 						//Timer to be used in up counter mode
	    .dmaClrAct  = false,							// No DMA clear request
	    .quadModeX4 = false, 							//No quad mode used
		.clkSel     = timerClkSelHFPerClk, 				//Select clock to High frequency
		.fallAction = timerInputActionNone, 			//No action on falling input edge
	    .riseAction = timerInputActionNone, 			//No action on rising input edge
	    .oneShot    = false, 							//Continuous counting.
	    .sync       = true, 							//In synchronization with other timers
	  };
	TIMER_Init(TIMER0, &timer0Init);					//Initializing TIMER0
}

/* ****************************************************
 * Function to set up TIMER1 in cascade mode
 *
 * Input variable:  none
 * Local variable: none
 * Global variable: none
 * Return type: none
 * macro: none
 *******************************************************/
void TIMER1_SETUP()
{
	CMU_ClockEnable(cmuClock_HFPER,true);				// Enabling High Frequency[14MHz] clock
	CMU_ClockEnable(cmuClock_TIMER1,true);				// Enabling clock to TIMER1

	/* Creating struct to set the TIMER1 parameters */
	TIMER_Init_TypeDef timer1Init =
	  {
	    .enable     = true, 							//Start counting when init is completed
	    .debugRun   = false, 							//Counter stops counting during debug
	    .prescale   = timerPrescale1,					//No prescaler required
	    .mode       = timerModeUp, 						//Timer to be used in up counter mode
	    .dmaClrAct  = false,							//No DMA clear request
	    .quadModeX4 = false, 							//No quad mode used
		.clkSel     = timerClkSelCascade, 				//Select clock to Cascade mode. Clock gets triggered on UF/OF of TIMER0
		.fallAction = timerInputActionNone, 			//No action on falling input edge
	    .riseAction = timerInputActionNone, 			//No action on rising input edge
	    .oneShot    = false, 							//Continuous counting.
	    .sync       = true, 							//In synchronization with other timers
	  };

	TIMER_Init(TIMER1, &timer1Init);					//Initializing TIMER1
}


/* ****************************************************
 * Function to set up LETIMER0 for calibration
 *
 * Input variable:  count
 * Local variable: none
 * Global variable: none
 * Return type: none
 * macro: none
 *******************************************************/
void LETIMER0_CALIB_SETUP(uint32_t count)
{
	/*structure defining the configurations of LETIMER0*/
	const LETIMER_Init_TypeDef letimer_calib_Init =
	{
	  .enable         = true,                   // Start counting when init completed.
	  .debugRun       = false,                  // Counter shall not keep running during debug halt.
	  .rtcComp0Enable = false,                  // Don't start counting on RTC COMP0 match.
	  .rtcComp1Enable = false,                  // Don't start counting on RTC COMP1 match.
	  .comp0Top       = true,                   // Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP
	  .bufTop         = false,                  // Don't load COMP1 into COMP0 when REP0 reaches 0.
	  .out0Pol        = 0,                      // Idle value for output 0.
	  .out1Pol        = 0,                      // Idle value for output 1.
	  .ufoa0          = letimerUFOANone,        // No changes on underflow
	  .ufoa1          = letimerUFOANone,        // No changes on underflow
	  .repMode        = letimerRepeatOneshot    // will run for only one time
	};
	 LETIMER_Init(LETIMER0, &letimer_calib_Init);	// Initialize LETIMER0
	 LETIMER0->CNT = count;
	 LETIMER_Enable(LETIMER0, true);
}

/***********************************************************************************
 * Routine for calibration
 *
 * Input variable:  none
 * Global variable: total_count_LFXO,total_count_ULFRCO,calib_ratio
 * Local variable:  none
 * Return type: none
 * Macros: ULFRCO_COUNT
 *
 ***********************************************************************************/
void LETIMER_CALIBRATION()
{
	while((LETIMER0->CNT)!=0);								//wait till LETIMER count reaches 0
	LETIMER_IntClear(LETIMER0, LETIMER_IFC_UF);				//clear interrupts
	total_count_LFXO = TIMER0->CNT;							//get the TIMER0 count
	total_count_LFXO |= (TIMER1->CNT << 16);				//Get the TIMER1 count and then calculate total count
	TIMER0->CNT =0;											//Reset the TIMER0 count
	TIMER1->CNT =0;											//Reset the TIMER1 count
	LETIMER_Enable(LETIMER0,false);							//Disable LETIMER0
	CLOCK_SETUP(cmuSelect_ULFRCO);							//Switch the clock to ULFRCO
	LETIMER0_CALIB_SETUP(ULFRCO_COUNT);						//Setup the count in LETIMER0 to 1000
	while((LETIMER0->CNT)!=0);								//wait till LETIMER count reaches 0
	LETIMER_IntClear(LETIMER0, LETIMER_IFC_UF);				//clear interrupts
	total_count_ULFRCO = TIMER0->CNT;						//get the TIMER0 count
	total_count_ULFRCO |= (TIMER1->CNT << 16);				//Get the TIMER1 count and then calculate total count
	calib_ratio = ((float)(total_count_LFXO))/((float)(total_count_ULFRCO));		//calculate the calibration ratio
	CMU_ClockEnable(cmuClock_TIMER0,false);								//Disable TIMER0
	CMU_ClockEnable(cmuClock_TIMER1,false);								//Disable TIMER1
}

/******************************************************
 * Function is used to setup the onboard LEDS.
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: LED0_PORT, LED0_PIN, LED1_PORT, LED1_PIN
 *
 *******************************************************/
void GPIO_LED_SETUP()
{
	GPIO_PinModeSet(LED0_PORT, LED0_PIN, gpioModePushPull, 0);		// Configure Pe2 as push pull and output*/
	GPIO_DriveModeSet(LED0_PORT, gpioDriveModeStandard);	// Standard drive mode

	GPIO_PinModeSet(LED1_PORT, LED1_PIN, gpioModePushPull, 0);		// Configure Pe2 as push pull and output*/
	GPIO_DriveModeSet(LED1_PORT, gpioDriveModeStandard);	// Standard drive mode
}

/* **************************************************************************
 * Function to turn on the LED
 *
 * Input variable: led_on
 * Local variable: none
 * Global variable: none
 * Return type: none
 * MACRO:  LED0_PORT,LED1_PORT, LED1_PIN, LED1_PIN
 *
 ***************************************************************************/
void LED_ON(uint8_t led_no)
{
	if(led_no == 0)
	{
		GPIO_PinOutSet(LED0_PORT,LED0_PIN);						//Glow LED0
	}
	else
	{
		GPIO_PinOutSet(LED1_PORT,LED1_PIN);						//Glow LED0
	}
}

/* **************************************************************************
 * Function to turn on the LED
 *
 * Input variable: led_on
 * Local variable: none
 * Global variable: none
 * Return type: none
 * MACRO:  LED0_PORT,LED1_PORT, LED1_PIN, LED1_PIN
 *
 ***************************************************************************/
void LED_OFF(uint8_t led_no)
{
	if(led_no == 0)
	{
		GPIO_PinOutClear(LED0_PORT,LED0_PIN);						//Glow LED0
	}
	else
	{
		GPIO_PinOutClear(LED1_PORT,LED1_PIN);						//Glow LED0
	}
}


/*****************************************************************************************************
 * Routine to setup LEUART
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: LEUART_TXPORT,LEUART_RXPORT,LEUART_TXPIN,LEUART_RXPIN,LEUART_LOC
 *
 * COPYRIGHT: The below used routine has been taken from Silicon Labs
 ***********************************************************************************************************************/

void LEUART_GPIO_SETUP()
{
	GPIO_PinModeSet(LEUART_TXPORT, LEUART_TXPIN, gpioModePushPull, 1);
	GPIO_PinModeSet(LEUART_RXPORT, LEUART_RXPIN, gpioModeInput, 1);
}

/*****************************************************************************************************
 * Routine to setup LEUART clock
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: none
 *
 * COPYRIGHT: The below used routine has been taken from Silicon Labs
 ***********************************************************************************************************************/


void LEUART_CLOCK_SETUP()
{
	 CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);	 //Select LFB clock tree for LEAURT0
	 CMU_ClockEnable(cmuClock_LEUART0, true);			 //Enabling clock to LEAURT0

	 CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);	//No prescaler for current clock
}

/*****************************************************************************************************
 * Routine to setup LEUART
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros:LEUART_BAUD,LEUART_PARITY,LEUART_refFreq,LEUART_DISABLE,LEUART_DATA_BIT,LEUART_STOP_BIT
 *
 * COPYRIGHT: The below used routine has been taken from Silicon Labs
 ***********************************************************************************************************************/

void LEUART_SETUP(void)
{

  LEUART_GPIO_SETUP();

  LEUART_Init_TypeDef leuart_init =
  {
	.enable = LEUART_DISABLE,		//Disable the leuart on init function
	.refFreq = LEUART_refFreq,		//using current reference clock
	.baudrate = LEUART_BAUD,		//setting baudrate to 9600
	.parity = LEUART_PARITY ,		//No parity check required
	.databits = LEUART_DATA_BIT,		//Setting data bits to 8
	.stopbits =  LEUART_STOP_BIT 	//Setting stop bit to 1
  };

  LEUART_CLOCK_SETUP();				//Setting the LEUART clock config

  LEUART_Reset(LEUART0);

  LEUART_Init(LEUART0, &leuart_init);				//Initialising the LEAURT0

  /* Enable pins at default location */
  LEUART0->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | LEUART_LOC;

  LEUART0->IFC |= INT_CLR_ALL;

 // LEUART_IntEnable(LEUART0, LEUART_IEN_TXC);
  LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV);
  LEUART_TxDmaInEM2Enable(LEUART0, true);

  /* Finally enable it */
  LEUART_Enable(LEUART0, leuartEnable);

  NVIC_EnableIRQ(LEUART0_IRQn);
}

void LEUART0_IRQHandler(void)
{
	uint8_t removed_item;
	uint8_t recd;
	INT_Disable();
	leuart_interrupt_flag = LEUART_IntGet(LEUART0);
	if(leuart_interrupt_flag & LEUART_IF_TXC)
	{
		LEUART_IntClear(LEUART0,leuart_interrupt_flag);
		if(cbuff->no_of_items)
			{
				remove_item(cbuff, &removed_item);
				LEUART0->TXDATA = removed_item;
			}
			else
			{
				unblockSleepMode(LEUART_EM);
				blockSleepMode(ENERGY_MIN);
			}
	}
	else if (leuart_interrupt_flag & LEUART_IF_RXDATAV)
	{
		LEUART_IntClear(LEUART0,leuart_interrupt_flag);
		recd = LEUART0->RXDATA;
		if(recd != 0xD8)
		{
			if(recd == 1)
			{
				LED_ON(1);
			}
			else if(recd == 0)
			{
				LED_OFF(1);
			}
		}
		//if(recd_data_index < 100)
//		{
//			recd_data[recd_data_index] = LEUART0->RXDATA;
//			recd_data_index++;
//		}

	}
	INT_Enable();
}


/************************************************************************************
 * Function is used to setup the TMP36 GPIO
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: LIGHT_SENSOR_PORT, LIGHT_EXCITE_PORT, LIGHT_SENSOR_PIN, LIGHT_EXCITE_PIN
 *
 ***********************************************************************************/
void TMP36_GPIO_SETUP()
{
	GPIO_PinModeSet(TMP36_PORT, TMP36_PIN, gpioModeInput, 1);	//Configure Excitation for sensor as pushpull
	GPIO_DriveModeSet(TMP36_PORT, gpioDriveModeStandard);	// Standard drive mode
}


/* **************************************************************************
 * Function is used to calculate temperature in degree celsius
 *
 * Input variable:  adcSample
 * Local variable: none
 * Global variable:none
 * Return type: float temp
 * MACRO: none
 *
 *COPYRIGHT: The below used routine has been taken from Silicon Labs
 ***************************************************************************/
float convertToCelsius(int32_t adcSample)
{
	float temp;
	float voltage_value;
	voltage_value = (adcSample*1250/4096);
	temp = (voltage_value-500)/10;
	return temp;
}

/************************************************************************************
 * Function is used to setup the ADC
 *
 * Input variable:  none
 * Global variable: frequency_value
 * Local variable:  none
 * Return type: none
 * Macros:ADC_PRESCALER,ADC_ACQ_TIME,ADC_INPUT,ADC_REFERENCE_VOLTAGE,ADC_RESOLUTION
 *
 ***********************************************************************************/
void ADC_TMP36_SETUP ()
{
		uint8_t time_base = 0;
		frequency_value = CMU_ClockFreqGet(cmuClock_HFPER);
		time_base = ADC_TimebaseCalc(frequency_value);
		const ADC_Init_TypeDef adc0_init =
		{
			.ovsRateSel = _ADC_CTRL_PRESC_DEFAULT,			//Set oversampling to 0
			.lpfMode  = adcLPFilterBypass,					//No filter
			.warmUpMode = adcWarmupNormal,					// Normal Warm-up
			.prescale = ADC_PRESCALER,						// Prescaler of 50 required for Acquisition time of 7.14 useconds
			.tailgate = false,								//No tailgating required
			.timebase = time_base,							//Setting time base for warmup
		};

		ADC_Init(ADC0, &adc0_init);							//init ADC0

		const ADC_InitSingle_TypeDef adc0_single_init =
		{
				.acqTime = ADC_ACQ_TIME,						//Setting Acquisition time of 7.14 usecs ie 2 clock cycles
				.diff = false,								// Single ended input mode
				.input = ADC_INPUT_TMP36,				//Temperature reference
				.leftAdjust = false,						//left adjust not required
				.prsEnable = false,							//prs enable not required
				.reference = ADC_REFERENCE_VOLTAGE,					//Reference Voltage of 1.25V
				.rep = true,								//repetition required
				.resolution = ADC_RESOLUTION,					//12 bit resolution
		};

		ADC_InitSingle(ADC0, &adc0_single_init);			//init ADC0 for Single Scan

		ADC0->IFC = INT_CLR_ALL;							//Clear all interrupts
		//ADC0->IEN = ADC_IEN_SINGLE;							//enable single interrupt
		//NVIC_EnableIRQ(ADC0_IRQn);							//enable NVIC
}


/* **************************************************************************
 * Function to read the ADC values and calculate the temperature in celsius
 *
 * Input variable: none
 * Local variable: none
 * Global variable: conversion_count
 * Return type: float temp_C
 * MACRO:  TOTAL_CONNVERSION
 *
 *  ***************************************************************************/
float TEMP_CALC()
{
	uint16_t conversion_count = 0;			//variable to store the no of ADC conversions
	blockSleepMode(ADC_EM);									//Energy Mode 1
	float temp_C = 0;
	conversion_count = 0;
	uint32_t sum = 0;
	uint32_t average_value = 0;
	ADC_Start(ADC0,ADC_MODE);					//Start ADC
	while(conversion_count != TOTAL_CONVERSION)			//Check for 750 conversions
	{
		while(!(ADC0->IF & ADC_IFS_SINGLE));
		sum += ADC0->SINGLEDATA;						//Add the newest read value to take the sum
		conversion_count++;								//increment conversion count
		ADC_IntClear(ADC0,ADC_IFC_SINGLE);
	}
	ADC0->CMD = ADC_CMD_SINGLESTOP;						//Disable ADC
	unblockSleepMode(ADC_EM);								//Come out of Energy Mode 1
	average_value = sum / TOTAL_CONVERSION;				//Take average of the ADC readings
	temp_C = convertToCelsius(average_value);			//Calculate temperature in Celsius
	return temp_C;
}

void MIC_GPIO_SETUP()
{
	GPIO_DriveModeSet(MIC_PORT, gpioDriveModeStandard);	// Standard drive mode
	GPIO_PinModeSet(MIC_PORT, MIC_PIN, gpioModeInput, 0);	// Configure Light sensor
}

void FSR_GPIO_SETUP()
{
	GPIO_DriveModeSet(FSR_PORT, gpioDriveModeStandard);	// Standard drive mode
	GPIO_PinModeSet(FSR_PORT, FSR_PIN, gpioModeInput, 0);	// Configure Light sensor
}

/************************************************************************************
 * Function is used to setup the ADC
 *
 * Input variable:  none
 * Global variable: frequency_value
 * Local variable:  none
 * Return type: none
 * Macros:ADC_PRESCALER,ADC_ACQ_TIME,ADC_INPUT,ADC_REFERENCE_VOLTAGE,ADC_RESOLUTION
 *
 ***********************************************************************************/
void ADC_MIC_SETUP ()
{
		uint8_t time_base = 0;
		frequency_value = CMU_ClockFreqGet(cmuClock_HFPER);
		time_base = ADC_TimebaseCalc(frequency_value);
		const ADC_Init_TypeDef adc0_mic_init =
		{
			.ovsRateSel = _ADC_CTRL_PRESC_DEFAULT,			//Set oversampling to 0
			.lpfMode  = adcLPFilterBypass,					//No filter
			.warmUpMode = adcWarmupNormal,					// Normal Warm-up
			.prescale = ADC_PRESCALER,						// Prescaler of 50 required for Acquisition time of 7.14 useconds
			.tailgate = false,								//No tailgating required
			.timebase = time_base,							//Setting time base for warmup
		};

		ADC_Init(ADC0, &adc0_mic_init);							//init ADC0

		const ADC_InitSingle_TypeDef adc0_single_mic_init =
		{
				.acqTime = ADC_ACQ_TIME,						//Setting Acquisition time of 7.14 usecs ie 2 clock cycles
				.diff = false,								// Single ended input mode
				.input = ADC_INPUT_MIC,				//Temperature reference
				.leftAdjust = false,						//left adjust not required
				.prsEnable = false,							//prs enable not required
				.reference = ADC_REFERENCE_VOLTAGE,					//Reference Voltage of 1.25V
				.rep = true,								//repetition required
				.resolution = ADC_RESOLUTION,					//12 bit resolution
		};

		ADC_InitSingle(ADC0, &adc0_single_mic_init);			//init ADC0 for Single Scan

		ADC0->IFC = INT_CLR_ALL;							//Clear all interrupts
		ADC0->IEN = ADC_IEN_SINGLE;							//enable single interrupt
		///NVIC_EnableIRQ(ADC0_IRQn);							//enable NVIC
}

void ADC_FSR_SETUP ()
{
		uint8_t time_base = 0;
		frequency_value = CMU_ClockFreqGet(cmuClock_HFPER);
		time_base = ADC_TimebaseCalc(frequency_value);
		const ADC_Init_TypeDef adc0_fsr_init =
		{
			.ovsRateSel = _ADC_CTRL_PRESC_DEFAULT,			//Set oversampling to 0
			.lpfMode  = adcLPFilterBypass,					//No filter
			.warmUpMode = adcWarmupNormal,					// Normal Warm-up
			.prescale = ADC_PRESCALER,						// Prescaler of 50 required for Acquisition time of 7.14 useconds
			.tailgate = false,								//No tailgating required
			.timebase = time_base,							//Setting time base for warmup
		};

		ADC_Init(ADC0, &adc0_fsr_init);							//init ADC0

		const ADC_InitSingle_TypeDef adc0_single_fsr_init =
		{
				.acqTime = ADC_ACQ_TIME,						//Setting Acquisition time of 7.14 usecs ie 2 clock cycles
				.diff = false,								// Single ended input mode
				.input = ADC_INPUT_FSR,				//Temperature reference
				.leftAdjust = false,						//left adjust not required
				.prsEnable = false,							//prs enable not required
				.reference = ADC_REFERENCE_VOLTAGE,					//Reference Voltage of 1.25V
				.rep = true,								//repetition required
				.resolution = ADC_RESOLUTION,					//12 bit resolution
		};

		ADC_InitSingle(ADC0, &adc0_single_fsr_init);			//init ADC0 for Single Scan

		ADC0->IFC = INT_CLR_ALL;							//Clear all interrupts
		ADC0->IEN = ADC_IEN_SINGLE;							//enable single interrupt
		///NVIC_EnableIRQ(ADC0_IRQn);							//enable NVIC
}
/* **************************************************************************
 * Function to read the ADC values and calculate the temperature in celsius
 *
 * Input variable: none
 * Local variable: none
 * Global variable: conversion_count
 * Return type: float temp_C
 * MACRO:  TOTAL_CONNVERSION
 *
 *  ***************************************************************************/
void MIC_CALC()
{
	uint8_t mic_flag = 0;
	uint16_t conversion_count = 0;			//variable to store the no of ADC conversions
	float voltage_value;
	blockSleepMode(ADC_EM);									//Energy Mode
	conversion_count = 0;
	uint32_t sum = 0;
	uint32_t average_value = 0;
	ADC_Start(ADC0,ADC_MODE);					//Start ADC
	while(conversion_count != TOTAL_CONVERSION)			//Check for 750 conversions
	{
		while(!(ADC0->IF & ADC_IFS_SINGLE));
		sum += ADC0->SINGLEDATA;						//Add the newest read value to take the sum
		conversion_count++;								//increment conversion count
		ADC_IntClear(ADC0,ADC_IFC_SINGLE);
	}
	ADC0->CMD = ADC_CMD_SINGLESTOP;						//Disable ADC
	ADC_Reset(ADC0);
	unblockSleepMode(ADC_EM);								//Come out of Energy Mode 1
	average_value = sum / TOTAL_CONVERSION;				//Take average of the ADC readings
	voltage_value = 850;
	//voltage_value = (average_value*1250/4096);
	if (voltage_value < 900)
	{
		mic_flag = 1;
		add_item(cbuff,0xE);
	}
	else
	{
		mic_flag = 0;
		add_item(cbuff,0xF);
	}
}

void FSR_CALC()
{
	uint8_t fsr_flag = 0;
	uint16_t conversion_count = 0;			//variable to store the no of ADC conversions
	float voltage_value;
	blockSleepMode(ADC_EM);									//Energy Mode
	conversion_count = 0;
	uint32_t sum = 0;
	uint32_t average_value = 0;
	ADC_Start(ADC0,ADC_MODE);					//Start ADC
	while(conversion_count != TOTAL_CONVERSION)			//Check for 750 conversions
	{
		while(!(ADC0->IF & ADC_IFS_SINGLE));
		sum += ADC0->SINGLEDATA;						//Add the newest read value to take the sum
		conversion_count++;								//increment conversion count
		ADC_IntClear(ADC0,ADC_IFC_SINGLE);
	}
	ADC0->CMD = ADC_CMD_SINGLESTOP;						//Disable ADC
	ADC_Reset(ADC0);
	unblockSleepMode(ADC_EM);								//Come out of Energy Mode 1
	average_value = sum / TOTAL_CONVERSION;				//Take average of the ADC readings
	voltage_value = 950;
	//voltage_value = (average_value*1250/4096);
	if (voltage_value < 900)
	{
		fsr_flag = 1;
		add_item(cbuff,0xE);
	}
	else
	{
		fsr_flag = 0;
		add_item(cbuff,0xF);
	}
}







/*********************************************
 * Call Back function to close off the DMA operation
 *
 * Input variable:  channel , primary, user pointer
 * Global variable: ramBufferAdcData
 * Return type: none
 * MACROS: TOTAL_CONVERSION,HIGH_TEMP_LIMIT_C,LOW_TEMP_LIMIT_C
 *
 *********************************************/
void DMA_ADC0_CB(unsigned int channel, bool primary, void *user)
{
    int32_t sum = 0;
    int32_t average;
    float temp_C = 0;
    int8_t temp_c_dec = 0;
    int8_t temp_c_int = 0;
    int8_t neg_temp_flag = 0;
    INT_Disable();									//Disable interrupts

    DMA_IntClear(DMA_IFC_CH0DONE);					//clear the DMA interrupts

    ADC0->CMD = ADC_CMD_SINGLESTOP;					//Disable ADC
    ADC_Reset(ADC0);
    unblockSleepMode(ADC_EM);

    for (int i=0; i < TOTAL_CONVERSION; i++)		//Add all the 750 samples
    {
            sum += ramBufferAdcData[i];
    }
    average = sum / TOTAL_CONVERSION;				//Take average of the samples

    temp_C = convertToCelsius(average);				//Calculate the temperature in celsius

    if (temp_C < 0)
    {
    	temp_C = (-1) * temp_C;
    	neg_temp_flag = 1;
    	add_item(cbuff,neg_temp_flag);
    }
    else
    {
    	neg_temp_flag = 0;
    	add_item(cbuff,neg_temp_flag);
    }
    temp_c_int = temp_C;
    temp_c_dec = (temp_C*10) - temp_c_int*10;
    add_item(cbuff, temp_c_int);
    add_item(cbuff, temp_c_dec);
    blockSleepMode(LEUART_EM);
    //LEUART0->IFS = LEUART_IFS_TXC;
    INT_Enable();										//Re-enable the interrupts

}

DMA_CB_TypeDef adc0_cb =								// DMA callback structure
{
		.cbFunc = DMA_ADC0_CB,
		.userPtr = NULL,
		.primary = true
};


/*********************************************
 * Individual call to setup up the DMA Channel for the ADC0
 *
 * Input variable:  none
 * Global variable: ramBufferAdcData
 * Return type: none
 * MACROS: ADC0_DMA_ARBITRATE,ADC0_DESTINATION_INCREMENT,ADC_DMA_TRANSFER_SIZE,ADC0_SOURCE_INCREMENT
 * 			DMA_CHANNEL_ADC
 *
 *********************************************/
void DMA_ADC0_Setup(void)
{
    DMA_CfgDescr_TypeDef DMA_ADC0_cfg =
    {
    	.arbRate = dmaArbitrate1,					//Arbitrate factor set to 1024 cycles
		.dstInc = ADC0_DESTINATION_INCREMENT,			//Destination increment set to 2 bytes
		.hprot = 0,										//no protection
		.size = ADC_DMA_TRANSFER_SIZE,					//DMA transfers of 2 bytes
		.srcInc = ADC0_SOURCE_INCREMENT					//Do not increment source
    };

    DMA_CfgDescr(DMA_CHANNEL_ADC, true, &DMA_ADC0_cfg);	//Configure the descriptor

    DMA_CfgChannel_TypeDef DMA_ADC0_channel =
    {
    		.cb = &adc0_cb,								//Address of callback routine
			.enableInt = true,							//enable the interrupts
			.highPri = true,							//assign high priority
			.select = DMAREQ_ADC0_SINGLE				//ADC0 for single scan mode
    };

    DMA_CfgChannel(DMA_CHANNEL_ADC, &DMA_ADC0_channel);	//Configure the DMA channel 0

    DMA_IntClear(DMA_IFC_CH0DONE);						//Clear the interrupts
    DMA_IntEnable(DMA_IEN_CH0DONE);						//Enable channel 0 interrupt
    DMA_ActivateBasic(DMA_CHANNEL_ADC,					//Activate Basic mode for DMA channel 0
                        true,							//use primary descriptor
                        false,							//Burst mode disabled
                        (void *)ramBufferAdcData,		//destination address
                        (void *)&(ADC0->SINGLEDATA),	//source address
                        TOTAL_CONVERSION - 1);			//No of ADC reads required
}

/***************************************************************************************
 * Main call to set up the DMA descriptor location and all of the individual DMA channels
 *
 * Input variable:  none
 * Global variable: ramBufferAdcData
 * Return type: none
 * MACROS: none
 *
 *********************************************/
void DMA_ADC_Setup(void)
{
    DMA_Init_TypeDef DMA_init =
    {
    		.controlBlock = dmaControlBlock,
			.hprot = 0
    };
    DMA_Init(&DMA_init);    /* initial DMA dma Control Block descriptor location */

    /* call all the individual DMA setup routines */
    DMA_ADC0_Setup();				//Setup the individual descriptors and channels
   // NVIC_EnableIRQ(DMA_IRQn);		//Enable the NVIC for DMA
}

void PIR_GPIO_SETUP()
{
	GPIO_PinModeSet(PIR_INPUT_PORT, PIR_INPUT_PIN, gpioModeInput, 0);
	GPIO_DriveModeSet(PIR_INPUT_PORT, gpioDriveModeStandard);
}

uint8_t PIR_GET_VALUE()
{
	uint8_t pir_value;
	pir_value = GPIO_PinInGet(PIR_INPUT_PORT, PIR_INPUT_PIN);
	return pir_value;
}

void WATER_LEVEL_GPIO_SETUP()
{
	GPIO_PinModeSet(WATER_LEVEL_INPUT_PORT,WATER_LEVEL_INPUT_PIN, gpioModeInputPull, 1);
	GPIO_DriveModeSet(WATER_LEVEL_INPUT_PORT, gpioDriveModeStandard);
}

uint8_t WATER_LEVEL_GET_VALUE()
{
	uint8_t water_level_value;
	water_level_value = GPIO_PinInGet(WATER_LEVEL_INPUT_PORT, WATER_LEVEL_INPUT_PIN);
	return water_level_value;
}

/******************************************************
 * Function is used to setup the GPIO for I2C1.
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: I2C1_SCL_PORT, I2C1_SCL_PIN, I2C1_SDA_PORT, I2C1_SDA_PIN
 *
 *******************************************************/
void I2C_GPIO_SETUP()
{
	GPIO_PinModeSet(I2C1_SCL_PORT,I2C1_SCL_PIN, gpioModeWiredAnd, 1);		// Configure SCL for I2C1 as push pull and output*/
	GPIO_DriveModeSet(I2C1_SCL_PORT, gpioDriveModeLowest);				// Standard drive mode

	GPIO_PinModeSet(I2C1_SDA_PORT,I2C1_SDA_PIN, gpioModeWiredAnd, 1);		// Configure SDA for I2C1 as push pull and output*/
	GPIO_DriveModeSet(I2C1_SDA_PORT, gpioDriveModeLowest);				// Standard drive mode
}

/******************************************************
 * Function is used to setup the GPIO for external light sensor.
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: TSL2651_PWR_PORT, TSL2651_PWR_PIN, TSL2651_INT_PORT, TSL2651_INT_PIN
 *
 *******************************************************/
void TSL2651_GPIO_SETUP()
{
	GPIO_PinModeSet(TSL2651_PWR_PORT,TSL2651_PWR_PIN, gpioModePushPull, 0);	// Configure PWR PRT for TSL2651 as push pull and output*/
	GPIO_DriveModeSet(TSL2651_PWR_PORT, gpioDriveModeLowest);				// Standard drive mode

	GPIO_PinModeSet(TSL2651_INT_PORT,TSL2651_INT_PIN, gpioModeInput, 0);	// Configure INT PIN for TSL2651 as push pull and output*/
	GPIO_DriveModeSet(TSL2651_INT_PORT, gpioDriveModeLowest);				// Standard drive mode
	GPIO_PinOutClear(TSL2651_PWR_PORT,TSL2651_PWR_PIN);

}


/******************************************************
 * Function is used to reset the I2C
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: none
 *
 *******************************************************/
void I2C_RESET()
{
	if (I2C1->STATE & I2C_STATE_BUSY) 						//check if busy, if busy then abort
	{
	      I2C1->CMD = I2C_CMD_ABORT;
	 }
}


/******************************************************
 * Function is used to enable SCL
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: I2C1_SCL_PORT,I2C1_SCL_PIN
 *
 *******************************************************/
void SCL_ENABLE()
{
	GPIO_PinOutSet(I2C1_SCL_PORT,I2C1_SCL_PIN);

    for (int i=0;i<9;i++) {
           GPIO_PinOutClear(I2C1_SCL_PORT, I2C1_SCL_PIN);
           GPIO_PinOutSet(I2C1_SCL_PORT, I2C1_SCL_PIN);
       }
}

/******************************************************
 * Function is used to enable SDA
 *
 * Input variable:  none
 * Global variable: none
 * Local variable:  none
 * Return type: none
 * Macros: I2C1_SDA_PORT,I2C1_SDA_PIN
 *
 *******************************************************/
void SDA_ENABLE()
{
	GPIO_PinOutSet(I2C1_SDA_PORT,I2C1_SDA_PIN);
}


/*********************************************
 * Routine to Setup the I2C1
 *
 * Input variable:  none
 * Global variable: none
 * Return type: none
 * Macros: LOCATION_ROUTE
 *********************************************/
void I2C1_SETUP()
{
	CMU_ClockEnable(cmuClock_I2C1, true);					//Enabling Clock to I2C1
	I2C_Init_TypeDef i2c1_Init =
	{
			.enable = false,								//do not enavle on init
			.clhr = i2cClockHLRStandard,					//4:4 ratio
			.freq = I2C_FREQ_STANDARD_MAX,					//92KHz
			.master = true,									//Enable as MASTER
			.refFreq = 0									//no std ref
	};

	I2C1->ROUTE |= ((LOCATION_ROUTE << _I2C_ROUTE_LOCATION_SHIFT)|I2C_ROUTE_SDAPEN|I2C_ROUTE_SCLPEN); 	//Set the route 						//Setting the route for I2C1
	I2C_Init(I2C1,&i2c1_Init);								//Initializing the I2C
	I2C_RESET();
	SCL_ENABLE();
	SDA_ENABLE();
	I2C_IntClear(I2C1,INT_CLR_ALL);							//Clear all the interrupts
	I2C_IntEnable(I2C1,I2C_IEN_ACK|I2C_IEN_NACK|I2C_IEN_MSTOP);			//Enable ACK and NACK interrupts
	I2C_Enable(I2C1, true);
}

/*********************************************
 * Routine to setup the write for I2C1
 *
 * Input variable: reg_addr, data
 * Global variable: none
 * Return type: none
 * Macros: SLAVE_ADDR, WRITE
 *********************************************/
void I2C1_Write(uint8_t reg_addr, uint8_t data)
{
	I2C_IntClear(I2C1,INT_CLR_ALL);					//Clear all the interrupts
	I2C1->TXDATA = ((SLAVE_ADDR << 1 ) | I2C_WRITE);//sending slave address
	I2C1->CMD |= I2C_CMD_START;					 //Send the START command
	I2C1->IFC = I2C_IFC_START;

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1->TXDATA = reg_addr;					 //Send the register address

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1->TXDATA = data;					 	//Send the data

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1 -> CMD |= I2C_CMD_STOP;				 //Send the STOP command
	while ((I2C1->IF & I2C_IF_MSTOP) ==  0);  	 //check for MSTOP
	I2C1->IFC=I2C_IFC_MSTOP;					 //Clear the MSTOP interrupt
}

/*********************************************
 * Routine to setup the read for I2C1
 *
 * Input variable: reg_addr
 * Global variable: none
 * Return type: uint16_t
 * Macros: SLAVE_ADDR, WRITE, READ
 *********************************************/
uint8_t I2C1_Read(uint8_t reg_addr)
{
	I2C_IntClear(I2C1,INT_CLR_ALL);
	uint8_t data;
	I2C1->TXDATA = ((SLAVE_ADDR << 1 ) | I2C_WRITE);  //sending slave address
	I2C1->CMD |= I2C_CMD_START;					 	  //Send the START command
	I2C1->IFC = I2C_IFC_START;						  //Clear the START flag

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1->TXDATA = reg_addr;					 //Send the register address

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	I2C1->CMD |= I2C_CMD_START;					 //Send the START command
	I2C1->TXDATA = ((SLAVE_ADDR << 1 ) | I2C_READ);  //sending slave address
	I2C1->IFC = I2C_IFC_START;

	while((I2C1->IF & I2C_IF_ACK) == 0);		 //Wait for ACK flag
	I2C1->IFC = I2C_IFC_ACK;					 //Clear the ACK flag

	while ((I2C1->IF & I2C_IF_RXDATAV) ==  0);  //if buffer is set, get data from receive buffer
	data =  I2C1->RXDATA;
	I2C1->CMD =I2C_CMD_NACK;

	I2C1 -> CMD |= I2C_CMD_STOP;				 //Send the STOP command
	while ((I2C1->IF & I2C_IF_MSTOP) ==  0);  	 //check for MSTOP
	I2C1->IFC=I2C_IFC_MSTOP;

	return data;
}

/*****************************************************************************************************
 * Routine to setup the TSL2651
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: THRESHOLD_LOW_LOW, SENSOR_THRESHOLD_LOW_LOW, THRESHOLD_LOW_HIGH,SENSOR_THRESHOLD_LOW_HIGH
 * 		   THRESHOLD_HIGH_LOW,SENSOR_THRESHOLD_HIGH_LOW,THRESHOLD_HIGH_HIGH,SENSOR_THRESHOLD_HIGH_HIGH,
 * 		   INTERRUPT,INTERRUPT_REG_VAL,TIMING_REG,TIMING_REG_VAL,CONTROL,POWER_UP
 *******************************************************************************************************/
void TSL2651_SETUP()
{
	I2C1_Write(CONTROL,POWER_UP);									//Powering up the device
	I2C1_Write(THRESHOLD_LOW_LOW,SENSOR_THRESHOLD_LOW_LOW);			//Setting the threshold in low low register
	I2C1_Write(THRESHOLD_LOW_HIGH,SENSOR_THRESHOLD_LOW_HIGH);		//Setting the threshold in low high register
	I2C1_Write(THRESHOLD_HIGH_LOW,SENSOR_THRESHOLD_HIGH_LOW);		//Setting the threshold in high low register
	I2C1_Write(THRESHOLD_HIGH_HIGH,SENSOR_THRESHOLD_HIGH_HIGH);		//Setting the threshold in high high register
	I2C1_Write(INTERRUPT,INTERRUPT_REG_VAL);						//Giving the persistence value of 4 and enbaling interrupt
	I2C1_Write(TIMING_REG,TIMING_REG_VAL);							//Giving integration time of 101 ms and a low gain
}

/*****************************************************************************************************
 * Routine to setup delay
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: none
 *******************************************************************************************************/

void delay(uint32_t delay_value)
{
	for(int i=0; i<delay_value ; i++);
}


/*****************************************************************************************************
 * Routine to power up the TSL2651
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: TSL2651_PWR_PORT,TSL2651_PWR_PIN
 ***************************************************************************************/
void TSL2651_POWERUP()
{
	GPIO_PinOutSet(TSL2651_PWR_PORT,TSL2651_PWR_PIN);				//Turnng on the device
	delay(10000);
	I2C1_SETUP();													//Setup the I2C1
	TSL2651_SETUP();												//Setup the TSL2651
	delay(10000);
}

/*****************************************************************************************************
 * Routine to power down the TSL2651
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: TSL2651_PWR_PORT,TSL2651_PWR_PIN,INTERRUPT_CLEAR,INTERRUPT_DISABLE
 ***************************************************************************************/
void TSL2651_POWERDOWN()
{
	I2C1_Write(INTERRUPT_CLEAR,INTERRUPT_DISABLE);						//Clear and the disable the interrupts
	I2C1_Write(CONTROL,POWER_DOWN);										//Disabling the power
	GPIO_PinOutClear(TSL2651_PWR_PORT, TSL2651_PWR_PIN);				//Switching the power
}


/*****************************************************************************************************
 * Routine to GPIO interrupt enable
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: TSL2651_PWR_PORT,TSL2651_PWR_PIN
 ***************************************************************************************/

void GPIO_INT_ENABLE()
{
	 /*Configure PD1 interrupt on falling edge */
	 GPIO_ExtIntConfig(TSL2651_INT_PORT, TSL2651_INT_PIN,1, false, true, true);
	 NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	 NVIC_EnableIRQ(GPIO_ODD_IRQn);		//Enable GPIO_ODD interrupt vector in NVIC
}


/*****************************************************************************************************
 * Routine to GPIO interrupt disable
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: TSL2651_PWR_PORT,TSL2651_PWR_PIN
 ***************************************************************************************/

void GPIO_INT_DISABLE()
{
	 /*Configure PD1 interrupt on falling edge */
	 GPIO_ExtIntConfig(TSL2651_INT_PORT, TSL2651_INT_PIN,1, false, true, false);
	 GPIO->IFC =0xFFFF;
	 NVIC_DisableIRQ(GPIO_ODD_IRQn);		//Enable GPIO_ODD interrupt vector in NVIC
}


/*****************************************************************************************************
 * Routine to GPIO interrupt handler
 *
 * Input variable: none
 * Global variable: none
 * Return type: none
 * Macros: SENSOR_ADC0_LOW,SENSOR_ADC0_HIGH,INTERRUPT_CLEAR,INTERRUPT_REG_VAL,SENSOR_THRESHOLD_LOW,SENSOR_THRESHOLD_HIGH
 ***********************************************************************************************************************/

void GPIO_ODD_IRQHandler(void)
{
  blockSleepMode(ENERGY_MIN);
  uint16_t adcdata0low = 0;							//variable to read the lower byte of sensor adc0
  uint16_t adcdata0high = 0;						//variable to read the higher byte of sensor adc0
  uint16_t adcdata0 = 0;							//variable to store the total 16 bit data of adc sensor
  INT_Disable();									//Disable all interrupts
  GPIO->IFC |= 0xFFFF;								//Clear the interrupts
  adcdata0low = I2C1_Read(SENSOR_ADC0_LOW);   		// Reading from adc0 data0 low register
  adcdata0high = I2C1_Read(SENSOR_ADC0_HIGH);		//Reading from adc0 data0 high register
  adcdata0 = ((adcdata0high << 8 ) | adcdata0low);
  I2C1_Write(INTERRUPT_CLEAR,INTERRUPT_REG_VAL);	//Clear the interrupt on sensor and re-enable it
  if(adcdata0 < SENSOR_THRESHOLD_LOW)				//Check for condition
  {
	  LED_ON(0);
  }
  else if(adcdata0 > SENSOR_THRESHOLD_HIGH)
  {
	  LED_OFF(0);
  }
  unblockSleepMode(ENERGY_MIN);
  INT_Enable();										//Enable interrupts

}

/* **************************************************************************
 * Function is used to setup the LETIMER0 peripheral
 *
 * Input variable:  none
 * Local variable: none
 * Global variable:letimer_prescaler,period_EM3,on_period_EM3,calib_ratio
 * Return type: none
 * MACRO: ENERGY_MIN,ULFRCO_COUNT,CALIBRATION,ONE_CYCLE
 *
 ***************************************************************************/
void LETIMER0_SETUP()
{
  /*Set initial compare values for COMP0 and COMP1
	COMP1 keeps it's value and is used as TOP value
	for the LETIMER.
	COMP1 gets decremented through the program execution
	to generate a different PWM duty cycle */

	/*structure defining the configurations of LETIMER0*/

	const LETIMER_Init_TypeDef letimerInit =
	{
	  .enable         = true,                   // Start counting when init completed.
	  .debugRun       = false,                  // Counter shall not keep running during debug halt.
	  .rtcComp0Enable = false,                  // Don't start counting on RTC COMP0 match.
	  .rtcComp1Enable = false,                  // Don't start counting on RTC COMP1 match.
	  .comp0Top       = true,                   // Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP
	  .bufTop         = false,                  // Don't load COMP1 into COMP0 when REP0 reaches 0.
	  .out0Pol        = 0,                      // Idle value for output 0.
	  .out1Pol        = 0,                      // Idle value for output 1.
	  .ufoa0          = letimerUFOANone,        // No changes on underflow
	  .ufoa1          = letimerUFOANone,        // No changes on underflow
	  .repMode        = letimerRepeatFree       // Free running
	};

	prescaler_set();

	if(CALIBRATION == 1)
	{
		period_EM3 = (calib_ratio * ULFRCO_COUNT * ONE_CYCLE);		//calculate the new period for EM3 using calibration ratio
		on_period_EM3 = (calib_ratio * ULFRCO_COUNT * ON_TIME);		//calculate the new on-time for EM3 using calibration ratio
	}
	else
	{
		period_EM3 = (ULFRCO_COUNT * ONE_CYCLE);			//calculate the new period for EM3 using calibration ratio
		on_period_EM3 = (ULFRCO_COUNT * ON_TIME);			//calculate the new on-time for EM3 using calibration ratio
	}

	if(ENERGY_MIN == 3)										//Check for minimum energy mode
	{
		LETIMER_CompareSet(LETIMER0, 0, period_EM3);
		LETIMER_CompareSet(LETIMER0, 1, on_period_EM3);
	}
	else
	{
		CMU->LFAPRESC0 = (letimer_prescaler << 8);			//set prescalar
		LETIMER_CompareSet(LETIMER0, 0, period);
	    LETIMER_CompareSet(LETIMER0, 1, on_period);
	}

	 LETIMER_IntClear(LETIMER0, INT_CLR_ALL); 							// Clears all interrupts LETIMER0_IFC_UF
	 LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0|LETIMER_IEN_COMP1);  // Enabling COMP0 and COMP1 interrupts
	 LETIMER_Init(LETIMER0, &letimerInit);								// Initialize LETIMER0
	 NVIC_EnableIRQ(LETIMER0_IRQn);										//Enable NVIC
	 LETIMER_Enable(LETIMER0, true);
}

/*********************************************
 * Interrupt Handler for  peripheral LETIMER0
 *
 * Input variable:  none
 * Global variable: letimer_interrupt_flag,acmp_value
 * Return type: none
 * Macros: HIGH_LEVEL, LOW_LEVEL ,HIGH_TEMP_LIMIT_C, LOW_TEMP_LIMIT_C
 *********************************************/
void LETIMER0_IRQHandler(void)
{
	INT_Disable();									//Disable interrupts
	letimer_interrupt_flag = LETIMER_IntGet(LETIMER0);		//Check if COMP0 and COMP1 interrupt
	if(letimer_interrupt_flag & LETIMER_IF_COMP0)			//check for COMP0 interrupt
	{
		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0);			//Clear COMP0 interrupt
		ADC_MIC_SETUP();
		MIC_CALC();
		ADC_FSR_SETUP();
		FSR_CALC();

		pir_value = PIR_GET_VALUE();
		pir_value = 0;
		if (pir_value == 0)
		{
			//LED_ON(0);
			add_item(cbuff,0xA);

		}
		else
		{
			//LED_OFF(0);
			add_item(cbuff,0xB);
		}

		water_level_value = WATER_LEVEL_GET_VALUE();
		water_level_value = 1;
		if (water_level_value == 0)
		{
			//LED_ON(1);
			add_item(cbuff,water_level_value);
		}
		else
		{
			//LED_OFF(1);
			add_item(cbuff,water_level_value);
		}
#if 0
		if(interrupt_cnt == 0)								//Check for the interrupt count number
		{
			TSL2651_POWERUP();					//Power up the I2C sensor
			interrupt_cnt++;					//Increment interrupt count number
			GPIO_INT_ENABLE();					//Enable the interrupts
		}
		else if (interrupt_cnt == 1)
		{
			interrupt_cnt++;
		}
		else if(interrupt_cnt ==2)
		{
			TSL2651_POWERDOWN();				//Power down the I2C Sensor
			GPIO_INT_DISABLE();					//Disable the GPIO interrupts
			I2C_Enable(I2C1,false);				//I2C1 disable
			interrupt_cnt = 0;
		}
#endif
		#ifdef DMA_REQD										//Check if DMA transfer or ADC polling
			ADC_TMP36_SETUP();
			DMA_ADC_Setup();								//Setup DMA
			blockSleepMode(ADC_EM);							//Enter Block Sleep Mode 1
			ADC_Start(ADC0, ADC_MODE);					//Enable ADC
		#else
			temp_celsius = TEMP_CALC();					//Calculate the temperature
			if((temp_celsius > HIGH_TEMP_LIMIT_C) || (temp_celsius < LOW_TEMP_LIMIT_C))	//check if tempereature is withtin threshold
			{
				LED_ON(1);
			}
			else
			{
				LED_OFF(1);
			}
		#endif
	}
	else
	{
		LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP1);		//Clear COMP1 interrupt
	}

	INT_Enable();							//Enable interrupts
}



int main(void)
{
  /* Chip errata */
  CHIP_Init();

	#if CALIBRATION == 1
  	  CLOCK_SETUP(cmuSelect_LFXO);				//Setup clock for LFXO
  	  TIMER0_SETUP();							//setup timer0
  	  TIMER1_SETUP();							//setup timer1
  	  LETIMER0_CALIB_SETUP(LFXO_COUNT);			//Setup LETIMER to LFXO for calibration
  	  LETIMER_CALIBRATION();					//Call calibration
	#endif

  if(LEUART_ENABLE == 1)
  {
	  CLOCK_SETUP(cmuSelect_LFXO);
	  LEUART_SETUP();
  }

  if(ENERGY_MIN ==3)						//Check for energy min mode and set clock
  {
	  CLOCK_SETUP(cmuSelect_ULFRCO);
  }
  else
  {
	  CLOCK_SETUP(cmuSelect_LFXO);
  }
  cbuff = &circular_buffer_structure;
  Buffer_Init(cbuff,array,20);
  GPIO_LED_SETUP();
  I2C_GPIO_SETUP();							//I2C_GPIO_SETUP
  TSL2651_GPIO_SETUP();						//I2C SENSOR GPIO SETUP
  TMP36_GPIO_SETUP();

  LETIMER0_SETUP();							//Setup the LETIMER
  PIR_GPIO_SETUP();
  WATER_LEVEL_GPIO_SETUP();
  MIC_GPIO_SETUP();
  FSR_GPIO_SETUP();
  while (1) {
	 sleep();
  }
}
