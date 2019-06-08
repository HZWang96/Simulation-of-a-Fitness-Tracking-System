/* EE2028 Assignment 2 Main Code
 *
 * Done by:
 * Wang Haozhe (A0168337W)
 * Wang Shuhui (A0173413M)
 */

#include "stdio.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "acc.h"
#include "oled.h"
#include "light.h"
#include "temp.h"
#include "led7seg.h"
#include "pca9532.h"


#define ACC_THRESHOLD 0.1
#define LIGHT_THRESHOLD 3000
#define TEMP_THRESHOLD 28.0
#define HALF_SECOND 500
#define ONE_SECOND 1000
#define THREE_SECONDS 3000
#define FIVE_SECONDS 5000
#define NUM_HALF_PERIODS 340

//----------------------Variables for timer--------------------//
volatile uint32_t msTicks; // counter for 1ms SysTicks
volatile uint32_t oneSecondTicks, halfSecondTicks, threeSecondsTicks,fiveSecondsTicks, Current_Ticks;
uint8_t time[17] = {0};

//--------------------Variables and arrays for sensor readings-----------------//

// Light sensor
uint8_t light_intensity[16] = {0};
uint32_t light_msg_length = 0;
uint16_t light_reading;

// Accelerometer
int8_t x_i = 0;
int8_t y_i = 0;
int8_t z_i = 0;
int8_t acc_x = 0;
int8_t acc_y = 0;
int8_t acc_z = 0;
int8_t net_acc_x;
int8_t net_acc_y;
int8_t net_acc_z;
float net_acc;
uint8_t acceleration[12] = {0};

// Temperature sensor
uint32_t counter = 0;
volatile uint32_t t1;
volatile uint32_t t2;
volatile float temp_reading;
uint8_t temperature[15] = {0};

//--------------------------Variables for LED array----------------------------//
volatile uint16_t led_division_width = LIGHT_THRESHOLD/16;
volatile uint16_t num_leds_on;
uint16_t LED_ON[16] = {0x0001, 0x0003, 0x0007, 0x000F, 0x001F, 0x003F, 0x007F, 0x00FF,
 0x01FF, 0x03FF, 0x07FF, 0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF};

//---------------------Variable to store the state of SW4---------------------//
volatile uint8_t SW4 = 1;

//------------------------------Flags used in code----------------------------------//
volatile bool new_light_reading = FALSE;   // set flag to TRUE when there is a new light reading
volatile bool new_temp_reading = FALSE;  // set flag to TRUE when there is a new temperature reading
volatile bool init_complete = FALSE;  // set flag to TRUE when initialization is complete
volatile bool start_toggle = FALSE;   //  set flag to TRUE when only SW3 is pressed
volatile bool rest_flag = TRUE;     // set flag to TRUE when temperature reading exceeds TEMP_THRESHOLD
volatile bool emergency_over = FALSE;  // set flag to TRUE when emergency is over
bool SW4_pressed = FALSE;           // set flag to TRUE when SW4 is pressed

//  SysTick_Handler - just increment SysTick counter
void SysTick_Handler(void)
{
	msTicks++;
}

// Get number of msTicks (for temp_init function)
uint32_t num_msTicks(void)
{
	return msTicks;
}

void pinsel_uart3(void){
	// set UART function (function 2) for port 0 pin 0(TXD3) and pin 1(RXD3)
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect on port 0 pin 10 and 11 function 2*/
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C2 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void)
{
	// Initialize SW4
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);

	// Initialize SW3 for EINT0 interrupt
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);

	// Initialize Temperature Sensor
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, 1<<2, 0);
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void rgb_pinConfig(void)
{
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;

	PinCfg.Portnum= 2;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Portnum= 0;
	PinCfg.Pinnum = 26;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Portnum= 2;
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(2, 1<<0, 1);
	GPIO_SetDir(2, 1<<1, 1);
	GPIO_SetDir(0, 1<<26, 1);
}

static void init_LightInt(void)
{
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum= 2;
	PinCfg.Pinnum = 5;

	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(2, 1<<5, 0);

	light_clearIrqStatus();  // clear interrupt active status when initializing
	light_setLoThreshold(3000); // set threshold for interrupting (interrupt below 3000 lux)
	light_setHiThreshold(3000); // set threshold for interrupting (interrupt above 3000 lux)
}

void init_uart(void){
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	//pin select for uart3;
	pinsel_uart3();

	//supply power & setup working parameters for uart3
	UART_Init(LPC_UART3, &uartCfg);

	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
}


static void Clear7SegDisplay(void)
{
	led7seg_setChar(':', FALSE);
}

static void displayLetter(uint8_t *letter) {
	uint8_t next_letter = 0;
	uint8_t i = 0;
	while(1)
	{
		if(msTicks - oneSecondTicks >= ONE_SECOND)
		{
			oneSecondTicks = msTicks;
			next_letter = *letter++;
			led7seg_setChar(next_letter, FALSE);
			if(i % 2 == 0)
				GPIO_SetValue(0, 1<<26); // blue LED on
			else
				GPIO_ClearValue(0, 1<<26); // blue LED off
			i++;
		}
		if (next_letter == '\0'){
			Clear7SegDisplay();
			GPIO_ClearValue(0, 1<<26); // blue LED off
			break;
		}
	}
}

static uint8_t *letter = (uint8_t*)"SAUED";


void EINT0_IRQHandler(void)
{
	if(SW4_pressed == TRUE)
	{
		emergency_over = TRUE;
		SW4_pressed = FALSE;
		LPC_SC->EXTINT = (1<<0);
	}
	else
	{
		start_toggle = TRUE;
		LPC_SC->EXTINT = (1<<0);
	}
}


void EINT3_IRQHandler(void)
{
	if((LPC_GPIOINT->IO2IntStatF>>5) & 0x1)
	{
		new_light_reading = TRUE;
		LPC_GPIOINT->IO2IntClr = 1<<5;
		light_clearIrqStatus();  // clear interrupt active status
	}

	if((LPC_GPIOINT->IO0IntStatF>>2) & 0x1)
	{
		if(counter == 0)
			t1 = msTicks;

		counter++;

		if(counter == 171)
		{
			t2 = msTicks;
			if (t2 > t1)
				t2 -= t1;

			else
				t2 += (0xFFFFFFFF - t1 + 1);

			new_temp_reading = TRUE;
			counter = 0;
		}
		LPC_GPIOINT->IO0IntClr = 1<<2;
	}
}

int main(void)
{
	uint8_t UART_message[61] = {0};
	//Setup SysTick Timer to interrupt at 1ms intervals
	if(SysTick_Config(SystemCoreClock / 1000))
	{
		while(1);
	}

	halfSecondTicks = oneSecondTicks = threeSecondsTicks = msTicks;

	/*-----------------Initialization of FitNUS--------------------*/
	init_i2c();
	init_ssp();
	init_GPIO();
	pinsel_uart3();
	init_uart();
	oled_init();
	rgb_pinConfig();
	led7seg_init();
	Clear7SegDisplay(); // turn off all segments on the 7-segment display
	light_enable();     // enable light sensor
	light_setRange(LIGHT_RANGE_4000); // set light sensor range to 0 lux to 4000 lux
	init_LightInt();  // configure light sensor interrupt
	temp_init(&num_msTicks); // enable temperature sensor
	acc_init();    // initialize the accelerometer
	oled_clearScreen(OLED_COLOR_BLACK);

	// print initialization message on OLED screen
	oled_putString(5, 0, (uint8_t*)"Initialization", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(33, 15, (uint8_t*)"mode.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(10, 30, (uint8_t*)"Press TOGGLE", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(23, 45, (uint8_t*)"to climb", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	// send Start to FiTrack X
	uint8_t *msg = (uint8_t*)"Start\r\n\n";
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg),BLOCKING);

	/*-----------------End of Initialization----------------------*/



	/*------Configure interrupts and set priority levels-------*/
	uint32_t SW3_priority;
	uint32_t light_and_temp_priority;

	NVIC_SetPriorityGrouping(5);
	SW3_priority = NVIC_EncodePriority(5,0b1,0b000); //SW3 interrupt
	NVIC_SetPriority(EINT0_IRQn, SW3_priority); //SW3 interrupt has a higher priority than Light Interrupt
	LPC_SC->EXTMODE = (1<<0);
	LPC_SC->EXTPOLAR = (0<<0);
	NVIC_EnableIRQ(EINT0_IRQn);


	light_and_temp_priority = NVIC_EncodePriority(5,0b10,0b000); //Light interrupt.
	NVIC_SetPriority(EINT3_IRQn, light_and_temp_priority); //Light Interrupt has a lower priority than SW3 interrupt.
	LPC_GPIOINT->IO2IntEnF |= 1<<5; // light sensor toggles from 1 to 0 in I2C, hence falling edge interrupt

	LPC_GPIOINT->IO0IntEnF |= 1<<2; // trigger temperature interrupt on falling edge of output waveform
	NVIC_EnableIRQ(EINT3_IRQn);


	/*------------------------- Toggle Mode ---------------------------*/


	while(init_complete == FALSE)
	{
		// Toggle from initialization mode to climb mode once SW3 is pressed
		if(start_toggle == TRUE)
		{
			oled_clearScreen(OLED_COLOR_BLACK);
			oled_putString(5, 0, (uint8_t*)"INITIALIZATION", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(15, 15, (uint8_t*)"COMPLETE.", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(20, 30, (uint8_t*)"ENTERING", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(10, 45, (uint8_t*)"CLIMB MODE", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			uint8_t i = 0x39;
			uint8_t j = 0;

			// Toggle the state of the blue LED every 1 second
			while(j < 11)
			{
				if(msTicks - halfSecondTicks >= HALF_SECOND){
					halfSecondTicks = msTicks;
					led7seg_setChar(i, FALSE);
					i--;
					if(j % 2 == 0)
						GPIO_SetValue(0, 1<<26); // blue LED on
					else
						GPIO_ClearValue(0, 1<<26); // blue LED off
					j++;
				}
			}
			init_complete = TRUE;
			GPIO_ClearValue(0, 1<<26); // blue LED off
			Clear7SegDisplay();
			oled_clearScreen(OLED_COLOR_BLACK);
		}
	}


	/*-----------------------------Climb Mode-------------------------------------------*/
	acc_read(&x_i,&y_i ,&z_i);  // read acceleration when entering climb mode

	// send CLIMB mode to FiTrack X
	msg = (uint8_t*)"CLIMB mode\r\n\n";
	UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg),BLOCKING);
	fiveSecondsTicks = msTicks;

	while(1)
	{
		if(init_complete == TRUE) // program will not continue running if initialization not complete
		{
			oled_putString(30, 0, (uint8_t*)"CLIMB", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

			// Display light sensor reading on the OLED diplay
			if(new_light_reading == TRUE)
			{
				light_reading = light_read();
				sprintf(light_intensity, "Light: %d lux", light_reading);
				light_msg_length = strlen(light_intensity);
				light_intensity[15] = '\0';

				if(light_msg_length == 14)
				{
					light_intensity[14] = ' ';
					oled_putString(0,15,(uint8_t*)light_intensity,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
				}

				else if(light_msg_length == 13)
				{
					light_intensity[13] = ' ';
					light_intensity[14] = ' ';
					oled_putString(0,15,(uint8_t*)light_intensity,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
				}

				else if(light_msg_length == 12)
				{
					light_intensity[12] = ' ';
					light_intensity[13] = ' ';
					light_intensity[14] = ' ';
					oled_putString(0,15,(uint8_t*)light_intensity,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
				}

				else
					oled_putString(0,15,(uint8_t*)light_intensity,OLED_COLOR_WHITE,OLED_COLOR_BLACK);

				// Display DIM on the OLED display if light intensity falls below 3000 lux
				if(light_reading < LIGHT_THRESHOLD)
				{
					oled_putString(40,45,(uint8_t*)"DIM",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
					num_leds_on =(LIGHT_THRESHOLD - light_reading)/led_division_width;
					num_leds_on = LED_ON[num_leds_on];
					pca9532_setLeds(num_leds_on,0xffff);
				}
				else
				{
					// turn off all LEDs in the LEDs array
					pca9532_setLeds(0x0,0xffff);

					// clear the "DIM" message from the OLED display
					oled_putString(40,45,(uint8_t*)"   ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
				}
				new_light_reading = FALSE;
			}


			// computes the temperature reading from temperature sensor
			if(new_temp_reading == TRUE)
			{
				temp_reading =  ((2*1000*t2) / (NUM_HALF_PERIODS) - 2731)/10.0;
				new_temp_reading = FALSE;
			}

			sprintf(temperature, "Temp: %.1f deg",temp_reading);
			oled_putString(6,25,(uint8_t*)temperature,OLED_COLOR_WHITE,OLED_COLOR_BLACK);

			// Display "REST NOW" on the OLED display if temperature goes beyond threshold
			if(temp_reading > TEMP_THRESHOLD && rest_flag == TRUE)
			{
				oled_clearScreen(OLED_COLOR_BLACK);
				threeSecondsTicks = msTicks;
				while (msTicks - threeSecondsTicks < THREE_SECONDS)
					oled_putString(20,30,(uint8_t*)"REST NOW",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
				oled_clearScreen(OLED_COLOR_BLACK);
				rest_flag = FALSE;
			}

			if(temp_reading < TEMP_THRESHOLD)
			{
				rest_flag = TRUE;
			}

			// read accelerometer
			acc_read(&acc_x, &acc_y, &acc_z);

			// calculate net acceleration
			net_acc_x = acc_x - x_i;
			net_acc_y = acc_y - y_i;
			net_acc_z = acc_z - z_i;
			net_acc = sqrt(pow(net_acc_x, 2) + pow(net_acc_y, 2) + pow(net_acc_z, 2))/64;

			sprintf(acceleration, "Acc: %.2fg",net_acc);
			oled_putString(12,35,(uint8_t*)acceleration,OLED_COLOR_WHITE,OLED_COLOR_BLACK);

			// send acceleration,temperature and light sensor readings to FiTrack X every 5 seconds
			if(msTicks - fiveSecondsTicks >= FIVE_SECONDS)
			{
				fiveSecondsTicks = msTicks;
				UART_Send(LPC_UART3, (uint8_t *) light_intensity, 16,BLOCKING);
				UART_Send(LPC_UART3, (uint8_t *) "\r\n", 2,BLOCKING);
				UART_Send(LPC_UART3, (uint8_t *) temperature, 15,BLOCKING);
				UART_Send(LPC_UART3, (uint8_t *) "\r\n", 2,BLOCKING);
				UART_Send(LPC_UART3, (uint8_t *) acceleration, 12,BLOCKING);
				UART_Send(LPC_UART3, (uint8_t *) "\r\n\n", 3,BLOCKING);
			}

		}

	/*----------------------------Emergency Mode---------------------------------*/
			uint8_t i = 0;

			// enter emergency mode if net acceleration is higher than the threshold of 0.1g
			if(net_acc > ACC_THRESHOLD)
			{
				fiveSecondsTicks = Current_Ticks = msTicks;
				emergency_over = FALSE;
				// send EMERGENCY! to FiTrack X
				msg = (uint8_t*)"EMERGENCY!\r\n\n";
				UART_Send(LPC_UART3, msg, strlen(msg), BLOCKING);

				oled_clearScreen(OLED_COLOR_BLACK);
				while(emergency_over == FALSE)
				{
					oled_putString(20, 0,(uint8_t*)"EMERGENCY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
					oled_putString(33,10,(uint8_t*)"Mode!",OLED_COLOR_WHITE,OLED_COLOR_BLACK);

					// compute temperature reading from the temperature sensor
					if(new_temp_reading == TRUE)
					{
						temp_reading =  ((2*1000*t2) / (NUM_HALF_PERIODS) - 2731)/10.0;
						new_temp_reading = FALSE;
					}
					sprintf(temperature, "Temp: %.1f deg",temp_reading);

					// read accelerometer
					acc_read(&acc_x, &acc_y, &acc_z);

					// calculate net acceleration
					net_acc_x = acc_x - x_i;
					net_acc_y = acc_y - y_i;
					net_acc_z = acc_z - z_i;
					net_acc = sqrt(pow(net_acc_x, 2) + pow(net_acc_y, 2) + pow(net_acc_z, 2))/64;
					sprintf(acceleration, "Acc: %.2fg", net_acc);

					// Display temperature and net acceleration readings on the OLED display
					oled_putString(5,20,(uint8_t*)temperature,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
					oled_putString(15,30,(uint8_t*)acceleration,OLED_COLOR_WHITE,OLED_COLOR_BLACK);

					// display time elapsed on the OLED display
					sprintf(time, "Duration: %d s", (msTicks - Current_Ticks)/1000);
					oled_putString(0,40,(uint8_t*)time,OLED_COLOR_WHITE,OLED_COLOR_BLACK);

					// alternate blue and red LEDs every 500ms
					if (msTicks - halfSecondTicks >= HALF_SECOND){
						halfSecondTicks = msTicks;
						if (i%2 == 0){
							GPIO_ClearValue(2, 1<<0); // red LED off
							GPIO_SetValue(0, 1<<26); // blue LED on
						}
						else{
							GPIO_ClearValue(0, 1<<26); // blue LED off
							GPIO_SetValue(2, 1<<0);  // red on
						}
						i++;
					}

					// send acceleration,temperature and time elapsed to FiTrack X every 5 seconds
					if(msTicks - fiveSecondsTicks >= FIVE_SECONDS)
					{
						fiveSecondsTicks = msTicks;
						UART_Send(LPC_UART3, (uint8_t *) temperature, 15,BLOCKING);
						UART_Send(LPC_UART3, (uint8_t *) "\r\n", 2,BLOCKING);
						UART_Send(LPC_UART3, (uint8_t *) acceleration, 12,BLOCKING);
						UART_Send(LPC_UART3, (uint8_t *) "\r\n", 2,BLOCKING);
						UART_Send(LPC_UART3, (uint8_t *) time, 17,BLOCKING);
						UART_Send(LPC_UART3, (uint8_t *) "\r\n\n", 3,BLOCKING);
					}

					// check if SW4 has been pressed
					SW4 = (GPIO_ReadValue(1)>>31)&0x1;
					if(SW4 == 0)
						SW4_pressed = TRUE;
					else
						SW4_pressed = FALSE;
				}
              /*------------------------------Emergency Over-----------------------------------*/
				// send "Emergency is cleared! Time consumed for recovery: xx sec" to FiTracks X
				sprintf(UART_message, "Emergency is cleared! Time consumed for recovery: %d sec\r\n\n", (msTicks - Current_Ticks)/1000);
				UART_Send(LPC_UART3,UART_message, 61, BLOCKING);

				GPIO_ClearValue(2, 1<<0); // red LED off
				displayLetter(letter); // display S-A-U-E-D on the 7 segment display
				oled_clearScreen(OLED_COLOR_BLACK);

				// send "CLIMB mode" message to UART again before entering climb mode
				msg = (uint8_t*)"CLIMB mode\r\n\n";
				UART_Send(LPC_UART3, (uint8_t *) msg, strlen(msg),BLOCKING);

				// read accelerometer again before entering climb mode
				acc_read(&x_i,&y_i ,&z_i);
			}

		}
	return 0;
}
