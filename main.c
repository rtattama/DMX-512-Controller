//REKHA TATTAMANGALAM RAJAN
//UTA ID: 1001164021

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include "tm4c123gh6pm.h"

#define PIN1   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))
#define PIN2   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define PIN3   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define PIN4   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))
#define PIN5   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
#define PIN6   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define PIN7   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))
#define PIN8   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
#define PIN9   (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))
#define PIN10  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define RED_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

#define DE  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define LED (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define MAX_BUFFERLEN 80
#define MAX_DMXDEVICE 512
#define SPACE 32

char IS[512] = { '\0' };
char command[10] = { '\0' };
char address[10] = { '\0' };
char values[5] = { '\0' };
char lower[10] = { '\0' };
char alpha[10] = { '\0' };
char buffer_sprintf[120] = { '\0' };
char DMX_buffer[40] = { '\0' };
char DMX_maxbuffer[120] = { '\0' };
int max_address = 512;
uint8_t DMX_VALUE[MAX_DMXDEVICE];
int DMX_RX_DATA[MAX_DMXDEVICE];
bool MASTER_MODE = false;
volatile int TX_FLAG = 0;
int TX_BREAKMAB = 0;
int TX_STARTCODE = 1;
int TX_PACKETS = 2;
int PACKET_NUM = 0;
int START_CODE = 0;
int TX_PHASE = 0;
int v;
int r;
int h;
int l;
int er1;
int er2;
int RX_PACKET;
int RX_data;
int RX_PHASE;
int DIP_VALUE;
int TX;
int RX;
int value;
int LED_TIMEOUT = 25;
int poll_flag = 0;
int p;

void putcUart0(char c) {
	while (UART0_FR_R & UART_FR_TXFF)
		;
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str) {
	int i;
	int lenght = strlen(str);
	for (i = 0; i < lenght; i++)
		putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0() {
	while (UART0_FR_R & UART_FR_RXFE)
		;
	return UART0_DR_R & 0xFF;
}

void configureUart1for_250000() {
	UART1_CTL_R = 0;
	UART1_CC_R = UART_CC_CS_SYSCLK; //40 MH
	UART1_IBRD_R = 10; // 10 for 250000bits per seconds ,
	UART1_FBRD_R = 0;
	UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
	if (MASTER_MODE == true) {
		UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN | UART_CTL_EOT;
		UART1_IM_R = UART_IM_TXIM; // turn-on TX interrupt
	} else {
		UART1_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN;
		UART1_IM_R = UART_IM_RXIM;         // turn-on RX interrupt
	}
	NVIC_EN0_R |= 1 << (INT_UART1 - 16);  // turn-on interrupt 21 (UART1)
}

void step8UART1_250K() {
	UART1_CTL_R = 0;
	UART1_CC_R = UART_CC_CS_SYSCLK; //40 MH
	UART1_IBRD_R = 10; // 10 for 250000bits per seconds ,
	UART1_FBRD_R = 0;
	UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
	if (MASTER_MODE == true) {
		UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN | UART_CTL_EOT;
		UART1_IM_R &= ~(UART_IM_TXIM);  // turn-on TX interrupt
	} else {
		UART1_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN;
		UART1_IM_R &= ~(UART_IM_RXIM);         // turn-on RX interrupt
	}
	NVIC_EN0_R &= ~(1 << (INT_UART1 - 16));  // turn-on interrupt 21 (UART1)
}

int getValue() {
	char valueString[80];
	if (!PIN1)
		MASTER_MODE = true;
	else
		MASTER_MODE = false;
	value = !PIN2 << 8 | !PIN3 << 7 | !PIN4 << 6 | !PIN5 << 5 | !PIN6 << 4
			| !PIN7 << 3 | !PIN8 << 2 | !PIN9 << 1 | !PIN10 << 0;
	sprintf(valueString, "\n\rDip Switch Address = %d \n\r", value);
	putsUart0(valueString);
	putsUart0("\n\rReady\n\r");
	DIP_VALUE = value;
	if (MASTER_MODE == true) {
		sprintf(valueString, "Controller is operating in the Master Mode.\n\r");
		putsUart0(valueString);
		DE = 1;
	} else {
		sprintf(valueString, "Controller is operating in the Slave Mode.\n\r");
		putsUart0(valueString);
		DE = 0;
		//configureUart1for_250000();
	}
	return 0;
}

void configureInputPORTA(int pin) {
	int mask = 0;
	mask = 1 << pin;
	GPIO_PORTA_DIR_R &= ~mask;
	GPIO_PORTA_DR2R_R |= mask; // set drive strength to 2mA
	GPIO_PORTA_DEN_R |= mask;
	GPIO_PORTA_PUR_R |= mask;  // enable internal pull-up for inputs
}

void configureInputPORTB(int pin) {
	int mask = 0;
	mask = 1 << pin;
	GPIO_PORTB_DIR_R &= ~mask;
	GPIO_PORTB_DR2R_R |= mask; // set drive strength to 2mA
	GPIO_PORTB_DEN_R |= mask;
	GPIO_PORTB_PUR_R |= mask;  // enable internal pull-up for inputs
}

void configureInputPORTD(int pin) {
	int mask = 0;
	mask = 1 << pin;
	GPIO_PORTD_DIR_R &= ~mask;
	GPIO_PORTD_DR2R_R |= mask; // set drive strength to 2mA
	GPIO_PORTD_DEN_R |= mask;
	GPIO_PORTD_PUR_R |= mask;  // enable internal pull-up for inputs
}

void configureInputPORTE(int pin) {
	int mask = 0;
	mask = 1 << pin;
	GPIO_PORTE_DIR_R |= 0x20;
	GPIO_PORTE_DR2R_R |= 0x20; // set drive strength to 2mA
	GPIO_PORTE_DEN_R |= 0x2E;
	GPIO_PORTE_PUR_R |= 0x0E;  // enable internal pull-up for inputs
}

void configureInputPORTC(int pin) {
	int mask = 0;
	mask = 1 << pin;
	GPIO_PORTC_DIR_R |= 0x60;
	GPIO_PORTC_DR2R_R |= 0x60; // set drive strength to 2mA
	GPIO_PORTC_DEN_R |= 0x60;
	GPIO_PORTC_PUR_R |= 0;  // enable internal pull-up for inputs
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us) {
	// Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");
	// 1
	__asm("WMS_LOOP1:   SUB  R1, #1");
	// 6
	__asm("             CBZ  R1, WMS_DONE1");
	// 5+1*3
	__asm("             NOP");
	// 5
	__asm("             B    WMS_LOOP1");
	// 5*3
	__asm("WMS_DONE1:   SUB  R0, #1");
	// 1
	__asm("             CBZ  R0, WMS_DONE0");
	// 1
	__asm("             B    WMS_LOOP0");
	// 1*3
	__asm("WMS_DONE0:");
	// ---	// 40 clocks/us + error
}

void blinkGreenLED() {
	//Blink the Green Led for 250 milli second at start up
	GREEN_LED = 1;
	waitMicrosecond(250000);
	GREEN_LED = 0;
}
void initHw() {

	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
			| SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

	// Set GPIO ports to use APB (not needed since default configuration -- for clarity)
	SYSCTL_GPIOHBCTL_R = 0;

	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOA
			| SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOF
			| SYSCTL_RCGC2_GPIOC;

	GPIO_PORTF_DIR_R = 0x0E;
	GPIO_PORTF_DR2R_R = 0x0E;
	GPIO_PORTF_DEN_R = 0x0E;

	GPIO_PORTB_DIR_R |= 0x20;
	GPIO_PORTB_DR2R_R |= 0x20; // set drive strength to 2mA
	GPIO_PORTB_DEN_R |= 0x20;

	GPIO_PORTE_DIR_R |= 0x20;  // bits 1 as output
	GPIO_PORTE_DR2R_R |= 0x20; // set drive strength to 2mA
	GPIO_PORTE_DEN_R |= 0x20;

	// set drive strength to 2mA
	//GPIO_PORTE_DEN_R |= 0x0E;
	//GPIO_PORTA_PUR_R |= 0x0E;

	configureInputPORTA(6);
	configureInputPORTA(7);
	configureInputPORTB(4);
	configureInputPORTD(3);
	configureInputPORTD(2);
	configureInputPORTD(1);
	configureInputPORTD(0);
	configureInputPORTE(3);
	configureInputPORTE(2);
	configureInputPORTE(1);
	configureInputPORTC(6);

	//A = 1;

	//DE = 1;

	// Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
	GPIO_PORTA_DEN_R |= 3; // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3; // default, added for clarity
	GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
	UART0_CTL_R = 0; // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK; // use system clock (40 MHz)
	UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	UART0_FBRD_R = 45; // round(fract(r)*64)=45
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
	UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
	GPIO_PORTC_DEN_R |= 0x70;
	GPIO_PORTC_AFSEL_R |= 0x30;
	GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

}
/*void configurePWM(){
 SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)
 | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;

 // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
 SYSCTL_GPIOHBCTL_R = 0;

 // Enable GPIO port B and E peripherals
 SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

 // Configure RED LED
 GPIO_PORTF_DIR_R |= 0x02;   // make bit1 an output
 GPIO_PORTF_DR2R_R |= 0x02;  // set drive strength to 2mA
 GPIO_PORTF_DEN_R |= 0x02;   // enable bit1 for digital
 GPIO_PORTF_AFSEL_R |= 0x02; // select auxilary function for bit 1
 GPIO_PORTF_PCTL_R = GPIO_PCTL_PF1_M1PWM5; // enable PWM on bit 1


 SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM1;             // turn-on PWM0 module
 __asm(" NOP");                                   // wait 3 clocks
 __asm(" NOP");
 __asm(" NOP");
 SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM0 module
 SYSCTL_SRPWM_R = 0;                              // leave reset state
 PWM1_2_CTL_R = 0;                                // turn-off PWM0 generator 1

 PWM1_2_GENB_R = PWM_2_GENB_ACTCMPBD_ZERO | PWM_2_GENB_ACTLOAD_ONE;
 // output 3 on PWM0, gen 1b, cmpb

 PWM1_2_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

 PWM1_INVERT_R = PWM_INVERT_PWM5INV;
 // invert outputs for duty cycle increases with increasing compare values
 //PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)

 PWM1_2_CTL_R = PWM_2_CTL_ENABLE;                 // turn-on PWM0 generator 1

 PWM1_ENABLE_R = PWM_ENABLE_PWM5EN;


 }*/

void Timerconfig() {
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
	TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
	TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
	TIMER1_TAILR_R = 0x61A80;  // set load value to 40e6 for 1 Hz interrupt rate
	TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
	NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
	TIMER1_CTL_R |= TIMER_CTL_TAEN;
}
void Timer1ISR() {
	//Timerconfig();
	if (LED_TIMEOUT != 0) {
		BLUE_LED = 1;
		LED_TIMEOUT--;
		Timerconfig();
	} else if (LED_TIMEOUT == 0) {
		TIMER1_ICR_R = TIMER_ICR_TATOCINT;
		BLUE_LED = 0;
	}

}

void configureUart1for_100000() {
	UART1_CTL_R = 0;
	UART1_CC_R = UART_CC_CS_SYSCLK; //40 MH
	UART1_IBRD_R = 25; // 25 for 100000bits per seconds , to generate a period of 10usec to trasfer 1 bit
	UART1_FBRD_R = 0;
	UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
	UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_EOT;
	UART1_IM_R = UART_IM_TXIM;        // turn-on TX interrupt
	NVIC_EN0_R |= 1 << (INT_UART1 - 16); // turn-on interrupt 21 (UART1)
}
void step8UART1_100K() {
	UART1_CTL_R = 0;
	UART1_CC_R = UART_CC_CS_SYSCLK; //40 MH
	UART1_IBRD_R = 25; // 25 for 100000bits per seconds , to generate a period of 10usec to trasfer 1 bit
	UART1_FBRD_R = 0;
	UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
	UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_EOT;
	UART1_IM_R &= ~(UART_IM_TXIM); // turn-on TX interrupt
	NVIC_EN0_R &= ~(1 << (INT_UART1 - 16)); // turn-on interrupt 21 (UART1)
}

void inputstring() {
	int m = 0;
	int k;
	char ch = '\0';
	for (k = 0; k < MAX_BUFFERLEN; k++) {
		IS[k] = '\0';
	}
	for (m = 0; m <= MAX_BUFFERLEN; m++) {
		ch = getcUart0();
		putcUart0(ch);
		if (ch == '\r') {
			putsUart0("\n\r");
			//IS[i] = NULL;
			break;
		} else if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')
				|| (ch >= '0' && ch <= '9')) {
			IS[m] = ch;
		} else if (ch == 8) {
			//Decrement the value of index
			m = m - 2;
			putcUart0(32);
			putcUart0(8);
		} else if (ch == 32 && m > 0) {
			if (IS[m - 1] == 32) {
				m--;
			} else {
				IS[m] = 32;
			}
		} else {
			m--;
		}
	}
}

char tolowerstring(char *lower) {
	int r;
	int temp_lower = strlen(lower);
	for (r = 0; r < temp_lower; r++) {
		if (isupper(command[r])) {
			command[r] = tolower(command[r]);
		}
	}
}

int alphatoint(char *alpha) {

	int z;
	int temp_alpha = strlen(alpha);
	for (z = 0; z <= temp_alpha; z++) {
		int x = atoi(address[z]);
	}
}

int input_string() {
	int s;
	int k = 0;
	int temp_string = strlen(command);
	for (s = 0; s < temp_string; s++) {
		if (isalpha(command[s]) == 0) {
			putsUart0("\n\r");
			for (k = 0; k < s; k++) {
				putcUart0(32);
			}
			putcUart0(94);
			sprintf(buffer_sprintf,
					"\n\rError in command %s , Please enter valid command such as SET,GET,MAX,ON,OFF,CLEAR,MAX,POLL.\n\r",
					command);
			putsUart0(buffer_sprintf);
			return 0;
		}
	}
}

int input_address() {
	int temp;
	int k = 0;
	int temp_digit = strlen(address);
	if (address == NULL) {
		putsUart0(
				"\n\rError : Please enter a valid address between 0 to 512 or Max Address.\n\r");
		return false;
	}
	for (v = 0; v < temp_digit; v++) {
		if (isdigit(address[v]) == 0) {
			putsUart0("\n\r");
			for (k = 0; k < strlen(command); k++) {
				putcUart0(32);
			}
			for (k = 0; k <= v; k++) {
				putcUart0(32);
			}
			putcUart0(94);
			putsUart0(
					"\n\rError:Please enter a valid address between 0 to 512 or Max Address.\n\r");
			return false;
		}
	}

	h = atoi(address);
	if (strcmp("max", command) == 0) {
		if ((h > 0) || (h < 512)) {
			return true;
		}
	}

	if ((h < 0) || (h > max_address)) {
		putsUart0(
				"\n\rError:Please enter a valid address between 0 to 512 or Max Address.\n\r");
		return false;
	}
	return true;
}

int input_values() {
	int k = 0;
	int temp_digit = strlen(values);
	if (values == NULL) {
		putsUart0(
				"\n\rError :Please enter a valid value between 0 to 256.\n\r");
		return false;
	}
	for (r = 0; r < temp_digit; r++) {
		if (isdigit(values[r]) == 0) {
			putsUart0("\n\r");
			for (k = 0; k < strlen(command); k++) {
				putcUart0(32);
			}
			for (k = 0; k < strlen(address); k++) {
				putcUart0(32);
			}
			putcUart0(32);
			for (k = 0; k <= r; k++) {
				putcUart0(32);
			}
			putcUart0(94);
			putsUart0(
					"\n\rError:Please enter a valid value between 0 to 256.\n\r");
			return false;
		}
	}

	l = atoi(values);
	if ((l < 0) || (l > 256)) {
		putsUart0("Error :Please enter a valid value between 0 to 256.\n\r ");
		return false;
	}
	return true;
}

void TX_ISR() {
	if (TX_FLAG == 1 && MASTER_MODE == 1) {
		configureUart1for_100000();
		TX_PHASE = TX_STARTCODE;
		UART1_DR_R = 0x00;
		putsUart0("Controller is in transmission mode.\n\r ");
	} else if (TX_FLAG == 0 && MASTER_MODE == 1) {
		putsUart0("Transmission flag is not set.\n\r");
	} else {
		putsUart0("Controller is in receive mode.\n\r");
		//configureUart1for_250000();
	}
}
void LED_func() {
	GPIO_PORTE_DIR_R |= 0x20;  // bit 5 as output
	GPIO_PORTE_DR2R_R |= 0x20; // set drive strength to 2mA
	GPIO_PORTE_DEN_R |= 0x2E;
}
void deviceupdate() {
	int x;
	//getValue();
	LED_func();
	//configurePWM();
	x = DMX_RX_DATA[DIP_VALUE + 0];
	//PWM1_2_CMPB_R = x;
	if (x == 0) {
		RED_LED = 0;
		LED = 0;
	} else {
		RED_LED = 1;
		LED = 1;
	}
}

void Uart1ISR() {
	if (TX == 1) {
		//putsUart0("Transmission \n\r ");

		if (PACKET_NUM > max_address) {
			TX_PHASE = TX_BREAKMAB;
			PACKET_NUM = 0;
		}

		if (TX_PHASE == TX_BREAKMAB) {
			configureUart1for_100000(); // So that we transfer the data at lower rate
			UART1_DR_R = 0x00; // Send the break code 1 start code,8 bit , 2 stop code == 90usec,20usec
			TX_PHASE = TX_STARTCODE; // So that next we trasfer the start code

		}
		if (TX_PHASE == TX_STARTCODE) {
			configureUart1for_250000(); // So that we trasfer the data at higher rate
			UART1_DR_R = START_CODE;	   // Send the start code at higher rate
			TX_PHASE = TX_PACKETS;// Send packets after sending the START code which may be 0x00,0x0c

		}

		if (TX_PHASE == TX_PACKETS) {
			UART1_DR_R = DMX_VALUE[PACKET_NUM++]; // Transfer and increment the to send data byte to trasnfer .

		}
		RED_LED ^= 1;
	} else if (RX == 1) {
		RX_data = UART1_DR_R;
		int detectBreak = RX_data & UART_DR_FE;

		if (detectBreak && ((RX_data & 0xFF) == 0)) {
			RX_PHASE = 0;
			UART1_ECR_R = 0xff;
			//deviceupdate();
			//GREEN_LED ^= 1;
		} else {
			DMX_RX_DATA[RX_PHASE++] = (0xFF & RX_data);
			deviceupdate();

			//GREEN_LED ^= 1;

		}
		GREEN_LED = 1;
	}
    GREEN_LED = 0;
	UART1_ICR_R |= UART_ICR_TXIC | UART_ICR_RXIC;
}

void step8_txisr() {
	if (poll_flag == 1) {
		step8UART1_100K();
		UART1_DR_R = 0x00;
		TX_PHASE = TX_STARTCODE;
	}
}

void step8_count() {
	p = 0;
	int start_bit = 0;
	int stop_bit = 512;
	for (p = start_bit; p <= stop_bit; p++) {
		DMX_VALUE[p] = 1;
	}
}

void polling() {
	while (p > 1) {
		step8_count();
		START_CODE = 0xF0;
		TX_PHASE = TX_STARTCODE;
		step8UART1_100K();
		UART1_DR_R = 0xF0;
		pollingUart();
		p = p / 2;

	}
}

int linearsearch(int *pointer, int a, int finds)
{
   int d;
   for ( d = 0 ; d < a ; d++ )
   {
      if ( *(pointer+d) == finds )
         return d;
   }

   return -1;
}


void pollingUart() {
	if ((poll_flag == 1) && (TX == 1)) {
		//putsUart0("Transmission \n\r ");

		if (PACKET_NUM > max_address) {
			TX_PHASE = TX_BREAKMAB;
			PACKET_NUM = 0;
		}

		if (TX_PHASE == TX_BREAKMAB) {
			configureUart1for_100000(); // So that we transfer the data at lower rate
			UART1_DR_R = 0x00; // Send the break code 1 start code,8 bit , 2 stop code == 90usec,20usec
			TX_PHASE = TX_STARTCODE; // So that next we trasfer the start code

		}
		if (TX_PHASE == TX_STARTCODE) {
			configureUart1for_250000(); // So that we trasfer the data at higher rate
			UART1_DR_R = 0xF0;	   // Send the start code at higher rate
			TX_PHASE = TX_PACKETS;// Send packets after sending the START code which may be 0x00,0x0c

		}
		if (TX_PHASE == TX_PACKETS) {
			UART1_DR_R = DMX_VALUE[PACKET_NUM++]; // Transfer and increment the to send data byte to trasnfer .

		}

	} else if (poll_flag == 0) {
		RX_data = UART1_DR_R;
		int detectBreak = RX_data & UART_DR_FE;

		if (detectBreak && ((RX_data & 0xFF) == 0)) {
			RX_PHASE = 0;
			UART1_ECR_R = 0xff;
			//deviceupdate();
			GREEN_LED ^= 1;
		} else {
			DMX_RX_DATA[RX_PHASE++] = (0xFF & RX_data);
			deviceupdate();
		}

	}
}

void valid_input() {
	if ((strcmp("set", command) == 0) || (strcmp("get", command) == 0)
			|| (strcmp("max", command) == 0)) {
		input_address();
	}
	if ((strcmp("set", command) == 0)) {
		input_values();
	}
}
int parseinput() {
	strcpy(command, strtok(IS, " "));
	strcpy(address, strtok(NULL, " "));
	strcpy(values, strtok(NULL, " "));

	int g;
	tolowerstring(command);
	input_string(command);
	if (strcmp("set", command) == 0) {
//input_address();
//input_values();
		if ((input_address() == true) && (input_values() == true)) {
			putsUart0("Command to process : set \n\r");
			BLUE_LED = 1;
			Timerconfig();
			DMX_VALUE[h] = l;
			sprintf(DMX_buffer, "The processed set command is: %d - > %d ", h,
					DMX_VALUE[h]);
			putsUart0(DMX_buffer);
			putsUart0("\n\rReady\n\r");
		}
	} else if (strcmp("get", command) == 0) {
//input_address();
		if ((input_address() == true)) {
			putsUart0("Command to process : get \n\r");
			BLUE_LED = 1;
			Timerconfig();
			sprintf(DMX_buffer, "The processed get command is: %d - > %d ", h,
					DMX_VALUE[h]);
			putsUart0(DMX_buffer);
			putsUart0("\n\rReady\n\r");
		}
	} else if (strcmp("clear", command) == 0) {
		putsUart0("Command to process: clear \n\r");
		BLUE_LED = 1;
		Timerconfig();
		for (g = 0; g <= max_address; g++) {
			DMX_VALUE[g] = 0;
			RED_LED = 0;
		}
		putsUart0("\n\rReady\n\r");
	} else if (strcmp("on", command) == 0) {
		if (TX_FLAG == true) {
			putsUart0("Transmission is in progress \n\r");
		} else {
			TX_FLAG = true;
			putsUart0("Transmission flag is set \n\r");
		}
		start_TX();
		putsUart0("Command to process: on \n\r");
		BLUE_LED = 1;
		Timerconfig();
		putsUart0("\n\rReady\n\r");
	} else if (strcmp("off", command) == 0) {
		TX_FLAG = false;
		UART1_CTL_R = UART1_CTL_R & ~UART_CTL_TXE;
		UART1_CTL_R = UART1_CTL_R & ~UART_CTL_RXE;
		putsUart0("Command to process: off \n\r");
		BLUE_LED = 1;
		Timerconfig();
		if (TX == 1) {
			RED_LED = 0;
		}

		putsUart0("\n\rReady\n\r");

	} else if (strcmp("max", command) == 0) {
//input_address();
		if ((input_address() == true)) {
			putsUart0("Command to process: max\n\r");
			max_address = h;
			sprintf(DMX_maxbuffer, "The max address is: %d ", max_address);
			putsUart0(DMX_maxbuffer);
			BLUE_LED = 1;
			Timerconfig();
			putsUart0("\n\rReady\n\r");
		}
	} else if (strcmp("poll", command) == 0) {
		poll_flag = 1;
		putsUart0("Command to process: poll \n\r");
		BLUE_LED = 1;
		Timerconfig();
		putsUart0("\n\rReady\n\r");
	} else {
		putsUart0("\n\rError in command");
		putsUart0("\n\rReady\n\r");
	}
	return 0;

}
void start_TX() {
	configureUart1for_100000();
	TX_PHASE = TX_STARTCODE;
	UART1_DR_R = 0x00;
}
int main(void) {
// Initialize hardware
	initHw();
	putsUart0("REKHA T RAJAN \n\r");
	putsUart0("EE-5314 PROJECT \n\r");
	BLUE_LED = 1;
	Timerconfig();
	getValue();
	//blinkGreenLED();
	TX_ISR();
	waitMicrosecond(1000 * 1000);
	if (MASTER_MODE == true) {
		TX = 1;
		DE = 1;
		while (1) {
			inputstring();
			if (strlen(IS) > 0) {

				parseinput();

			}
		}
	} else {
		RX = 1;
		DE = 0;
		configureUart1for_250000();
		while (1) {
			inputstring();
			if (strlen(IS) > 0) {
				parseinput();
			}
		}
	}
}
