/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010

 Application example: Robot control over serial port via USB-RS232 converter
 					  (located on the ATMEGA260 microcontroller adaptor board)

 Concepts covered:  serial communication

 Serial Port used: UART2

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.

 Pins for PWM are kept at logic 1.

 Connection Details:

  Motion control:		L-1---->PA0;		L-2---->PA1;
   						R-1---->PA2;		R-2---->PA3;
   						PL3 (OC5A) ----> Logic 1; 	PL4 (OC5B) ----> Logic 1;


  Serial Communication:	PORTD 2 --> RXD1 UART1 receive for RS232 serial communication
						PORTD 3 --> TXD1 UART1 transmit for RS232 serial communication

						PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
						PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication

						PORTE 0 --> RXD0 UART0 receive for ZigBee wireless communication
						PORTE 1 --> TXD0 UART0 transmit for ZigBee wireless communication

						PORTJ 0 --> RXD3 UART3 receive available on microcontroller expansion socket
						PORTJ 1 --> TXD3 UART3 transmit available on microcontroller expansion socket

Serial communication baud rate: 9600bps
To control robot use number pad of the keyboard which is located on the right hand side of the keyboard.
Make sure that NUM lock is on.

Commands:
			Keyboard Key   ASCII value	Action
				8				0x38	Forward
				2				0x32	Backward
				4				0x34	Left
				6				0x36	Right
				5				0x35	Stop
				7				0x37	Buzzer on
				9				0x39	Buzzer off

 Note:

 1. Make sure that in the configuration options following settings are
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0  (For more information read section: Selecting proper optimization
 					options below figure 2.22 in the Software Manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
 	Rest of the things are the same.

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

 4. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to
 	2 Ampere. When both motors of the robot changes direction suddenly without stopping,
	it produces large current surge. When robot is powered by Auxiliary power which can supply
	only 1 Ampere of current, sudden direction change in both the motors will cause current
	surge which can reset the microcontroller because of sudden fall in voltage.
	It is a good practice to stop the motors for at least 0.5seconds before changing
	the direction. This will also increase the useable time of the fully charged battery.
	the life of the motor.

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose.
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to:
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "dStar.h"

#include "lcd.h"
//#include "dStar.h"
//#include "adjacency.h"

//#define size 49
//#define INF 600000

unsigned char data; //to store received data from UDR1
signed int count = -1;
signed int arr_size = 0;
signed int *sequence_arr;
signed int slave_size = 0;
signed int arr_slave[20];
signed int Counter = 0;

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//Function to initialize ports
void port_init()
{
	buzzer_pin_config();
	lcd_port_config();
}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char arr_size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char arr_size: 8 bit
// parity: Disabled
void uart2_init(void)
{
 UCSR2B = 0x00; //disable while setting baud rate
 UCSR2A = 0x00;
 UCSR2C = 0x06;
 UBRR2L = 0x5F; //set baud rate lo
 UBRR2H = 0x00; //set baud rate hi
 UCSR2B = 0x98;
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

void remove_zero() {
	int i = 0, j = 1;
    arr_slave[0] = 13;
	while (i < arr_size)
	{
		if(sequence_arr[i]!=0)
		{
			arr_slave[j] = sequence_arr[i];
			j++;
		}
		i++;
	}
	slave_size = j;
}

void print() {
	int k = 1;
	int j = 1;
	for (int i = 0; i < slave_size; ++i)
	{
		if (j == 9)
		{
			j = 1;
			k = 2;
		}

		lcd_print(k,j,arr_slave[i],2);
		j += 2;
	}
}

void simulation(int from,int to) {
	dStar(from, to);
	while (sequence_arr[Counter] == 0)
	{
		//lcd_print(1,14,Counter,2);
		lcd_cursor(1,1);
		lcd_string("   Waiting!!!   ");
		lcd_cursor(2,1);
		lcd_string(" For Slave Bot! ");
		_delay_ms(100);
	}

	buzzer_on();
	lcd_cursor(1,1);
	lcd_string("MNP DETECTED ");
	lcd_print(1,14,sequence_arr[Counter],2);
	lcd_cursor(2,1);
	lcd_string("Strinking Node!!");
	_delay_ms(500);
	buzzer_off();

	++Counter;
	//lcd_print(1,4,Counter,2);
	UDR0 = Counter;
}

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
	UDR2 = data; 				//echo data back to PC
	if (count == -1)
	{
		_delay_ms(100);
		arr_size = (signed int) data;
		//arr_size -= 48;
		count++;
		sequence_arr = (signed int*) malloc(arr_size*sizeof(signed int));
	}

	else if (count < arr_size) {
		_delay_ms(100);
		sequence_arr[count] = (signed int) data;
		//sequence_arr[count] -= 48;
		count++;
	}

	else if (count > arr_size){
		Counter = (signed int) data;
		//lcd_print(1,15,Counter,2);
	}
}

/*SIGNAL(SIG_USART2_RECV) 		// ISR for receive complete interrupt
{
	data = UDR2; 				//making copy of data from UDR0 in 'data' variable

	UDR2 = data;
}*/

//Function To Initialize all The Devices
void init_devices()
{
 cli(); //Clears the global interrupts
 port_init();  //Initializes all the ports
 uart0_init(); //Initailize UART0 for serial communiaction
 uart2_init(); //Initailize UART1 for serial communiaction
 sei();   //Enables the global interrupts
}

//Main Function
int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	while(1) {
		if (count == arr_size)
		{
			count++;
			remove_zero();
			for (int i = 0; i < slave_size; i++) {
				simulation (arr_slave[i], arr_slave[i+1]);
			}
			while(1) {
				lcd_cursor(1,1);
				lcd_string("      Task      ");
				lcd_cursor(2,1);
				lcd_string("  Completed!!!  ");
			}
		}
		lcd_cursor(1,1);
		lcd_string("      Task      ");
		lcd_cursor(2,1);
		lcd_string("  Initiated!!!  ");
	}
}

