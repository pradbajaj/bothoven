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

/*
	*Team ID: eYRC-BV#1651
	*Author List: Aayush, Pradyumna, Pranjal, Shashwat
	*filename: master_mapRun.c
	*Theme: Bothoven
	*Functions: mapRun(), move()
	*Global Variable: IR_THRESHOLD, WL_THRESHOLD, VELOCITY_MAX, VELOCITY_MIN, VELOCITY_LOW, THRESHOLD,
						ShaftCountLeft, ShaftCountRight, Degrees, ADC_Values, flag, Left_white_line,
						Center_white_line, Right_white_line, AVG, senser_value_L, senser_value_C, senser_value_R,
						left_motor, right_motor, Front_ultrasonic_sensor, Front_IR_Sensor, data, arr_size, arr,
						count, sequence_arr, arr_slave, master_size, arr_master, Counter, update, arr_master_Counter
*/

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "mapRun.h"
#include "lcd.h"
#include "BFSPathFind.h"
#include "adjacency.h"


unsigned char data; //to store received data from UDR1
signed int arr_size = 0;
signed int *arr;
signed int count = -1;
signed int *sequence_arr;
signed int *arr_slave;
signed int master_size = 0;
signed int arr_master[20][2];
signed int Counter = 0;
signed int update; 
signed int arr_master_Counter = 1;

/*
	*********************************NOTE**************************************
	*The following functions configures the pins required for various stuff
	*like lcd_port, adc_pin etc
	**********************************END**************************************
*/

void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

/*
	*Function name: buzzer_pin_config()
	*Input: NIL
	*Output: NIL
	*Logic: Configures the pin used for buzzer
	*Example Call: int *cost = BFS(source);
*/

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
void port_initial()
{
	servo3_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	buzzer_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
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

void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}

void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
} 

/*
	*Function Name: seperate()
	*Input: NIL
	*Output: NIL
	*Logic: Divides the array recieved through python file (contained in arr) into sequence_arr (for master bot)
			and slave_arr (for slave bot). Sequence_arr has the same length as the the array recieved through python,
			the indices that have to be played by the slave have been replaced by 0s.
			In, slave_arr the indices that have to be played by the master has been replaced by 0s.
			This method allows for us to maintain sequence of plays.
	*example-call: seperate();
*/
void seperate() {
	for (int i = 0; i < arr_size; ++i)
	{
		if ((arr[i] >= 8 && arr[i] <= 18) || (arr[i] >= 28 && arr[i] <= 32))
		{
			arr_slave[i] = arr[i];
			_delay_ms(100);

			UDR0 = arr_slave[i];

			sequence_arr[i] = 0;
		}
		else {
			arr_slave[i] = 0;
			_delay_ms(100);

			UDR0 = arr_slave[i];

			sequence_arr[i] = arr[i];
		}
	}
}

/*
	*Function Name: remove_zero()
	*Input: NIL
	*Output: NIL
	*Logic: Removes the zeros from the seqence_arr which were inserted by seperate
			Converts the MNPs to corresponding nodes according to map_link
	*example-call: remove_zero();
*/
void remove_zero() {
	int i = 0, j = 1;
    arr_master[0][0] = 1;
    arr_master[0][1] = 24;
	while (i < arr_size)
	{
		if(sequence_arr[i]!=0)
		{
			arr_master[j][0] = map_link[sequence_arr[i]][0];
			j++;
		}
		i++;
	}
	master_size = j;
}

void print() {
	int k = 1;
	int j = 1;
	for (int i = 0; i < master_size; i++)
	{
		if (j == 17)
		{
			j = 1;
			k = 2;
		}

		lcd_print(k,j,arr_master[i][0],2);

		j += 2;
	}
}

/*
	*Function Name: strike()
	*Input: NIL
	*Output: NIL
	*Logic: rotates the servo motor to right, wait for 50 millisecond, rotate to left,
			wait for another 50 millisecond and finally comes at center position
	*example-call: strike();
*/
void strike(int previous_node) {
	unsigned char i = 0;
	lcd_cursor(1,1);
	lcd_string("MNP DETECTED ");
	lcd_print(1,14,sequence_arr[Counter],2);
	lcd_cursor(2,1);
	lcd_string("Strinking Node!!");

	if (strike_side[sequence_arr[Counter]][2] == 0)
	{
		// This means we have to strike either of the 3 hexagonal nodes
		//   So for that we will first move from current to current+1 
		//   then take a 180 degree turn and then strike
		int initial_angle, final_angle;
		for (int index = 0; index < 5; ++index) {
			if (map[map_link[sequence_arr[Counter]][0]][index] == map_link[sequence_arr[Counter]][0] + 1) {
				final_angle = map_angle[map_link[sequence_arr[Counter]][0]][index];
				break;
			}
		}
		for (int index = 0; index < 5; ++index) {
			if (map[previous_node][index] == map_link[sequence_arr[Counter]][0]) {
				initial_angle = map_angle[previous_node][index];
				break;
			}
		}
		signed int angle[] = {(final_angle - initial_angle),180};
		mapRun(angle,1);
		//Further we could add here the condition if their is an obstacle between say 25 and 26
		//so robot will try to travel to 37 and do the same thing
	}

	if (strike_side[sequence_arr[Counter]][0] == previous_node) {
		for (i = 105; i <210; i++)
		{
			servo_3(i);
			 _delay_ms(5);
		}
		_delay_ms(50);
		servo_3(105);
		servo_3_free();
	}
	else
	if (strike_side[sequence_arr[Counter]][1] == previous_node) {
		for (i = 105; i > 0; i--)
		{
			servo_3(i);
			 _delay_ms(5);
		}
		_delay_ms(50);
		servo_3(105);
		servo_3_free();
	}
}

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
	
	//UDR2 = data; 				//echo data back to PC

	update = (signed int) data;
	if (update < 100) {
		Counter = update;
	}
}

/*
	Here we are getting the array from python file and storing it in the arr
*/
SIGNAL(SIG_USART2_RECV) 		// ISR for receive complete interrupt
{
	data = UDR2; 				//making copy of data from UDR2 in 'data' variable

	UDR2 = data; 				//echo data back to PC

	if (count == -1)
	{
		arr_size = (signed int) data;
		arr_size -= 48;
		count++;
		// lcd_print(1,1,arr_size,2);
		arr = (signed int*) malloc(arr_size*sizeof(signed int));
		sequence_arr = (signed int*) malloc(arr_size*sizeof(signed int));
		arr_slave = (signed int*) malloc(arr_size*sizeof(signed int));
	}

	else if (count < arr_size) {
		arr[count] = (signed int) data;
		arr[count] -= 48;
		count++;
		// lcd_print(2,1,count+1,2);
	}
}

//Function To Initialize all The Devices
void initial_devices()
{
 	cli(); //Clears the global interrupts
 	port_initial();  //Initializes all the ports
 	adc_init();
 	timer1_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	uart0_init(); //Initailize UART0 for serial communiaction
	uart2_init(); //Initailize UART1 for serial communiaction
 	sei();   //Enables the global interrupts
}

/*
	*Function Name: simulation
	*Input: initial position and destination
	*Output: NIL
	*Logic: simulates (later changed to actually do it)
			the dStar and holding at node detection.
	*Example Call: simulation(1, 8);
*/
void simulation(int from, int to, int prefix) {

	signed int *success = BFSPathFind(from,to,prefix);

	while (sequence_arr[Counter] == 0)
	{
		stop();
		velocity(0,0);

		if(update > 100) {
			signed int *fesible_node = BFSPathFind(to, update/100,success[1]);
			if (fesible_node[0] == 0) {
				strike(fesible_node[1]);
				++Counter;
				UDR0 = Counter;
				success = BFSPathFind(update/100, to,fesible_node[1]);
			}
			else {
				++Counter;
				UDR0 = Counter;
				success = BFSPathFind(fesible_node[0], to,fesible_node[1]);
			}
			
			update = 0;
		}

		lcd_cursor(1,1);
		lcd_string("   Waiting!!!   ");
		lcd_cursor(2,1);
		lcd_string(" For Slave Bot! ");
		_delay_ms(100);
	}

	if (success[0] == 0) {
		strike(success[1]);
		arr_master[arr_master_Counter][1] = success[1];
		++arr_master_Counter;

		++Counter;
		UDR0 = Counter;
	} 
	else {
		signed int update_for_other = 0;
		arr_master[arr_master_Counter][0] = success[0];
		arr_master[arr_master_Counter][1] = success[1];

		update_for_other = to*100;
		UDR0 = update_for_other;	
	}
}

/*
	*Function Name: main
	*Input:	NIL
	*Output: Success or failure as in 0 for success
	*Logic: Runs the entire program.
	*Example Call: Automatic calling
*/
int main(void)
{
	initial_devices();
	lcd_set_4bit();
	lcd_init();
	initMap();
	servo_3(105);
	arr_size = 3;
	arr = (signed int*) malloc(arr_size*sizeof(signed int));
	sequence_arr = (signed int*) malloc(arr_size*sizeof(signed int));
	arr_slave = (signed int*) malloc(arr_size*sizeof(signed int));
	arr[0] = 3;
	arr[1] = 26;
	arr[2] = 23;
	while(1) {
		if (count == arr_size)
		{
			lcd_cursor(1,1);
			lcd_string("      Array     ");
			lcd_cursor(2,1);
			lcd_string("   Recieved!!!  ");
			_delay_ms(500);
			count++;
			UDR0 = arr_size;
			seperate();
			remove_zero();
			for (int i = 0; i < master_size - 1; i++) {
			lcd_cursor(1,1);
			lcd_string("   Waiting001   ");
				simulation (arr_master[i][0], arr_master[i+1][0], arr_master[i][1]);	//Converted into nodes while removing zeros
			}
			for ( int i = 0; i < 20; i++) {
				buzzer_on();
				_delay_ms(200);
				buzzer_off();
				_delay_ms(50);
			}
			while(1) {
				stop();
				velocity(0,0);
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
		count = 3;
		_delay_ms(1000);
	}
}

