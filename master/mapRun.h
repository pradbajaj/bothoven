/************************************************************************************
Written by: Vinod Desai,Sachitanand Malewar NEX Robotics Pvt. Ltd.
Edited by: e-Yantra team
AVR Studio Version 6
Date: 19th October 2012
 
 This experiment demonstrates the application of a simple line follower robot. The 
 robot follows a WHITE line over a BLACK backround
 
 Concepts covered:  ADC, LCD interfacing, motion control based on sensor data
 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4
 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PF0		Battery Voltage
			  1			PF1		WHITE line sensor 3
			  2			PF2		WHITE line sensor 2
			  3			PF3		WHITE line sensor 1
			  4			PF4		IR Proximity analog sensor 1*****
			  5			PF5		IR Proximity analog sensor 2*****
			  6			PF6		IR Proximity analog sensor 3*****
			  7			PF7		IR Proximity analog sensor 4*****
			  8			PK0		IR Proximity analog sensor 5
			  9			PK1		Sharp IR range sensor 1
			  10		PK2		Sharp IR range sensor 2
			  11		PK3		Sharp IR range sensor 3
			  12		PK4		Sharp IR range sensor 4
			  13		PK5		Sharp IR range sensor 5
			  14		PK6		Servo Pod 1
			  15		PK7		Servo Pod 2
 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
 
 Motion control Connection:
 			L-1---->PA0;		L-2---->PA1;
   			R-1---->PA2;		R-2---->PA3;
   			PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 
 
 LCD Display interpretation:
 ****************************************************************************
 *LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		BLANK			*
 *BLANK				BLANK				BLANK				BLANK			*
 ****************************************************************************
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code
 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)
 2. Make sure that you copy the lcd.c file in your folder
 3. Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor
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
	*filename: mapRun.c
	*Theme: Bothoven
	*Functions: mapRun(), move()
	*Global Variable: NIL
*/
#ifndef __MAP_RUN__
#define __MAP_RUN__

// #include "BFSPathFind.h"
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
// #include "lcd.h"

#define		IR_THRESHOLD	100
#define		WL_THRESHOLD	90
#define		VELOCITY_MAX	175
#define		VELOCITY_MIN	125
#define 	VELOCITY_LOW	0
#define		Threshold 		40		//Defining Threshold value of black line
									// Sensor Value less than threshold will be considered white line

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void move();

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
signed int AVG = 0;
signed int senser_value_L,senser_value_C,senser_value_R ;
int left_motor = 0, right_motor = 0;
unsigned char Front_ultrasonic_Sensor=0;
unsigned char Front_IR_Sensor=0;

/*
	*********************************NOTE**************************************
	*The following functions configures the pins required for various stuff
	*like lcd_port, adc_pin etc
	**********************************END**************************************
*/

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	buzzer_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);

	return ADC_Value;
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) 
{
  motion_set (0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void)
{
  motion_set (0x00);
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		move();
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}

void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}

/*
	*Function Name: move
	*Input: NIL
	*Output: NIL
	*Logic: Get the values of white line sensors and moves the firebird accordingly
	*Example Call: move();
*/

void move()
{
	senser_value_L = ADC_Conversion(3);	//Getting data of Left WL Sensor
	senser_value_C = ADC_Conversion(2);	//Getting data of Center WL Sensor
	senser_value_R = ADC_Conversion(1);	//Getting data of Right WL Sensor

	// If black line is at left sensor and center sensor is not decting a black line
	// Speed up right motor a bit to get a left turn
	// Further set the flag to be equal to '1' suggesting that bot has taken left turn this time
	if ((senser_value_L > Threshold) && (senser_value_C < Threshold))
	{
		forward();
		left_motor = 125;
		right_motor = 250;
		velocity(left_motor,right_motor);
		flag = 1;
	}
	// If black line is at right sensor and center sensor is not decting a black line
	// Speed up left motor a bit to get a right turn
	// Further set the flag to be equal to '2' suggesting that bot has taken right turn this time
	else if ((senser_value_R > Threshold) && (senser_value_C < Threshold))
	{
		forward();
		left_motor = 250;
		right_motor = 125;
		velocity(left_motor,right_motor);
		flag = 2;
	}
	// If black line is at center sensor and left & right sensors are not decting a black line
	// Speed up both motors equally to get a straight movement
	else if ((senser_value_C > Threshold) && (senser_value_L < Threshold) && (senser_value_R < Threshold))
	{
		forward();
		left_motor = 250;
		right_motor = 250;
		velocity(left_motor, right_motor);
	}
	// If black line is nowhere to be found
	// Speed up right or left motor according to the flag set
	else if ((senser_value_R < Threshold) && (senser_value_C < Threshold) && (senser_value_L < Threshold))
	{
		forward();
		left_motor = 100;
		right_motor = 250;
		if (flag == 1) {
			velocity(left_motor, right_motor);
		}
		else {
			velocity(right_motor, left_motor);
		}
	}
}

/*
	*Function Name: mapRun()
	*Input: NIL
	*Output: 0 on successful completion
	*Logic: Given a array of nodes, executes dStar to find the best path
			to touch all the nodes and follows it.
*/
int* mapRun(signed int angle[], int Size_)
{
	// after dectecting the note bot will turn angle[i] angle
	signed int count = -1;

	int *res = (int*) malloc (3*sizeof(int));		//Holds result
	for (int i = 0; i < 3; i++)
		res[i] = 0;

	while(count < Size_)
	{
		int flag = 0;

		Front_IR_Sensor = ADC_Conversion(11);    //Getting data of Center IR Proximity Sensor Sensor

		// If their an object with in 9 cm range of IR sensor
		// LCD will print "OBSTRACLE DETECTED !"
		// Bot will take an initial right turn to shift from the black line
		// Then bot will take right turn until the center whiteline sensor is on top of black line
		if (Front_IR_Sensor < 90)
		{
			lcd_cursor(1,1);
			lcd_string("    OBSTRACLE   ");
			lcd_cursor(2,1);
			lcd_string("    DETECTED !  ");
			right();
	        _delay_ms(300);
	        back();
	        _delay_ms(200);
	        do
	        {
	          right();
	          Center_white_line = ADC_Conversion(2);
	        }
	        while (Center_white_line < 75);
	        stop();
	        _delay_ms(100);
			res[0] = 1;
			res[1] = count;
			res[2] = count+1;
			return res;
		}

		// If center center plus left or right sensor detects the black line
		// The bot is over a node
		if ((senser_value_C > 50) && (senser_value_L > 50) || (senser_value_C > 50) && (senser_value_R > 50))
		{
			lcd_cursor(2,1);
			lcd_string("NOTE DETECTED!!!");
			
			count++;
			stop();

			//if angle[i] is 60 the bot has to take a 60 degree turn
			// bot will move 6 cm ahead and 30 degree left to skip the current black line
			// then it will rotate left until it finds another black line
			// lcd will print "60 degree success"
			if (angle[count] == 60 || angle[count] == -300)
			{
				velocity(250,250);
				forward_mm(60);
		        _delay_ms(100);

		        left_degrees(30);
		        do
		        {
		          left();
		          senser_value_C = ADC_Conversion(2);
		        }
		        while (senser_value_C < 75);
		        stop();
		        lcd_cursor(1,1);
		        lcd_string("60 degree succes");
			}
			//if angle[i] is 120 the bot has to take a 120 degree turn
			// bot will move 1.5 cm ahead and 30 degree back left to skip the current black line
			// then it will rotate soft left until it finds another black line
			// lcd will print "120 degree success"
			else if (angle[count] == 120 || angle[count] == -240)
			{
				velocity(250,250);
				forward();
		        _delay_ms(350);
		        left_degrees(90);
		        _delay_ms(500);
		        do
		        {
		          left();
		          senser_value_C = ADC_Conversion(2);
		        }
		        while (senser_value_C < 75);
		        stop();
		        lcd_cursor(1,1);
		        lcd_string("120 degree succes");
			}
			//if angle[i] is -60 the bot has to take a -60 degree turn
			// bot will move 6 cm ahead and 30 degree right to skip the current black line
			// then it will rotate rihgt until it finds another black line
			// lcd will print "-60 degree success"
			else if (angle[count] == -60 || angle[count] == 300)
			{
				velocity(250,250);
				forward_mm(60);
		        _delay_ms(100);
		        right_degrees(30);
		        do
		        {
		          right();
		          senser_value_C = ADC_Conversion(2);
		        }
		        while (senser_value_C < 75);
		        stop();
		        lcd_cursor(1,1);
		        lcd_string("-60 degree suces");
			}
			//if angle[i] is -120 the bot has to take a -120 degree turn
			// bot will move 1.5 cm ahead and 30 degree back right to skip the current black line
			// then it will rotate soft right until it finds another black line
			// lcd will print "-120 degree success"
			else if (angle[count] == -120 || angle[count] == 240)
			{
				velocity(250,250);
				forward();
		        _delay_ms(350);
		        right_degrees(90);
		        _delay_ms(500);
		        do
		        {
		          right();
		          senser_value_C = ADC_Conversion(2);
		        }
		        while (senser_value_C < 75);
		        stop();
		        lcd_cursor(1,1);
		        lcd_string("120 degree succes");
			}
			//if angle[i] is 180 the bot has to take a 190 degree turn
			// bot will move left for 500 milliseconds to skip the current black line
			// then it will rotate left until it finds another black line
			else if (angle[count] == 180 || angle[count] == -180)
			{
				velocity(250,250);
				forward();
				right();
		        _delay_ms(300);
		        back();
		        _delay_ms(200);
		        do
		        {
		          right();
		          Center_white_line = ADC_Conversion(2);
		        }
		        while (Center_white_line < 75);
		        stop();
		        _delay_ms(100);
			}

			// by default lcd will be print "MOVING ON FLEX"
			lcd_cursor(1,1);
			lcd_string("                ");
			lcd_cursor(2,1);
			lcd_string("MOVING ON FLEX!!");
		}
		if (count < Size_)
		{
			move();// calling the move function
		}
		
	}
	return res;
}

#endif			//__MAP_RUN__
