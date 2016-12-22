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
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.h"

#define		IR_THRESHOLD	100
#define		WL_THRESHOLD	90
#define		VELOCITY_MAX	175
#define		VELOCITY_MIN	125
#define 	VELOCITY_LOW	0
#define 	BLACK			0x5A
#define 	WHITE			10
#define		Threshold 		40

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void move(int[][], int);

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

void move(int[][], int)
{
	senser_value_L = ADC_Conversion(3);	//Getting data of Left WL Sensor
	senser_value_C = ADC_Conversion(2);	//Getting data of Center WL Sensor
	senser_value_R = ADC_Conversion(1);	//Getting data of Right WL Sensor

	if ((senser_value_L > Threshold) && (senser_value_C < Threshold))
	{
		forward();
		left_motor = 200;
		right_motor = 255;
		velocity(left_motor,right_motor);
		flag = 1;
	}
	else if ((senser_value_R > Threshold) && (senser_value_C < Threshold))
	{
		forward();
		left_motor = 255;
		right_motor = 200;
		velocity(left_motor,right_motor);
		flag = 2;
	}
	else if ((senser_value_C > Threshold) && (senser_value_L < Threshold) && (senser_value_R < Threshold))
	{
		forward();
		left_motor = 255;
		right_motor = 255;
		velocity(left_motor, right_motor);
	}
	else if ((senser_value_R < Threshold) && (senser_value_C < Threshold) && (senser_value_L < Threshold))
	{
		forward();
		left_motor = 170;
		right_motor = 255;
		if (flag == 1) {
			velocity(left_motor, right_motor);
		}
		else {
			velocity(right_motor, left_motor);
		}
	}
}

//Main Function
int main()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	//int n = 8; // nummber of angles
	signed int angle[] = {0,-60,0,60,0,-1,-120,0,-1,-120,-1,120,-60,-60,120,120,0,120,0,-60,0,60,-1,180,-60,0,60,0,-60,-1,180,60,0,60,-60,-60,120,-120,0,120,0,-1,0,0,180,0,0,0,-120,0,-60,-1,180,60,0,-60,-1,0,0,0,0,-1}; // after dectecting the note bot will turn angle[i] angle
	//signed int angle[] = {120,0}; // after dectecting the note bot will turn angle[i] angle
	signed int count = -1;

	while(count < 61)
	{
		int flag = 0;

		Front_IR_Sensor = ADC_Conversion(6);    //Getting data of Center IR Proximity Sensor Sensor

		if (Front_IR_Sensor < 90)
		{
			lcd_cursor(1,1);
			lcd_string("    OBSTRACLE   ");
			lcd_cursor(2,1);
			lcd_string("    DETECTED !  ");
			right();
			_delay_ms(100);
			do
			{
				right();
				Center_white_line = ADC_Conversion(2);
			}
			while (Center_white_line < 0x50);
			stop();
			_delay_ms(100);
			forward();
		}

		move();

		if ((senser_value_C > Threshold) && (senser_value_L > Threshold))
		{
			lcd_cursor(2,1);
			lcd_string("NOTE DETECTED!!!");
			// move while encoders certain value reach
			
			count++;
			if (angle[count] == -1)
			{
				buzzer_on();
				stop();
				velocity(0,0);
				lcd_cursor(2,1);
				lcd_string("MNP DETECTED !!!");
				_delay_ms(500);
				buzzer_off();
				count++;
			}

			if (angle[count] == 60)
			{
				forward_mm(60);
				_delay_ms(100);
				left_degrees(30);
				do
				{
					left();
					senser_value_C = ADC_Conversion(2);
				}
				while (senser_value_C < 110);
				stop();
				lcd_cursor(1,1);
				lcd_string("60 degree succes");
			}
			else if (angle[count] == 120)
			{
				forward_mm(15);
				_delay_ms(100);
				soft_left_2_degrees(30);
				do
				{
					soft_left();
					senser_value_C = ADC_Conversion(2);
				}
				while (senser_value_C < 110);
				stop();
				lcd_cursor(1,1);
				lcd_string("120 degree suces");
			}
			else if (angle[count] == -60)
			{
				forward_mm(60);
				_delay_ms(100);
				right_degrees(30);
				do
				{
					right();
					senser_value_C = ADC_Conversion(2);
				}
				while (senser_value_C < 110);
				stop();
				lcd_cursor(1,1);
				lcd_string("-60 degree suces");
			}
			else if (angle[count] == -120)
			{
				forward_mm(15);
				_delay_ms(100);
				soft_right_2_degrees(30);
				do
				{
					soft_right();
					senser_value_C = ADC_Conversion(2);
				}
				while (senser_value_C < 110);
				stop();
				lcd_cursor(1,1);
				lcd_string("-120 degre suces");
			}
			else if (angle[count] == 180)
			{
				left();
				_delay_ms(500);
				do
				{
					left();
					Center_white_line = ADC_Conversion(2);
				}
				while (Center_white_line < 110);
				stop();
				_delay_ms(100);
				forward();
			}

			lcd_cursor(1,1);
			lcd_string("                ");
			lcd_cursor(2,1);
			lcd_string("MOVING ON FLEX!!");
		}
		else if ((senser_value_C > Threshold) && (senser_value_R > Threshold))
		{
			lcd_cursor(2,1);
			lcd_string("NOTE DETECTED!!!");
			// move while encoders certain value reach
			
			count++;
			if (angle[count] == -1)
			{
				buzzer_on();
				lcd_cursor(2,1);
				stop();
				velocity(0,0);
				lcd_string("MNP DETECTED !!!");
				_delay_ms(500);
				buzzer_off();
				count++;
			}

			if (angle[count] == 60)
			{
				forward_mm(60);
				_delay_ms(100);
				left_degrees(30);
				do
				{
					left();
					senser_value_C = ADC_Conversion(2);
				}
				while (senser_value_C < 110);
				stop();
				lcd_cursor(1,1);
				lcd_string("60 degree succes");
			}
			else if (angle[count] == 120)
			{
				forward_mm(15);
				_delay_ms(100);
				soft_left_2_degrees(30);
				do
				{
					soft_left();
					senser_value_C = ADC_Conversion(2);
				}
				while (senser_value_C < 110);
				stop();
				lcd_cursor(1,1);
				lcd_string("120 degree suces");
			}
			else if (angle[count] == -60)
			{
				forward_mm(60);
				_delay_ms(100);
				right_degrees(30);
				do
				{
					right();
					senser_value_C = ADC_Conversion(2);
				}
				while (senser_value_C < 110);
				stop();
				lcd_cursor(1,1);
				lcd_string("-60 degree suces");
			}
			else if (angle[count] == -120)
			{
				forward_mm(15);
				_delay_ms(100);
				soft_right_2_degrees(30);
				do
				{
					soft_right();
					senser_value_C = ADC_Conversion(2);
				}
				while (senser_value_C < 110);
				stop();
				lcd_cursor(1,1);
				lcd_string("-120 degre suces");
			}
			else if (angle[count] == 180)
			{
				left();
				_delay_ms(500);
				do
				{
					left();
					Center_white_line = ADC_Conversion(2);
				}
				while (Center_white_line < 110);
				stop();
				_delay_ms(100);
				forward();
			}

			lcd_cursor(1,1);
			lcd_string("                ");
			lcd_cursor(2,1);
			lcd_string("MOVING ON FLEX!!");
		}
	
		//print_sensor(2,3,3);	//Prints value of WHITE Line Sensor1
		//print_sensor(2,7,2);	//Prints Value of WHITE Line Sensor2
		//print_sensor(2,11,1);	//Prints Value of WHITE Line Sensor3
		//print_sensor(2,1,6);	//Prints Value of IR sensor
		//lcd_print(2,1,pid,3);
		//lcd_print(1,1,senser_value_1,2);
		//lcd_print(1,4,senser_value_2,2);
		//lcd_print(1,7,senser_value_3,2);
		//lcd_print(1,10,senser_value_4,2);
		//lcd_print(1,13,senser_value_5,2);
		//lcd_print(2,1,senser_value_6,2);
		//lcd_print(2,4,senser_value_7,2);
		//lcd_print(2,7,senser_value_L+50,2);
		//lcd_print(2,10,senser_value_C+50,2);
		//lcd_print(2,13,senser_value_R+50,2);
		//lcd_print(2,5,left_motor,3);
		//lcd_print(2,9,right_motor,3);
		//lcd_cursor(1,1);
		//lcd_string("Pradyumna Aayush");
		//lcd_cursor(2,1);
		//lcd_string("Shashwat Pranjal");
	}
	for (int i = 0; i < 20; ++i)
	{
		stop();
		velocity(0,0);
		buzzer_on();
		_delay_ms(100);
		buzzer_off();
		_delay_ms(150);
	}
	while(1)
	{
		stop();
		velocity(0,0);
		lcd_cursor(1,1);
		lcd_string("      Task      ");
		lcd_cursor(2,1);
		lcd_string("  Completed!!!  ");
	}
}
