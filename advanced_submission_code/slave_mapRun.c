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
//#include "dstar.h"
#include "lcd.h"
//#include "dStar.h"
//#include "adjacency.h"

//#define size 49
//#define INF 600000

#define		IR_THRESHOLD	100
#define		WL_THRESHOLD	90
#define		VELOCITY_MAX	175
#define		VELOCITY_MIN	125
#define 	VELOCITY_LOW	0
#define		Threshold 		40		//Defining Threshold value of black line

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


unsigned char data; //to store received data from UDR1
signed int count = -1;
signed int arr_size = 0;
signed int *sequence_arr;
signed int slave_size = 0;
signed int arr_slave[20];
signed int Counter = 0;

void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

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

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01; //Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03; //Output compare Register high value for servo 1
 OCR1AL = 0xFF; //Output Compare Register low Value For servo 1
 OCR1BH = 0x03; //Output compare Register high value for servo 2
 OCR1BL = 0xFF; //Output Compare Register low Value For servo 2
 OCR1CH = 0x03; //Output compare Register high value for servo 3
 OCR1CL = 0xFF; //Output Compare Register low Value For servo 3
 ICR1H  = 0x03; 
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
          For Overriding normal port functionality to OCRnA outputs.
          {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

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

void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
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
		left_motor = 200;
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
		right_motor = 200;
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
		left_motor = 170;
		right_motor = 250;
		if (flag == 1) {
			velocity(left_motor, right_motor);
		}
		else {
			velocity(right_motor, left_motor);
		}
	}
}

void mapRun(signed int angle[], int Size_)
{
	//init_devices();
	//lcd_set_4bit();
	//lcd_init();
	
	// after dectecting the note bot will turn angle[i] angle
	signed int count = -1;
	/*int *res = (int*) malloc (3*sizeof(int));		//Holds result
	for (int i = 0; i < 3; i++)
		res[i] = 0;*/

	while(count < Size_)
	{
		int flag = 0;

		Front_IR_Sensor = ADC_Conversion(6);    //Getting data of Center IR Proximity Sensor Sensor

		// If their an object with in 9 cm range of IR sensor
		// LCD will print "OBSTRACLE DETECTED !"
		// Bot will take an initial right turn to shift from the black line
		// Then bot will take right turn until the center whiteline sensor is on top of black line
		// if (Front_IR_Sensor < 90)
		// {
		// 	lcd_cursor(1,1);
		// 	lcd_string("    OBSTRACLE   ");
		// 	lcd_cursor(2,1);
		// 	lcd_string("    DETECTED !  ");
		// 	right();
	 //        _delay_ms(300);
	 //        back();
	 //        _delay_ms(200);
	 //        do
	 //        {
	 //          right();
	 //          Center_white_line = ADC_Conversion(2);
	 //        }
	 //        while (Center_white_line < 110);
	 //        stop();
	 //        _delay_ms(100);
		// 	//res[0] = 1;
		// 	//res[1] = count;
		// 	//res[2] = count+1;
		// 	//return res;
		// }



		// If center center plus left or right sensor detects the black line
		// The bot is over a node
		if ((senser_value_C > 2*Threshold) && (senser_value_L > 2*Threshold) || (senser_value_C > 2*Threshold) && (senser_value_R > 2*Threshold))
		{
			lcd_cursor(2,1);
			lcd_string("NOTE DETECTED!!!");
			// move while encoders certain value reach
			
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
		        while (senser_value_C < 110);
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
		        lcd_string("took turn");
		        do
		        {
		          left();
		          senser_value_C = ADC_Conversion(2);
		        }
		        while (senser_value_C < 110);
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
		        while (senser_value_C < 110);
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
		        lcd_string("took turn");
		        do
		        {
		          right();
		          senser_value_C = ADC_Conversion(2);
		        }
		        while (senser_value_C < 110);
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
		        while (Center_white_line < 110);
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
	//return res;
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

void simulation(int angle[], int size) {

	mapRun(angle, size);
	while (sequence_arr[Counter] == 0)
	{
		//lcd_print(1,14,Counter,2);
		stop();
		velocity(0,0);
		lcd_cursor(1,1);
		lcd_string("   Waiting!!!   ");
		lcd_cursor(2,1);
		lcd_string(" For Master Bot!");
		_delay_ms(100);
	}

	// velocity(0, 0);
	//buzzer_on();
	lcd_cursor(1,1);
	lcd_string("MNP DETECTED ");
	lcd_print(1,14,sequence_arr[Counter],2);
	lcd_cursor(2,1);
	lcd_string("Strinking Node!!");
	//_delay_ms(500);
	//buzzer_off();

	unsigned char i = 0;
	for (i = 105; i <210; i++)
		{
			servo_3(i);
			 _delay_ms(5);
		}
	_delay_ms(50);
	for (i = 210; i >0; i--)
		{
			 servo_3(i);
			 _delay_ms(5);
		}
 	 _delay_ms(50);

	for (i = 0; i <105; i++)
		{
			servo_3(i);
			_delay_ms(10);
		}
	// velocity(250, 250);

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
//Main Function
int main(void)
{
	initial_devices();
	lcd_set_4bit();
	lcd_init();
	servo_3(105);
	int angle_mat[6][11] = {{0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,0},
							{180,-60,0,60,0,-120,0,60,-60,0,0},
							{180,60,-60,0,120,0,-60,0,-120,0,0},
							{180,0,0,0,0,0,0,0,0,0,0},
							{120,0,60,0,-120,0,60,-60,0,0,0}};
	int angle_size[6] = {-1,2,8,10,2,7};
	while(1) {
		if (count == arr_size)
		// if (count == -1)
		{
			lcd_cursor(1,1);
			lcd_string("      Array     ");
			lcd_cursor(2,1);
			lcd_string("   Recieved!!!  ");
			_delay_ms(2000);	
			count++;
			remove_zero();
			//print();
			//_delay_ms(10000);
			slave_size = 6;
			for (int i = 0; i < slave_size; i++) {
				simulation (angle_mat[i], angle_size[i]);
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
	}
}