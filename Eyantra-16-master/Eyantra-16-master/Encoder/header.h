
/*
* Team Id: 1320
* Author List: Daniel James, Amrith M, Sarathchandran S, John Sebastian
* Filename: header.h
* Theme: Bothoven
* Functions:  motion_pin_config(),buzzer_pin_config(),buzzer_on(),buzzer_off(),init_timer5(),Sharp_dist(int),lcd_port_config(),left_encoder_pin_config(),right_encoder_pin_config(),right_position_encoder_interrupt_init(),left_position_encoder_interrupt_init()
			  motion_set(char),forward(),back(),left(),right(),soft_left(),soft_right(),angle_rotate(int),linear_distance_mm(int),forward_mm(int),back_mm(int),left_degrees(int),right_degrees(int),soft_right_degrees(int),soft_left_degrees(int),adc_pin_config()
			  adc_init(),velocity(int,int),port_init(),init_devices(),ADC_Conversion(char),print_sensor(char,char,int)
* Global Variables: ADC_Value,Left_white_line,Right_white_line,Center_white_line,ShaftCountLeft,ShaftCountRight,Degrees,DistanceShaft
*/

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>		//Remove this if program doesn't run. 
#define THRESHOLD 20		
//THRESHOLD : ADC Value decided as threshold for sensing black line. Value above threshold corresponds to a black line. Threshold may change for track and lighting
#define MAX_VEL 200

#define AVG_VEL 200

#define MIN_VEL 0

unsigned char ADC_Conversion(unsigned char);

unsigned char ADC_Value;

unsigned int value;

unsigned char Left_white_line = 0;
//Left_white_line : For storing the ADC value of left white line sensor
unsigned char Center_white_line = 0;
//Center_white_line : For storing the ADC value of center white line sensor
unsigned char Right_white_line = 0;
//Right_white_line : For storing the ADC value of right white line sensor

volatile unsigned long int ShaftCountLeft = 0; 
//ShaftCountLeft : to keep track of left position encoder 
volatile unsigned long int ShaftCountRight = 0; 
//ShaftCountRight: to keep track of right position encoder
volatile unsigned int Degrees; 
//Degrees: to accept angle in degrees for turning
volatile unsigned long int distanceShaft = 0;
//distanceShaft : To 

/*
* Function Name:	motipn_pin_config(void)
* Input:			None
* Output:			Initialize the required registers
* Logic:			Initialization : Function to configure ports to enable robot's motion (Port A and Port L here where the motors are connected and where pwm is generated)
* Example Call:		motion_pin_config()
*/

void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
* Function Name:	buzzer_pin_config(void)
* Input:			None
* Output:			Initialize the required registers
* Logic :			Initialization : Function to configure ports to use buzzer pin
* Example Call:		buzzer_on()
*/

void buzzer_pin_config(void)
{
	DDRC=DDRC | 0x08; 		// Port C pin 3 as output
	PORTC=PORTC & 0xF7;

}

/*
* Function Name:	buzzer_on(void)
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turns the pin connected to the buzzer to high state so that sound is produced
* Example Call:		buzzer_on()
*/

void buzzer_on(void)
{
	
	PORTC= 0x08;   // pin 3 to high 0000 1000
}

/*
* Function Name:	buzzer_off(void)
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turns the pin connected to the buzzer to low state so buzzer produces no sound
* Example Call:		buzzer_off()
*/

void buzzer_off(void)
{
	PORTC= 0x00;
}

/*
* Function Name:	init_timer5(void)
* Input:			None
* Output:			Initialize the required registers
* Logic :			Configuring the timer5 registers in Atmega2560 for using PWM functionalities
* Example Call:		init_timer5()
*/

void init_timer5(void)	//Timer For PWM
{
	TCCR5B = 0x00;	//stop
	TCNT5H = 0xFF;	//counter higher 8 bit value to which OCRxH is compared with
	TCNT5L = 0x01;	//counter higher 8 bit value to which OCRxH is compared with
	OCR5AH = 0x00;	//Output compare register high for left motor
	OCR5AL = 0xFF;	//Output compare register low for left motor
	OCR5BH = 0x00;	//Output compare register high for right motor	
	OCR5BL = 0xFF;	//Output compare register high for right motor
	OCR5C  = 0xFF;	//Motor C1
	OCR5CH = 0x00;	//Motor C1
	OCR5CL = 0xFF;
	TCCR5A = 0xA9;

/* COM5A1=1,COM5A0=0,COM5B1=1,COM5B0=0,COM5C1=1,COM5C0=0
For Overriding normal port functionality to OCRnA ouputs
WGM51=0,WGM50=1 along with WGM52 in TCCRB for selecting fast PWM 8 bit mode
*/

	TCCR5B = 0x0B;	//WGM12=1,CS12=0,CS11=1,CS10=1	(Prescaler=64)

}

/*
* Function Name:	Sharp_dist
* Input:			ADC Reading of front IR Sharp Sensor
* Output:			Distance in mm of Sharp sensor from the nearest detected obstacle
* Logic :			Calculating distance from volatage value of the sensor using the formula in sensor datasheet
* Example Call:		Sharp_dist(20)
*/

unsigned int Sharp_dist(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

/*
* Function Name:	lcd_port_config
* Input:			None
* Output:			Initialize the required registers
* Logic :			Configuring the LCD Pins to function
* Example Call:		lcd_port_config()
*/

void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

/*
* Function Name:	left_encoder_pin_config
* Input:			None
* Output:			Initialize the required registers
* Logic :			Function to configure INT4 (PORTE 4) pin as input for the left position encoder
* Example Call:		left_encoder_pin_config()
*/

void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name:	right_encoder_pin_config
* Input:			None
* Output:			Initialize the required registers
* Logic :			Function to configure INT5 (PORTE 5) pin as input for the right position encoder
* Example Call:		right_encoder_pin_config()
*/

void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name:	left_position_encoder_interrupt_init
* Input:			None
* Output:			Initialize the required registers
* Logic :			Initializing the interrput pins where left encoder is connected
* Example Call:		left_position_encoder_interrupt_init()
*/

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

/*
* Function Name:	right_position_encoder_interrupt_init
* Input:			None
* Output:			Initialize the required registers
* Logic :			Initializing the interrput pins where right encoder is connected
* Example Call:		right_position_encoder_interrupt_init()
*/

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

/*
* Function Name:	ISR(Interrupt Service Routine)
* Input:			None
* Output:			Initialize the required registers
* Logic :			ISR for right position encoder. Here counting the number of times the circle cuts the encoder
* Example Call:		called by the microcontroller when the interrupt receives a signal
*/

ISR(INT5_vect)  
{
 distanceShaft++;
 ShaftCountRight++;  //increment right shaft position count
}

/*
* Function Name:	ISR(Interrupt Service Routine)
* Input:			None
* Output:			Initialize the required registers
* Logic :			ISR for left position encoder. Here counting the number of times the circle cuts the encoder
* Example Call:		called by the microcontroller when the interrupt receives a signal
*/

ISR(INT4_vect)
{
 distanceShaft++;
 ShaftCountLeft++;  //increment left shaft position count
}

/*
* Function Name:	motion_set
* Input:			Direction - (character-Hexadecimal equivalent of the motor port configuration)
* Output:			Initialize the required registers
* Logic :			Function used for setting motor's direction. Robot will start moving in the specified direction once motion_set is called
* Example Call:		motion_set(0x00)
*/

void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;
 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

/*
* Function Name:	forward
* Input:			None
* Output:			Initialize the required registers
* Logic :			Calls the motion_set function with the hexadecimal value of the configuration which moves the motors forward
* Example Call:		forward()
*/

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

/*
* Function Name:	back
* Input:			None
* Output:			Initialize the required registers
* Logic :			Calls the motion_set function with the hexadecimal value of the configuration which moves the motors backward
* Example Call:		back()
*/

void back (void) //both wheels backward
{
  motion_set(0x09);
}

/*
* Function Name:	left
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turn the robot to left direction about the center(differential):Calls the motion_set function with the hexadecimal value of the configuration which moves the Left wheel backward, Right wheel forward
* Example Call:		left()
*/

void left (void)	//Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

/*
* Function Name:	right
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turn the robot to right direction about the center(differential):Calls the motion_set function with the hexadecimal value of the configuration which moves the Left wheel forward, Right wheel backward
* Example Call:		right()
*/

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

/*
* Function Name:	soft_left
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turn the robot to left:Calls the motion_set function with the hexadecimal value of the configuration which moves the Left wheel Stationary, Right wheel forward
* Example Call:		soft_left()
*/

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

/*
* Function Name:	soft_right
* Input:			None
* Output:			Initialize the required registers
* Logic :			Turn the robot to right:Calls the motion_set function with the hexadecimal value of the configuration which moves the Left wheel Forward, Right wheel Stationary
* Example Call:		soft_right()
*/

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

/*
* Function Name:	stop
* Input:			None
* Output:			Initialize the required registers
* Logic:			Stop the motors. Set all motor states to low by calling the motion_set function with 0 value
* Example Call:		stop()
*/

void stop (void)
{
  motion_set(0x00);
}

/*
* Function Name:	angle_rotate
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic :			For turning robot by specified degrees using encoders. When the shaftCount varibles reach the threshold, the motion is stopped
* Example Call:		angle_rotate(30)
*/

void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;
 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 
 while (1)
 {
  //lcd_print(2,8,ShaftCountLeft,2);
  //lcd_print(2,5,ShaftCountRight,2);
  //lcd_print(2,1,ReqdShaftCountInt,3);
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  			break;
 }
 stop(); //Stop robot
}

/*
* Function Name:	linear_distance_mm
* Input:			DistanceInMM(Integer storing the Distance in mm)
* Output:			Initialize the required registers
* Logic :			For moving robot by specified distance using encoders. When the shaftCount varibles reach the threshold, the motion is stopped
* Example Call:		linear_distance_mm(100)
*/

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;
 ReqdShaftCount =(float) DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
 ShaftCountLeft = ShaftCountRight = 0;
 while(1)
 {
  //lcd_print(2,1,ShaftCountLeft,2);
  //lcd_print(2,1,ShaftCountRight,5);
  //lcd_print(2,7,ReqdShaftCountInt,5);
  if((ShaftCountLeft > ReqdShaftCountInt) | (ShaftCountRight > ReqdShaftCountInt))
  {
  	break;
  }
 } 
 stop(); //Stop robot
}

/*
* Function Name:	forward_mm
* Input:			DistanceInMM(Integer storing the Distance in mm)
* Output:			Initialize the required registers
* Logic:			Function for moving the robot forward by specified distance. Robot starts moving forward and the linear_distance_mm function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the distance to be covered
* Example Call:		forward_mm(100)
*/

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

/*
* Function Name:	back_mm
* Input:			DistanceInMM(Integer storing the Distance in mm)
* Output:			Initialize the required registers
* Logic:			Function for moving the robot backward by specified distance. Robot starts moving backward and the linear_distance_mm function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the distance to be covered
* Example Call:		back_mm(100)
*/

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

/*
* Function Name:	left_degrees
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic:			Function for rotating the robot towards left by specified angle. Robot starts rotating left and the angle_rotate function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the angle to be rotated
* Example Call:		left_degrees(100)
*/

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}

/*
* Function Name:	right_degrees
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic:			Function for rotating the robot towards right by specified angle. Robot starts rotating right and the angle_rotate function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the angle to be rotated
* Example Call:		right_degrees(100)
*/

void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}

/*
* Function Name:	soft_left_degrees
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic:			Function for rotating the robot towards left by specified angle. Robot starts rotating left and the angle_rotate function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the angle to be rotated. Only right wheel moves here
* Example Call:		soft_left_degrees(100)
*/

void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

/*
* Function Name:	soft_right_degrees
* Input:			Degrees(Integer storing the Angle in Degrees)
* Output:			Initialize the required registers
* Logic:			Function for rotating the robot towards right by specified angle. Robot starts rotating right and the angle_rotate function is called. Motion is terminated once the terminating condition is reached in the function when the encoder wheel makes the requried number of cuts for the angle to be rotated. Only left wheel moves here
* Example Call:		soft_right_degrees(100)
*/

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

/*
* Function Name:	adc_pin_config
* Input:			None
* Output:			Initialize the required registers
* Logic:			Initialization : Function to configure ports for adc conversion. Ports where sensors are attached are configured for adc conversion.
* Example Call:		adc_pin_config()
*/

void adc_pin_config(void)
{
	DDRF=0x00;
	PORTF=0x00;
	DDRK=0x00;
	PORTK=0x00;
}

/*
* Function Name:	adc_init
* Input:			None
* Output:			Initialize the required registers
* Logic:			Initialization : Initializing Registers for ADC Conversion
* Example Call:		adc_pin_config()
*/

void adc_init(void)
{
	ADCSRA=0x00;
	ADCSRB=0x00;
	ADMUX=0x00;
	ADCSRA=0x86;	//ADEN=1 ADIE=1....
	ACSR=0x80;
}

/*
* Function Name:	velocity
* Input:			left,right(Integers storing left and right motor speeds (values 0-255))
* Output:			Velocity for motors are set
* Logic:			Changing the pulse width using registers
* Example Call:		velocity(255,255)
*/

void velocity(unsigned char left,unsigned char right)	//Set PWM Velocity
{
	//lcd_print(2,1,left,3);
	//lcd_print(2,5,right,3);
	OCR5AL = (unsigned char) left;
	OCR5BL = (unsigned char) right;
}

/*
* Function Name:	port_init
* Input:			None
* Output:			Initialize the required registers
* Logic:			Initialization : Initializing all the ports
* Example Call:		port_init()
*/

void port_init()
{
 motion_pin_config(); //robot motion pins config
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config	
 buzzer_pin_config();//Buzzer Pin
}

/*
* Function Name:	init_devices
* Input:			None
* Output:			Intialize the required registers
* Logic:			Initialization : Initializing all the devices connected to the ports and other pins
* Example Call:		init_devices()
*/

void init_devices()
{
 cli(); //Clears the global interrupt
 port_init();  //Initializes all the ports
 lcd_port_config();
 adc_pin_config();
 init_timer5();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 adc_init();
 sei();   // Enables the global interrupt 
}

/*
* Function Name:	ADC_Conversion
* Input:			ch(character storing the channel for ADC Conversion)
* Output:			Digital Sensor Readings
* Logic:			Converting the analog Readings to Digital values
* Example Call:		ADC_Conversion(3)
*/

unsigned char ADC_Conversion(unsigned char ch)
{
	unsigned char a;
	if(ch>7)
	{
		ADCSRB=0x08;
	}
	ch=ch & 0x07;
	ADMUX=0x20 | ch;
	ADCSRA =ADCSRA | 0x40;		//set start conv bit
	while((ADCSRA & 0x10)==0);	//wait for adc conv to complete
	a=ADCH;						//Result stored here after convo
	ADCSRA=ADCSRA | 0x10;		//Clear ADIF by setting it to 1
	ADCSRB=0x00;
	return a;
}

/*
* Function Name:	print_sensor(char row,char col,unsigned char channel)
* Input:			row,col,channel(characers denoting row,col,ADC Channel to print on LCD)
* Output:			prints the output on LCD
* Logic:			Converting the analog Readings to Digital values and printing them on LCD
* Example Call:		print_sensor(1,1,3);
*/

void print_sensor(char row,char col,unsigned char channel)
{
	ADC_Value=ADC_Conversion(channel);
	lcd_print(row,col,ADC_Value,3);
}
