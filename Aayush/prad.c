#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.h"
#include "motion.c"

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

void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}
//Function used for setting motor's direction

void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}

//Function to initialize Buzzer 
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
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
	servo3_pin_config();
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

void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	timer1_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}

void move()
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
{	servo_3(105);
	init_devices();
	lcd_set_4bit();
	lcd_init();
	//int n = 8; // nummber of angles
	signed int angle[] = {0, -60, 0, 60, 0, -120, 0, -120, 120, 0, 0}; // after dectecting the note bot will turn angle[i] angle
	signed int buzz[] = {0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0}; // after dectecting the note bot will turn angle[i] angle
	signed int count = -1;
	signed int buzzcount = -1;

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
			buzzcount++;
			if (buzz[buzzcount] == 1)
			{
				velocity(0, 0);
				while(sequence[seqcount]==0)
				{
					_delay_ms(1000);
				}
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
 				velocity(255, 255);
 				seqcount++;
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
				_delay_ms(1000);
				back();
				_delay_ms(500);
				left_degrees(80);
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
				_delay_ms(1000);
				back();
				_delay_ms(500);
				right_degrees(80);
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
			buzzcount++;
			if (buzz[buzzcount] == 1)
			{
				velocity(0, 0);
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
 				velocity(255, 255);
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
				_delay_ms(1000);
				back();
				_delay_ms(500);
				left_degrees(80);
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
			{	_delay_ms(1000);
				back();
				_delay_ms(500); 
				right_degrees(80);
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
