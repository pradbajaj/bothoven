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
#define		Threshold 		40		//Defining Threshold value of black line
									// Sensor Value less than threshold will be considered white line

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
unsigned char ADC_Value;
unsigned char data; //to store received data from UDR1
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
signed int AVG = 0;
signed int senser_value_L,senser_value_C,senser_value_R ;
int left_motor = 0, right_motor = 0;
unsigned char Front_ultrasonic_Sensor=0;
unsigned char Front_IR_Sensor=0;

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
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

//Function To Initialize UART1
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
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

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

/*
	*********************************NOTE**************************************
	*The following functions configures the pins required for various stuff
	*like lcd_port, adc_pin etc
	**********************************END**************************************
*/
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
		right_motor = 255;
		velocity(left_motor,right_motor);
		flag = 1;
	}
	// If black line is at right sensor and center sensor is not decting a black line
	// Speed up left motor a bit to get a right turn
	// Further set the flag to be equal to '2' suggesting that bot has taken right turn this time
	else if ((senser_value_R > Threshold) && (senser_value_C < Threshold))
	{
		forward();
		left_motor = 255;
		right_motor = 200;
		velocity(left_motor,right_motor);
		flag = 2;
	}
	// If black line is at center sensor and left & right sensors are not decting a black line
	// Speed up both motors equally to get a straight movement
	else if ((senser_value_C > Threshold) && (senser_value_L < Threshold) && (senser_value_R < Threshold))
	{
		forward();
		left_motor = 255;
		right_motor = 255;
		velocity(left_motor, right_motor);
	}
	// If black line is nowhere to be found
	// Speed up right or left motor according to the flag set
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

void mapRun(signed int angle[], int Size)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	// after dectecting the note bot will turn angle[i] angle
	signed int count = -1;

	while (count < Size) {

		move();// calling the move function

		// If center center plus left or right sensor detects the black line
		// The bot is over a node
		if ((senser_value_C > Threshold) && (senser_value_L > Threshold))
		{
			lcd_cursor(2,1);
			lcd_string("NOTE DETECTED!!!");
			// move while encoders certain value reach
			
			count++;
			// If angle[i] is -1 we have got a MNP hence the bot will beep
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

			//if angle[i] is 60 the bot has to take a 60 degree turn
			// bot will move 6 cm ahead and 30 degree left to skip the current black line
			// then it will rotate left until it finds another black line
			// lcd will print "60 degree success"
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
			//if angle[i] is 120 the bot has to take a 120 degree turn
			// bot will move 1.5 cm ahead and 30 degree back left to skip the current black line
			// then it will rotate soft left until it finds another black line
			// lcd will print "120 degree success"
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
			//if angle[i] is -60 the bot has to take a -60 degree turn
			// bot will move 6 cm ahead and 30 degree right to skip the current black line
			// then it will rotate rihgt until it finds another black line
			// lcd will print "-60 degree success"
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
			//if angle[i] is -120 the bot has to take a -120 degree turn
			// bot will move 1.5 cm ahead and 30 degree back right to skip the current black line
			// then it will rotate soft right until it finds another black line
			// lcd will print "-120 degree success"
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
			//if angle[i] is 180 the bot has to take a 190 degree turn
			// bot will move left for 500 milliseconds to skip the current black line
			// then it will rotate left until it finds another black line
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

			// by default lcd will be print "MOVING ON FLEX"
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
	}
}

signed int *angle;
signed int count=-1; 
signed int size;

void printAngle(signed int arr[]) {
	int row = 1;
	for(int i = 0; i<size; i++) {
		lcd_print(row,i+1,arr[i],2) ;
		i++;
		if (i+2 == 16)
			row = 2;
	}
}

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable 

	UDR2 = data;

	signed int hex = data;
	if (count == -1) {
		//hex=index;
		size = hex;
		angle=(int *)malloc(size*sizeof(int));
		count++;
	}
	else
	if(count<size)
	{
		angle[count]=data;
		count++;
		
	}
	else
	if(count == size)
	{
		mapRun(angle, size);
		//printAngle(angle);
	}
	else {
		lcd_cursor(2,1);
		lcd_string("all received");
	}
	
}

/*SIGNAL(SIG_USART2_RECV) 		// ISR for receive complete interrupt
{
	data = UDR2; 				//making copy of data from UDR0 in 'data' variable 

	UDR2 = data;

  UDR0 = data; 				//echo data back to PC

  data = UDR2; 				//making copy of data from UDR0 in 'data' variable 

	UDR2 = data;

	signed int hex = data;
	if (count == -1) {
		//hex=index;
		size = hex;
		angle=(int *)malloc(size*sizeof(int));
		count++;
	}
	else
	if(count==size)
	{
		//mapRun(angle, size);
		printAngle(angle);
	}
	else
	{
		angle[count]=data;
		count++;
	}

}*/




//Function To Initialize all The Devices
void init_devices()
{
 cli(); //Clears the global interrupts
 port_init();
adc_init();
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
	init_devices();
	while(1);
}

