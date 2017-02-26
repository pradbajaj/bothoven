// Amrith M ... 4/12/16	...White Line Follower FB5
#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

#define THRESHOLD 20		//Designing White Line Follower

#define MAX_VEL 200

#define MIN_VEL 0

unsigned char ADC_Conversion(unsigned char);

unsigned char ADC_Value;

unsigned int value;

unsigned char Left_white_line = 0;

unsigned char Center_white_line = 0;

unsigned char Right_white_line = 0;

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 

volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder

volatile unsigned int Degrees; //to accept angle in degrees for turning

void motion_pin_config(void)
{
	DDRA= DDRA | 0x0F; //0000 1111 --->	PA3 PA2 PA1 PA0 (HIGH)
	DDRL= DDRL | 0x18; //0001 1000 ---> PL3 PL4 (HIGH)
	PORTA= PORTA & 0xF0;
	PORTL= PORTL | 0x18;
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
void velocity(unsigned char left,unsigned char right)	//Set PWM Velocity
{
	OCR5AL = (unsigned char) left;
	OCR5BL = (unsigned char) right;
}

void motion_set(unsigned char dir)
{
	unsigned char PortA_Restore=0;
	dir &= 0x0F;			//Remove Upper nibbel
	PortA_Restore= PORTA;	//Setting PortA_Restore to current PORTA status			
	PortA_Restore &= 0xF0;	//Setting lower nibbel to 0 
	PortA_Restore |=dir;	//Setting PortA_Restore to current dir
	PORTA=PortA_Restore;	//Setting the current status of PORTA
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
void stop (void)
{
  motion_set(0x00);
}
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
  lcd_print(1,1,ShaftCountLeft,2);
  lcd_print(2,1,ShaftCountRight,2);
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

 ReqdShaftCount =(float) DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
 ShaftCountLeft = 0;
 while(1)
 {
  lcd_print(1,1,ShaftCountLeft,2);
  lcd_print(2,1,ShaftCountRight,2);
  if(ShaftCountLeft > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop robot
}
//MOTION			  // RB  RF  LF  LB						
void forward(void)    // PA3 PA2 PA1 PA0
{
	motion_set(0x06);   // 0 1 1 0
}

void backward(void)
{
	motion_set(0x09);	// 1 0 0 1
}

void diff_left(void) //Left wheel backward, right wheel stationary
{
 	motion_set(0x05);
}

void diff_right(void) //Left wheel stationary, Right wheel backward
{
 	motion_set(0x0A);
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

void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

void adc_pin_config(void)
{
	DDRF=0x00;
	PORTF=0x00;
	DDRK=0x00;
	PORTK=0x00;
}

void adc_init(void)
{
	ADCSRA=0x00;
	ADCSRB=0x00;
	ADMUX=0x00;
	ADCSRA=0x86;	//ADEN=1 ADIE=1....
	ACSR=0x80;
}

void init_devices(void)
{
	cli();
	motion_pin_config();
	init_timer5();
	lcd_port_config();
	adc_pin_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
	adc_init();
	sei();
}

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

void print_sensor(char row,char col,unsigned char channel)
{
	ADC_Value=ADC_Conversion(channel);
	lcd_print(row,col,ADC_Value,3);
}

int main(void)
{
	unsigned int i=0,djFlag=0,prev=0;
	unsigned char flag =0;
	init_devices();
	lcd_set_4bit();
	lcd_init();
	velocity(MAX_VEL,MAX_VEL);    // Set the speed to max velocity
 	lcd_print(2,1,MAX_VEL,3);
 	lcd_print(2,5,MAX_VEL,3);
	forward(); 
	while(1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1); //Getting data of Right WL Sensor
		print_sensor(1,1,3);
		print_sensor(1,5,2);
		print_sensor(1,9,1);
		flag=0; djFlag = 0;
		if (Left_white_line>THRESHOLD) djFlag = 100;
		if (Center_white_line>THRESHOLD) djFlag += 10;
		if (Right_white_line>THRESHOLD) djFlag +=1;
		lcd_print (2,9,djFlag,3);
		if (djFlag == 11 || djFlag == 110) {
			velocity(0,0);
			left_degrees(30);
			right_degrees(30);

			_delay_ms(5000);
		}
		else if (djFlag == 10) {
			//Go straight
			flag=1;

			velocity(MAX_VEL,MAX_VEL); 
			forward();     // Run robot at max velocity 
			lcd_print (2,1,MAX_VEL,3);
			lcd_print (2,5,MAX_VEL,3);
		} 
		else if (djFlag == 100) {
			//Go left
			flag=1;                                                       
			velocity(0,200);     // Run robot left wheel at max velocity and right wheel 
			lcd_print (2,1,MIN_VEL,3);           // at min velocity
			lcd_print (2,5,MAX_VEL,3);
		} 
		else if (djFlag == 1) {
			//Go right
			if (djFlag!=0) prev = 1;
			flag=1;
			velocity(200,0);      // Run robot right wheel at max velocity and left wheel 
			lcd_print (2,1,MIN_VEL,3);           // at min velocity
			lcd_print (2,5,MAX_VEL,3);
		} 
		else if (djFlag == 0) {
			//Error
			flag=1;
			velocity(MAX_VEL,MAX_VEL);      // stop the robot
			lcd_print (2,1,0,3);
			lcd_print (2,5,0,3);
		}
		else {
			velocity(0,0);
		}
	}
}
