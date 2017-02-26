#define F_CPU 14745600

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<avr/delay.h>

volatile unsigned long int ShaftCountLeft=0;	//Left Encoder
volatile unsigned long int ShaftCountRight=0;	//Right Encoder

void motion_pin_config(void)
{
	DDRA= DDRA | 0x0F; //0000 1111 --->	PA3 PA2 PA1 PA0 (HIGH)
	DDRL= DDRL | 0x18; //0001 1000 ---> PL3 PL4 (HIGH)
	PORTA= PORTA & 0xF0;
	PORTL= PORTL | 0x18;
}

void left_enc_pin(void)
{
	DDRE=DDRE|0xEF;		//port E pin 4 as input 
	PORTE= PORTE | 0x10;
}

void right_enc_pin(void)
{
	DDRE=DDRE|0xDF;		//port E pin 5 as input
	PORTE= PORTE | 0x20;
}

void left_enc_int(void)
{
	cli();					//Clear Global Interrupts
	EICRB = EICRB | 0x02;	//INT4 set to trigger with falling edge
	EIMSK = EIMSK | 0x10;	//
	sei();
}

void right_enc_int(void)
{
	cli();
	EICRB = EICRB | 0x08;
	EIMSK = EIMSK | 0x20;
	sei();
}

ISR(INT5_vect)
{
	ShaftCountRight++;
}

ISR(INT4_vect)
{
	ShaftCountLeft++;
}

void init_timer5(void)
{
	TCCR5B = 0x00;	//stop
	TCNT5H = 0xFF;	//counter higher 8 bit value to which OCRxH is compared with
	TCNT5L = 0x01;	//counter higher 8 bit value to which OCRxH is compared with
	OCR5AH = 0x00;	//Output compare register high for left motor
	OCR5AL = 0xFF;	//Output compare register low for left motor
	OCR5BH = 0x00;	//Output compare register high for right motor	
	OCR5BL = 0xFF;	//Output compare register high for right motor
	OCR5CH = 0x00;	//Motor C1
	OCR5CL = 0xFF;	//Motor C1
	TCCR5A = 0xA9;

/* COM5A1=1,COM5A0=0,COM5B1=1,COM5B0=0,COM5C1=1,COM5C0=0
For Overriding normal port functionality to OCRnA ouputs
WGM51=0,WGM50=1 along with WGM52 in TCCRB for selecting fast PWM 8 bit mode
*/

	TCCR5B = 0x0B;	//WGM12=1,CS12=0,CS11=1,CS10=1	(Prescaler=64)

}

void velocity(unsigned char left,unsigned char right)
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

//MOTION			  // RB  RF  LF  LB						
void forward(void)    // PA3 PA2 PA1 PA0
{
	motion_set(0x06);   // 0 1 1 0
}

void backward(void)
{
	motion_set(0x09);	// 1 0 0 1
}

void left(void)
{
	motion_set(0x05);	// 0 1 0 1
}

void right(void)
{
	motion_set(0x0A);	// 1 0 1 0
}

void soft_left(void)	
{
	motion_set(0x04);	// 0 1 0 0
}

void soft_right(void)
{
	motion_set(0x02);	// 0 0 1 0
}

void stop(void)
{
	motion_set(0x00);	// 0 0 0 0
}

/* LOGIC
When the wheel rotates and the encoder circle cuts the IR, interrupt is called..
When the Interrupt is called, the ISR function is invoked and thus shaft value i
incremented*/
void linear_dist_mm(unsigned int dist_mm)
{
	float ReqdShaftCount=0;
	unsigned long int ReqdShaftCountInt=0;
	
	ReqdShaftCount= dist_mm/5.338; 	//Number of pulses reqd
	ReqdShaftCountInt=(unsigned long int)ReqdShaftCount;

	ShaftCountRight=0;		//Initially set to 0 ..Later interrupt would change its value
	while(1)
	{
		if(ShaftCountRight>ReqdShaftCountInt)
		{
			break;
		}
	}
	stop();
}

void forward_mm(unsigned int dist_mm)
{
	forward();
	linear_dist_mm(dist_mm);

}

void port_init(void)
{
	motion_pin_config();
	init_timer5();
	left_enc_pin();
	right_enc_pin();

}

void init_devices(void)
{
	cli();
	port_init();
	left_enc_int();
	right_enc_int();
	sei();
}

int main(void)
{
	init_devices();
	forward_mm(100);
	while(1);

}
