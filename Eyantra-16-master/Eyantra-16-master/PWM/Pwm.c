#define F_CPU 14745600
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
void motion_pin_config(void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;
	PORTL= PORTL | 0x18;
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
	unsigned char PORTARestore= 0;
	dir &= 0x0F;			//remove upper nibbel
	PORTARestore = PORTA; // Read org value
	PORTARestore &= 0xF0; //Lower nibbel to 0
	PORTARestore |= dir;  // Lower nibbel to new val
	PORTA = PORTARestore;
}
void forward(void)
{
	motion_set(0x06);
}
int main(void)
{
	cli();
	motion_pin_config();
	init_timer5();
	sei();
	forward();
	while(1)
	{
		velocity(100,100);
		forward();
		_delay_ms(2000);

		velocity(0,255);
		forward();
		_delay_ms(2000);

		velocity(255,0);
		forward();
		_delay_ms(2000);
	}
}
