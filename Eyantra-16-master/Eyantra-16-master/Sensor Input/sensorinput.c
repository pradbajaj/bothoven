#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

unsigned char ADC_Conversion(unsigned char);

unsigned char ADC_Value;

unsigned int value;

float BATT_Voltage,BATT_V;

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
	lcd_port_config();
	adc_pin_config();
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
	init_devices();
	lcd_set_4bit();
	lcd_init();
	while(1){
		print_sensor(1,1,1);
		print_sensor(1,5,2);
		print_sensor(1,9,3);
	}
}
