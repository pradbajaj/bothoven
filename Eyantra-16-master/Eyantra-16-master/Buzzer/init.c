#define F_CPU 14745600
#include<avr/interrupt.h>
#include<avr/io.h>
#include<util/delay.h>
void buzzer_on(void)
{
	
	PORTC= 0x08;   // pin 3 to high 0000 1000
}

void buzzer_pin_config(void)
{
	DDRC=DDRC | 0x08; // pin 3 as op
	PORTC=PORTC & 0xF7;

}
void buzzer_off(void)
{
	PORTC= 0x00;
}
void init_device(void)
{
	cli();//clear interrupts
	buzzer_pin_config();
	sei();
}
int main(void)
{
	init_device();
	while(1)
	{	
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		_delay_ms(1000);
	}
}
