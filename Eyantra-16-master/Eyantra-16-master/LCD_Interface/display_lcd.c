#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/delay.h>
#include "lcd.c"
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}
void init_devices(void)
{
	cli();
	lcd_port_config();
	sei();
}
int main(void)
{
	init_devices();
	lcd_init();
	while(1){
	lcd_cursor(1,7);
	lcd_string("Dany");
	lcd_cursor(2,5);
	lcd_string("E-Yantra");}
}
