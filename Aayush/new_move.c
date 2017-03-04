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