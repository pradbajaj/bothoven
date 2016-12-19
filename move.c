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
		if (flag == 1) {
			velocity(left_motor, right_motor);
		}
		else {
			velocity(right_motor, left_motor);
		}
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