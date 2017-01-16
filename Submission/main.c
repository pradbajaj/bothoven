/*
	*Team ID: eYRC-BV#1651
	*Author List: Aayush, Pradyumna, Pranjal, Shashwat
	*filename: main.c
	*Theme: Bothoven
	*Functions: main()
	*Global Variable: NIL
*/

#define size 49
#define INF 600000

extern int dStar (int, int);
extern int initMap ();

extern int map[size][size];
extern int map_angle[size][size];
extern int map_link[size][size];

int main () {
	initMap ();
	int Nodes[10];
	Nodes[0] = 1;
	Nodes[1] = 7;
	Nodes[2] = 28;
	Nodes[3] = 27;
	Nodes[4] = 18;
	Nodes[5] = 24;
	Nodes[6] = 13;
	Nodes[7] = 30;
	Nodes[8] = 16;
	Nodes[9] = 20;
	for (int i = 0; i < 9; i++) {
		dStar (Nodes[i], Nodes[i+1]);
	}
	// after the bot completed all the angles
	// it will stop and beep for 5 seconds
	for (int i = 0; i < 20; ++i) {
		stop();
		velocity(0,0);
		buzzer_on();
		_delay_ms(100);
		buzzer_off();
		_delay_ms(150);
	}

	// after all the task is completed the lcd will print "TASK COMPLETED!!!"
	while(1) {
		stop();
		velocity(0,0);
		lcd_cursor(1,1);
		lcd_string("      Task      ");
		lcd_cursor(2,1);
		lcd_string("  Completed!!!  ");
	}
	return 0;
}