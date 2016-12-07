//Check abs function in embeded c
#define size 49
#define INF 10000

int initMap () {
	int map [size][size];
	//Initializes each nodes as unconnected
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
			map [i][j] = INF;
	//Connects nodes 1 through 24 in sequential order
	for (int i = 1; i < 25; i++)
		for (int j = 1; j < 25; j++)
			if (abs(i-j) == 1)
				map[i][j] = 1;
	//Connects nodes 25 through 38 in sequential order
	for (int i = 25; i < 39; i++)
		for (int j = 25; j < 39; j++)
			if (abs(i-j) == 1)
				map[i][j] = 1;
	//Manually connecting nodes that are connected in the map
	//Replace it with soft code if you ever find a method
	map[35][42] = map[42][35] = 1;
	map[42][41] = map[41][42] = 1;
	map[41][32] = map[32][41] = 1;
	map[32][48] = map[48][32] = 1;
	map[47][31] = map[31][47] = 1;
	map[31][40] = map[40][31] = 1;
	map[40][39] = map[39][40] = 1;
	map[39][28] = map[28][39] = 1;
	map[28][46] = map[46][28] = 1;
	map[45][27] = map[27][45] = 1;
	map[27][38] = map[38][27] = 1;
	map[36][25] = map[25][36] = 1;
	map[36][44] = map[44][36] = 1;
	map[43][35] = map[35][43] = 1;
	map[24][1]  = map[1][24]  = 1;
	map[23][44] = map[44][23] = 1;
	map[23][43] = map[43][23] = 1;
	map[19][34] = map[34][19] = 1;
	map[19][33] = map[33][19] = 1;
	map[15][48] = map[48][15] = 1;
	map[15][47] = map[47][15] = 1;
	map[11][30] = map[30][11] = 1;
	map[11][29] = map[29][11] = 1;
	map[7][46]  = map[46][7]  = 1;
	map[7][45]  = map[45][7]  = 1;
	map[3][26]  = map[26][3]  = 1;
	map[3][25]  = map[25][3]  = 1;
	//Mapping notes to nodes
	int map_link[34][6];
	for (int i = 0; i < 25; i++)
		for (int j = 0; j < 6; j++)
			map_link[i][j] = -1;
	for (int i = 1; i < 25; i++)
		for (int j = 0; j < 6; j++)
			map_link[i][j] = (j == 0 ? i : -1);
	//Manually linking nodes to notes
	//Replace hard code with a better method if and when available
	map_link[25][0] = 1;
	map_link[25][1] = 25;
	map_link[25][2] = 36;
	map_link[25][3] = 44;
	map_link[27][0] = 5;
	map_link[27][1] = 26;
	map_link[27][2] = 27;
	map_link[27][3] = 45;
	map_link[28][0] = 9;
	map_link[28][1] = 28;
	map_link[28][2] = 29;
	map_link[28][3] = 46;
	map_link[30][0] = 13;
	map_link[30][1] = 30;
	map_link[30][2] = 31;
	map_link[30][3] = 47;
	map_link[31][0] = 17;
	map_link[31][1] = 32;
	map_link[31][2] = 33;
	map_link[31][3] = 48;
	map_link[33][0] = 21;
	map_link[33][1] = 34;
	map_link[33][2] = 35;
	map_link[33][3] = 43;
	map_link[26][0] = 25;
	map_link[26][1] = 26;
	map_link[26][2] = 27;
	map_link[26][3] = 36;
	map_link[26][4] = 37;
	map_link[26][5] = 38;
	map_link[29][0] = 28;
	map_link[29][1] = 29;
	map_link[29][2] = 30;
	map_link[29][3] = 31;
	map_link[29][4] = 39;
	map_link[29][5] = 40;
	map_link[32][0] = 32;
	map_link[32][1] = 33;
	map_link[32][2] = 34;
	map_link[32][3] = 35;
	map_link[32][4] = 41;
	map_link[32][5] = 42;
//Mapping angles
int map_angle[48][48];

	map_angle[3][25]  = -150;
    map_angle[25][3]  = 30;
	map_angle[3][26]  = -90;
	map_angle[26][3]  = 90;
	map_angle[7][45]  = 150;
	map_angle[45][7]  = -30;
	map_angle[7][46]  = -150;
	map_angle[46][7]  = 30;
	map_angle[11][29] = 90;
	map_angle[29][11] = -90;
	map_angle[11][30] = 150;
	map_angle[30][11] = -30;
	map_angle[15][47] = 30;
	map_angle[47][15] = -150;
	map_angle[15][48] = 90;
	map_angle[48][15] = -90;
	map_angle[19][33] = -30;
	map_angle[33][19] = 150;
	map_angle[19][34] = 30;
	map_angle[34][19] = -150;
	map_angle[23][43] = -90;
	map_angle[43][23] = 90;
	map_angle[23][44] = -30;
	map_angle[44][23] = 150;
	map_angle[25][26] = -30;
	map_angle[26][25] = 150;
	map_angle[29][30] = -150;
	map_angle[30][29] = 30;
	map_angle[35][43] = 90;
	map_angle[43][35] = -90;
	map_angle[36][44] = 150;
	map_angle[44][36] = -30;
	map_angle[27][45] = -30;
	map_angle[45][27] = 150;
	map_angle[28][46] = 30;
	map_angle[46][28] = -150;
	map_angle[31][47] = -150;
	map_angle[47][31] = 30;
	map_angle[32][48] = -90;
	map_angle[48][32] = 90;
	map_angle[27][38] = -150;
	map_angle[38][27] = 30;
	map_angle[28][39] = 150;
	map_angle[39][28] = -30;
	map_angle[31][40] = 90;
	map_angle[40][31] = -90;
	map_angle[32][41] = 30;
	map_angle[41][32] = -150;
	map_angle[35][42] = -30;
	map_angle[42][35] = 150;
	map_angle[36][37] = -90;
	map_angle[37][36] = 90;
	map_angle[39][40] = -150;
	map_angle[40][39] = 30;
	map_angle[37][38] = 150;
	map_angle[38][37] = -30;
	map_angle[41][42] = 90;
	map_angle[42][41] = -90;
		
	return 0;
}
