/*
	*Team ID: eYRC-BV#1651
	*Author List: Aayush, Pradyumna, Pranjal, Shashwat
	*filename: adjacency.c
	*Theme: Bothoven
	*Functions: initMap()
	*Global Variable: map, map_link, map_angle
*/

#ifndef __ADJACENCY__
#define __ADJACENCY__

#include <math.h>
#include <stdlib.h>

// int size = 49;
#define INF 600000

//Stores absolute angles, links nodes to notes and maps graphs respectively
int map_angle[size][size];
int map_link[34][2];
int map [size][size];

/*
	*Function Name: initMap
	*Input: NIL
	*Output: integer-> 0 if everything was successful
	*Logic: Initializes the map with the know map.
	*		Required only at the begining.
	*Example Call: initMap();
*/
int initMap () {
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

	for (int i = 1; i < 25; i++)
		for (int j = 0; j < 2; j++)
			map_link[i][j] = (j == 0 ? i : -1);
	//Manually linking nodes to notes
	//Replace hard code with a better method if and when available
	map_link[25][0] = 1;
	map_link[27][0] = 5;
	map_link[28][0] = 9;
	map_link[30][0] = 13;
	map_link[31][0] = 17;
	map_link[33][0] = 21;
	map_link[26][0] = 37;
	map_link[26][1] = 38;
	map_link[29][0] = 39;
	map_link[29][1] = 40;
	map_link[32][0] = 41;
	map_link[32][1] = 42;
	
	//Storing absolute angle of every point.
	int sangle = -30;
	for (int i = 2; i < 23; i=i+2)
	{
		map_angle[i][i+1] = sangle;
		map_angle[i+1][i+2] = sangle;
		if (sangle < 0)
		{
			map_angle[i+1][i] = sangle + 180;
			map_angle[i+2][i+1] = sangle + 180;
		}
		if (sangle > 0)
		{
			map_angle[i+1][i] = sangle - 180;
			map_angle[i+2][i+1] = sangle - 180;
		}
		sangle -= 30;
		if (sangle < -180)
		{
			sangle = 150;
		}
	}
	map_angle[1][2] = 0;
	map_angle[2][1] = 180;
	map_angle[1][24] = 180;
	map_angle[24][1] = 0;

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
	map_angle[25][36] = -150;
	map_angle[36][25] = 30;
	map_angle[26][27] = -90;
	map_angle[27][26] = 90;
	map_angle[27][28] = 90;
	map_angle[28][27] = -90;
	map_angle[28][29] = -90;
	map_angle[29][28] = 90;
	map_angle[30][31] = 150;
	map_angle[31][30] = -30;
    map_angle[31][32] = -30;
	map_angle[32][31] = 150;
	map_angle[32][33] = 150;
	map_angle[33][32] = -30;
	map_angle[33][34] = 90;
	map_angle[34][33] = -90;
	map_angle[34][35] = 30;
	map_angle[35][34] = -150;
	map_angle[35][36] = 30;
	map_angle[36][35] = -150;
	
	return 0;
	
}

#endif		//__ADJACENCY__
