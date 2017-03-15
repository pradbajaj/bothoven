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

const int size = 25;
#define INF 600000

//Stores absolute angles, links nodes to notes and maps graphs respectively
int map_angle[25][25];
int map_link[34][2];
int map [49][49];

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
	//Connects nodes 1 through 14 in sequential order
	for (int i = 1; i < 14; i++)
		{map[i][i+1]=1;
		map[i][i-1]=1;}

	//Connects nodes 16 through 24 in sequential order
	for (int i = 17; i < 24; i++)
		{map[i][i+1]=1;
		map[i][i-1]=1;}
	//Manually connecting nodes that are connected in the map
	//Replace it with soft code if you ever find a method
	map[14][13] = 1;
	map[16][17] = 1;
	map[24][23] = 1;
	map[9][14]  = map[14][9]  = 1;
	map[3][14]  = map[14][3]  = 1;
	map[3][13]  = map[13][3]  = 1;
	map[12][15] = map[15][12] = 1;
	map[15][23] = map[23][15] = 1;
	map[16][23] = map[23][16] = 1;
	map[12][17] = map[17][12] = 1;
	

	for (int i = 1; i < 25; i++)
		for (int j = 0; j < 2; j++)
			map_link[i][j] = (j == 0 ? i : -1);
	//Manually linking nodes to notes
	//Replace hard code with a better method if and when available
	map_link[25][0] = 1;
	map_link[27][0] = 5;
	map_link[33][0] = 21;
	map_link[26][0] = 14;
	map_link[26][1] = 11;
	
	
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
	map_angle[23][43] = -90;
	map_angle[43][23] = 90;
	map_angle[23][44] = -30;
	map_angle[44][23] = 150;
	map_angle[25][26] = -30;
	map_angle[26][25] = 150;
	map_angle[35][43] = 90;
	map_angle[43][35] = -90;
	map_angle[36][44] = 150;
	map_angle[44][36] = -30;
	map_angle[27][45] = -30;
	map_angle[45][27] = 150;
	map_angle[27][38] = -150;
	map_angle[38][27] = 30;
	map_angle[35][42] = -30;
	map_angle[42][35] = 150;
	map_angle[36][37] = -90;
	map_angle[37][36] = 90;
	map_angle[37][38] = 150;
	map_angle[38][37] = -30;
	map_angle[25][36] = -150
	map_angle[36][25] = 30
	map_angle[26][27] = -90
	map_angle[34][35] = 30
	map_angle[35][34] = -150
	map_angle[35][36] = 30
	map_angle[36][35] = -150
	map_angle[19][34] = 30
	map_angle[34][19] = -150

	
	return 0;
	
}

#endif		//__ADJACENCY__

