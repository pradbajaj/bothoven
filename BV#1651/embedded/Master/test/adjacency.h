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

// const int size = 25;
#define INF 6000

//Stores absolute angles, links nodes to notes and maps graphs respectively
int map_angle[49][5];
int map_link[34][2];
int map [49][5];

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
    for (int i = 0; i<49; i++) {
        for (int j = 0; j<5; j++) {
            map[i][j] = -1;
        }
    }
    //Connects nodes 1 through 28 in sequential order
    for(int i = 1; i<49; i++) {
        map[i][0] = i-1;
        map[i][1] = i+1;
    }
    //Manually connecting nodes that are connected in the map
	//Replace it with soft code if you ever find a method
    map[1][0] = 24;
    map[3][2] = 25;
    map[3][3] = 26;
    map[7][2] = 45;
    map[7][3] = 46;
    map[11][2] = 29;
    map[11][3] = 30;
    map[15][2] = 47;
    map[15][3] = 48;
    map[19][2] = 33;
    map[19][3] = 34;
    map[23][2] = 43;
    map[23][3] = 44;
    map[24][1] = 1;
    map[25][0] = 3;
    map[25][2] = 36;
    map[26][2] = 3;
    map[27][2] = 38;
    map[27][3] = 45;
    map[28][2] = 39;
    map[28][3] = 46;
    map[29][2] = 11;
    map[30][2] = 11;
    map[31][2] = 40;
    map[31][3] = 41;
    map[32][2] = 41;
    map[32][3] = 48;
    map[33][2] = 19;
    map[34][2] = 19;
    map[35][2] = 42;
    map[35][3] = 43;
    map[36][2] = 25;
    map[36][3] = 44;
    map[38][1] = 27;
    map[39][0] = 28;
    map[40][1] = 31;
    map[40][0] = 32;
    map[42][1] = 35;
    map[43][0] = 23;
    map[43][1] = 35;
    map[44][0] = 23;
    map[44][1] = 36;
    map[45][0] = 7;
    map[45][1] = 27;
    map[46][0] = 7;
    map[46][1] = 28;
    map[47][0] = 15;
    map[47][1] = 31;
    map[48][0] = 15;
    map[48][1] = 32;

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
	for (int i = 0; i < 49; ++i)
	{
		for (int j = 0; j < 5; ++j)
		{
			map_angle[i][j] = INF;
		}
	}
	signed int sangle = -30;
	for (int i = 2; i < 23; i=i+2)
	{
		map_angle[i][1] = sangle;
		map_angle[i+1][1] = sangle;
		if (sangle < 0)
		{
			map_angle[i+1][0] = sangle + 180;
			map_angle[i+2][0] = sangle + 180;
		}
		if (sangle > 0)
		{
			map_angle[i+1][0] = sangle - 180;
			map_angle[i+2][0] = sangle - 180;
		}
		sangle -= 30;
		if (sangle < -180)
		{
			sangle = 150;
		}
	}

	map_angle[1][0] = 180;
	map_angle[1][1] = 0;
	map_angle[2][0] = 180;
	map_angle[3][2]  = -150;
	map_angle[3][3]  = -90;
	map_angle[7][2]  = 150;
	map_angle[7][3]  = -150;
	map_angle[11][2]  = 90;
	map_angle[11][3]  = 150;
	map_angle[15][2]  = 30;
	map_angle[15][3]  = 90;
	map_angle[19][2]  = -30;
	map_angle[19][3]  = 30;
	map_angle[23][2]  = -90;
	map_angle[23][3]  = -30;
	map_angle[24][1] = 0;	
    map_angle[25][0]  = 30;
	map_angle[25][1] = -30;
	map_angle[25][2] = -150;
	map_angle[26][0] = 150;
	map_angle[26][1] = -90;
	map_angle[26][2]  = 90;
	map_angle[27][0] = 90;
	map_angle[27][1] = -90;
	map_angle[27][2] = -150;
	map_angle[27][3] = -30;
	map_angle[28][0] = 90;
	map_angle[28][1] = -90;
	map_angle[28][2] = 150;
	map_angle[28][3] = 30;
	map_angle[29][0] = 90;
	map_angle[29][1] = -150;
	map_angle[29][2] = -90;
	map_angle[30][0] = 30;
	map_angle[30][1] = 150;
	map_angle[30][2] = -30;
	map_angle[31][0] = -30;
	map_angle[31][1] = 150;
	map_angle[31][2] = 90;
	map_angle[31][3] = -150;
	map_angle[32][0] = -30;
	map_angle[32][1] = 150;
	map_angle[32][2] = 30;
	map_angle[32][3] = -90;
	map_angle[33][0] = -30;
	map_angle[33][1] = 90;
	map_angle[33][2] = 150;
	map_angle[34][0] = -90;
	map_angle[34][1] = 30;
	map_angle[34][2] = -150;
	map_angle[35][0] = -150;
	map_angle[35][1] = 30;
	map_angle[35][2] = -30;
	map_angle[35][3] = 90;
	map_angle[36][0] = -150;
	map_angle[36][1] = -90;
	map_angle[36][2] = 30;
	map_angle[36][3] = 150;
	map_angle[37][0] = 90;
	map_angle[37][1] = -30;
	map_angle[38][0] = 150;
	map_angle[38][1] = 30;
	map_angle[39][0] = -30;
	map_angle[39][1] = -150;
	map_angle[40][0] = 30;
	map_angle[40][1] = -90;
	map_angle[41][0] = -150;
	map_angle[41][1] = 90;
	map_angle[42][0] = -90;
	map_angle[42][1] = 150;
	map_angle[43][0] = 90;
	map_angle[43][1] = -90;
	map_angle[44][0] = 150;
	map_angle[44][1] = -30;
	map_angle[45][0] = -30;
	map_angle[45][1] = 150;
	map_angle[46][0] = 30;
	map_angle[46][1] = -150;
	map_angle[47][0] = -150;
	map_angle[47][1] = 30;
	map_angle[48][0] = -90;
	map_angle[48][1] = 90;
	
	return 0;
	
}

#endif		//__ADJACENCY__
