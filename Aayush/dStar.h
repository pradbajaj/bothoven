/*
	*Team ID: eYRC-BV#1651
	*Author List: Aayush, Pradyumna, Pranjal, Shashwat
	*filename: dStar.c
	*Theme: Bothoven
	*Functions: dStar()
	*External Requirements:
		Funcitons: BFS(int), heuristic(int), fCostCalc(int, int),
					extractMin(int, int, int), reverse (int*, int), 
					pathFind(int*, int, int*)
		Variables: map, map_link, map_angle
*/
#ifndef __DSTAR__
#define __DSTAR__

#include <stdlib.h>
#include "DynamicQueue.h"

#define size 49
#define INF 600000

#include "dStarRequirement.h"
#include "mapRun.h"
#include "adjacency.h"

#include <avr/io.h>

/*
	*Funtion Name: Move (int[], int)
	*Input: An array that contains the path to be followed and
			the number of nodes in that path
	*Output: 3 integers where the first one tells if there was an obstacle
			In case an obstacle is encountered:
				2nd tells the node left
				3rd tells the node it was supposed to go to.
	*Logic: Converts nodes to an array of angle and calls the run function
			Returns the result by changing index to actual nodes
	*Example Call: int *res = Move (path, pathSize);
*/
int* Move (int path[], int pathSize) {
	signed int *angle = (signed int*) malloc(pathSize*sizeof(int));
	for (int j = 0, i = 1; i < pathSize-1; i++, j++) {
		//This could be buggy
		angle[j] = map_angle[path[i]][path[i+1]] - 
					map_angle[path[i-1]][path[i]];
	}
	angle[pathSize-1] = -1;
	int *res = mapRun (angle, pathSize);
	if (res[0] != 0) {
		res[1] = path[res[1]];
		res[2] = path[res[2]];
	}
	free(angle);
	return res;
}

bool callDStar (int *arr, int size) {
	for (int i = 0; i < size; i++) {
		dStar (arr[i], arr[i+1]);
	}
}

int split (int *array, int size) {
	int *arrayS = (int*) malloc (size * sizeof(int));
	int *arrayM = (int*) malloc (size * sizeof(int));
	int countS = 0, countM = 0;
	arrayS[countS++] = 1;
	arrayM[countM++] = 25;
	for (int i = 1; i <= size; i++) {
		if ((array[i]>=1 && array[i]<=7) || (array[i]>=19 && array[i]<=27) 
			||array[i]==33) {
			arrayM[countM++] = array[i];
		} 
		else {
			arrayS[countS++] = array[i];
		}
	}
	arrayM[countM] = arrayS[countS] = -1;
	callDStar (arrayS, countS);
	callDStar (arrayM, countM);
	//Call slave with the array value in arrayS and call dStar in slave
	//Calling dStar here is not possible right now as the slave cannot return obstacle.
	//Removing obstacle from current implementation will require altering the entire dStar
	//code
}

/*
	*Funtion Name: dStar(int, int)
	*Input: Source or the current position of the bot
			Destination or the buzzer point
	*Output: 0 in case of successful completion of the path
	*Logic: Calculates heuristics using BFS
			Applies A* search to find the path with least cost.
			If obstacle is encountered, updates the cost and re-evaluates the
			fastest route.
	*Example Call: dStar (1, 5);
*/
int dStar (int source, int dest) {
	while (1) {
		int *heuris = heuristic (dest);		//Gets the h cost or heuristic
		//parent maintains the path. Open list maintains the nodes to be explored
		//closed list maintains the node that are already scanned
		int parent[size], open[size], closed[size], current;	
		int gCost[size], fCost[size];	
		int openSize = 0, closedSize = 0;				//Maintains the size of
		//open list.
		//This speeds up the scanning process.
		gCost[source] = 0;
		fCost[source] = fCostCalc (gCost[source], heuris[source]);
		for (int i = 0; i < size; i++) {
			//Initialising open and closed list to be empty
			open[i] = closed[i] = 0;
			parent[i] = -1;
			gCost[i] = fCost[i] = INF;
		}
		open[source] = 1, openSize++;
		//Scan as long as the open list is not empty
		while (openSize > 0) {
			current = extractMin (open, fCost, size);
			open[current] = 0, openSize--;
			closed[current] = 1, closedSize++;
			if (current == dest) break;		//Path to destination is available
			for (int i = 0; i < size; i++) {
				if (map[current][i] != INF) {
					if (closed[i] == 0) {	//if point is not on the closed list
						if (open[i] == 0) {
							//If point is not in the open as well as the closed
							//list, add the point to open list. Also calculate
							//gCost and fCost. Update parent to current for
							//retracing the path.
							open[i] = 1;
							openSize++;
							gCost[i] = gCost[current] + map[current][i];
							fCost[i] = fCostCalc(gCost[i], heuris[i]);
							parent[i] = current;
						} else {
							//Otherwise update path if the new one is better than
							//the already discovered path.
							if ((gCost[current] + map[current][i]) < gCost[i]) {
								gCost[i] = gCost[current] + map[current][i];
								fCost[i] = fCostCalc(gCost[i], heuris[i]);
								parent[i] = current;
							}
						}
					}
				}
			}
		}
		if (parent[dest] == -1) {
			//Path cannot exist.
			lcd_cursor(1,1);
			lcd_string("Dest out of");
			lcd_cursor(2, 1);
			lcd_string("reach");
			free (heuris);
			return -1;
		}
		int *pathSize = (int*) malloc (sizeof(int));
		*pathSize = 0;
		int *path = pathFind (parent, dest, pathSize);
		int *result = Move (path, *pathSize);
		free (heuris);
		free (pathSize);
		if (result[0] == 0) {
			free (result);
			return 0;		//Movement complete
		}
		//Updates the map
		map[result[1]][result[2]] = map[result[2]][result[1]] = INF;
		free (result);
	}
}

#endif		//__DSTAR__