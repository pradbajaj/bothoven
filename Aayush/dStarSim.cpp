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

const int size = 49;
#define INF 600000

#include "dStarRequirement.h"
//#include "mapRun.h"
#include "adjacency.h"
#include <iostream>

using namespace std;
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
/*
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
}*/

int* move (int path[], int count) {
	int *res = new int[3];
	res[0] = res[1] = res[2] = 0;
	cout << path[0] << "\t";
	for (int i = 0; i < count-1; i++) {
		if ((path[i] == 6 && path[i+1] == 7) || (path[i] == 7 && 
			path[i+1] == 6) || (path[i] == 16 && path[i+1] == 17) ||
			(path[i] == 17 && path[i+1] == 16) || (path[i] == 21 && 
			path[i+1] == 22) || (path[i] == 22 && path[i+1] == 21) || 
			(path[i] == 23 && path[i+1] == 24) || (path[i] == 24 && 
			path[i+1] == 23) || (path[i] == 31 && path[i+1] == 32) || (path[i] == 32 && 
			path[i+1] == 31) || (path[i] == 35 && path[i+1] == 36) || (path[i] == 36 && 
			path[i+1] == 35) || (path[i] == 36 && path[i+1] == 44) || (path[i] == 44 && 
			path[i+1] == 36) || (path[i] == 28 && path[i+1] == 29) || (path[i] == 29 && 
			path[i+1] == 28) || (path[i] == 27 && path[i+1] == 28) || (path[i] == 28 && 
			path[i+1] == 27) || (path[i] == 27 && path[i+1] == 45) || (path[i] == 45 && 
			path[i+1] == 27)) {
				cout << "Obstacle between " << path[i] << " and " << path[i+1];
				res[0] = 1;
				res[1] = path[i];
				res[2] = path[i+1];
				return res;
			}
			cout << path [i+1] << "\t";
	}
	return res;
}

int dStar (int, int); 

void callDStar (int *arr, int size) {
	for (int i = 0; i < size; i++) {
		int res = dStar (arr[i], arr[i+1]);
		if (res != 0)
			dStar(res, arr[++i+1]);
		cout << endl;
	}
}

int split (int *array, int size) {
	int *arrayS = (int*) malloc ((size+2) * sizeof(int));
	int *arrayM = (int*) malloc ((size+2) * sizeof(int));
	int countS = 0, countM = 0;
	arrayS[countS++] = 24;
	arrayM[countM++] = 2;
	for (int i = 0; i < size; i++) {
		if ((arr[i] >= 8 && arr[i] <= 18) || (arr[i] >= 28 && arr[i] <= 32)) {
			arrayS[countS++] = array[i];
		} else {
			arrayM[countM++] = array[i];
		}
	}
	arrayM[countM] = arrayS[countS] = -1;
	cout << "Slave Bot: ";
	callDStar (arrayS, countS-1);
	cout << "\nMaster Bot: ";
	callDStar (arrayM, countM-1);
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
			cout << "Destination out of reach\t";
			free (heuris);
			return current;
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

int main () {
	initMap();
	int Size = 2;
	int *path = (int *) malloc (Size * sizeof (int));
	path[0] = 22;
	path[1] = 18;
	/*for (int i = 0; i < Size; i++) {
		path[i] = rand()%33 + 1;
	}*/
	split (path, Size);
	return 0;
}

#endif		//__DSTAR__