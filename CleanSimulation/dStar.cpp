#include <cstdlib>
#include "QueueDynamic.h"

#define size 49

//Returns distance of each point from the searching point. 
extern int* BFS (int);
//Returns the heuristic for destination
extern int* heuristic (int);
//Calculates the fCost
extern inline int fCostCalc (int, int);
//Returns the index with lowest value in an array
extern int extractMin (int, int, int);
//Reverses the array (needed to reverse so that it is now directed from source
//to destination)
extern void reverse (int*, int); 
extern int* pathFind (int*, int, int*);
extern int map[][];
extern int map_link[][];
extern int map_angle[][];
//Executes dStar and moves the bot from source to destination. Returns if move 
//was successful
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
		int *pathSize = (int*) malloc (sizeof(int));
		*pathSize = 0;
		int *path = pathFind (parent, dest, pathSize);
		int *result = move (path, *pathSize);
		if (result[0] == 0) return 0;		//Movement complete
		//Updates the map
		map[result[1]][result[2]] = map[result[2]][result[1]] = INF;
		free (heuris);
		free (pathSize);
	}
}