/*********************************POSSIBLE ISSUES******************************
	*Reliability of BFS in case of weighted graph
	*Initialize min
	*Check the value of pathSize at the end of pathFind
******************************************************************************/

#include "QueueDynamic.h"
#include <cstdlib>
#define size 49
#define INF 100000

extern int map[size][size];
extern int map_link[34][6];
extern int map_angle[48][48];

/******************************************************************************
	*Follows the path and moves accordingly and Returns 3 integers with first
	*corresponding to if the movement was successful 2nd corresponds to 
	*the node the bot departed from and 3rd to the intermediate destination
******************************************************************************/
int* move (int path[], int count) {

}

//Returns distance of each point from the searching point. 
int* BFS (int search) {
	struct Queue *Q = NewQueue();	//Initializing the queue required to 
									//maintan nodes to be visited
	int *bfs, source = search;
	bfs = (int *) malloc (size*sizeof (int));
	bool visited[size];				//Maintains if the node is visited
	for (int i = 1; i < size; i++) {
		bfs[i] = 0;
		visited[i] = false;
	}
	EnQueue (Q, search);
	while (!IsEmpty (Q)) {
		//Accessing the first node in the queue and marking it's neighbours to
		//be searched
		search = DeQueue (Q);
		for (int i = 1; i < size; i++) {
			if (!visited[i]) {
				if (map[search][i] != INF) {
					//Adding weight to bfs if the node is connected
					bfs[i] = map[search][i] + bfs[search];
					//In that case, neighbours of this node needs to be scanned
					//too
					EnQueue (Q, i);
					visited [i] = true;
				}
			}
		}
	}
	return bfs; 
}

//Returns the heuristic for destination
int* heuristic (int destination) {
	int *heuris = BFS (destination);	//Calculates heuristics of the node
	return heuris;
}

inline int fCostCalc (int gCost, int heuristic) {
	return gCost + heuristic;
}

//Returns the index with lowest value in an array
int extractMin (int list[], int cost[], int Size) {
	//Think of initializing min
	int min = -1;	
	for (int i = 0; i < Size; i++) {
		if (list[i] != 0) {
			if (min == -1) {
				min = i;
				continue;
			}
			min = ((cost[min] < cost[i]) ? min : i);
		}
	}
	return min;
}

//Reverses the array (needed to reverse so that it is now directed from source
//to destination)
void reverse (int *Rev, int Size) {
	int j = Size-1, i = 0, temp;
	while(i < j) {
		temp = Rev[i];
		Rev[i++] = Rev[j];
		Rev[j--] = temp;
	}
}

int* pathFind (int *parent, int destination, int *pathSize) {
	int *path = (int*) malloc (size*sizeof(int));
	*pathSize = 1;			//There is at least one element (destination)
	int i = destination, j = 1;
	path[0] = destination;
	//Creates a path 
	while (parent[i] != -1) {
		*pathSize++;
		path[j++] = parent[i];
		i = parent[i];
	}
	path[j] = -1;
	//Corrects the order of the path
	reverse (parent, *pathSize);
	return parent;
}

//Executes dStar and moves the bot from source to destination. Returns if move 
//was successful
int dStar (int source, int dest) {
	while (1) {
		int *heuris = heuristic (dest);		//Gets the h cost or heuristic
		//parent maintains the path. Open list maintains the nodes to be explored
		//closed list maintains the node that are already scanned
		int parent[size], open[size], closed[size], current;	
		int gCost[size], fCost[size];	
		int openSize = 0, closedSize = 0;				//Maintains the size of open list.
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
			if (current == dest) break;			//Path to destination is available
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