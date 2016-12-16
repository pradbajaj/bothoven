/*********************************POSSIBLE ISSUES******************************
	*Reliability of BFS in case of weighted graph
******************************************************************************/

#include "QueueDynamic.h"
#define size 49

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
	struct Queue Q = NewQueue();	//Initializing the queue required to 
									//maintan nodes to be visited
	int *bfs = new int [size], source = search;
	bfs = (int *) malloc (size*sizeof (int));
	bool visited[size];				//Maintains if the node is visited
	for (int i = 1; i < size; i++) {
		bfs[i] = 0;
		visited[i] = false;
	}
	Q.EnQueue (Q, search);
	while (!IsEmpty (Q)) {
		//Accessing the first node in the queue and marking it's neighbours to
		//be searched
		search = Q.DeQueue ();
		for (int i = 1; i < size; i++) {
			if (!visited[i]) {
				if (map[search][i] != INF) {
					//Adding weight to bfs if the node is connected
					bfs[i] = map[search][i] + bfs[search];
					//In that case, neighbours of this node needs to be scanned
					//too
					Q.EnQueue (i);
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

//Executes dStar and moves the bot from source to destination. Returns if move 
//was successful
int dStar (int source, int dest) {
	int *heuris = heuristic (dest);		//Gets the h cost or heuristic
	//parent maintains the path. Open list maintains the nodes to be explored
	//closed list maintains the node that are already scanned
	int parent[size], open[size], closed[size];	
	int gCost[size], fCost[size];	
	int openSize = 0;				//Maintains the size of open list.
	//This speeds up the scanning process.
	gCost[source] = 0;
	fCost[source] = fCostCalc (gCost[source], heuristic[source]);
	for (int i = 0; i < size; i++) {
		//Initialising open and closed list to be empty
		open [i] = closed [i] = 0;
		parent [i] = -1;
		gCost[i] = fCost[i] = INF;
	}
	open[source] = 1, openSize++;
	//Scan as long as the open list is not empty
	while (openSize > 0) {

	}
	return 0;
}