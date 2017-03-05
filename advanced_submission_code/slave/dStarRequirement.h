/*
	*Team ID: eYRC-BV#1651
	*Author List: Aayush, Pradyumna, Pranjal, Shashwat
	*filename: dStarRequirement.c
	*Theme: Bothoven
	*Functions: BFS(int), heuristic(int), fCostCalc(int, int), 
				extractMin(int[], int[],int), reverse(int*, int),
				pathFind(int*, int, int*)
	*Global Variable: externals: map, map_link, map_angle
*/
#ifndef __DSTAR_REQUIREMENT__
#define __DSTAR_REQUIREMENT__

#include "DynamicQueue.h"
#include <stdlib.h>

//const int size = 49;
#define INF 600000

#include "adjacency.h"

/*
	*Function name: BFS (int)
	*Input: Source of the search tree
	*Output: An array with cost of traversing all nodes from the source
	*Logic: This is a simple implementation of standard Breadth First Search
			It puts the source to a queue. Then as long as the queue is not
			empty, it dequeues the first element, enqueues all the neighbours
			which were not already explored and updates the cost.
	*Example Call: int *cost = BFS(source);
*/
int* BFS (int search) {
	struct Queue *Q = NewQueue();	//Initializing the queue required to 
									//maintan nodes to be visited
	int *bfs, source = search;
	bfs = (int *) malloc (size*sizeof (int));
	int visited[size];				//Maintains if the node is visited
	for (int i = 1; i < size; i++) {
		bfs[i] = 0;
		visited[i] = 0;
	}
	EnQueue (Q, search);
	while (!IsEmpty (Q)) {
		//Accessing the first node in the queue and marking it's neighbours to
		//be searched
		search = DeQueue (Q);
		for (int i = 1; i < size; i++) {
			if (visited[i] == 0) {
				if (map[search][i] != INF) {
					//Adding weight to bfs if the node is connected
					bfs[i] = map[search][i] + bfs[search];
					//In that case, neighbours of this node needs to be scanned
					//too
					EnQueue (Q, i);
					visited [i] = 1;
				}
			}
		}
	}
	return bfs; 
}

/*
	*Function name: heuristic (int)
	*Input: Source of the search tree
	*Output: An array with cost (heuristic) of traversing all nodes from the source
	*Logic: Calls BFS to calculate the heuristic
	*Example Call: int *cost = heuristic(source);
*/
int* heuristic (int destination) {
	int *heuris = BFS (destination);	//Calculates heuristics of the node
	return heuris;
}

/*
	*Function name: fCostCalc (int, int)
	*Input: gCost and hCost (heuristic)
	*Output: Single integer which is the fCost
	*Logic: Adds gCost and hCost to return the fCost.
	*Example Call: int fCost = fCostCalc (gCost, hCost);
*/
int fCostCalc (int gCost, int heuristic) {
	return gCost + heuristic;
}

/*
	*Function name: extractMin (int[], int[], int) 
	*Input: openList, array of fCosts and maximum number of Nodes
	*Output: Single integer returning the index of the node in open list with
			lowest cost
	*Logic: Starts index of min at -1. It then checks for all the elements in
			the open list in one scan while also checking their cost and
			updating min in the process.
	*Example Call: int min = extractMin (openList, fCost, size);
*/
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

/*
	*Function name: reverse (int *, int)
	*Input: Array to be reversed and the size of the array
	*Output: Reversed array (due to the use of pointer)
	*Logic: Maintains a pointer at both ends of the array and while the left
			index is strictly less than the right one, it swaps those. It then
			increases the left pointer while decreasing the right one.
	*Example Call: reverse (arr, size);
*/
void reverse (int *Rev, int Size) {
	int j = Size-1, i = 0, temp;
	while(i < j) {
		temp = Rev[i];
		Rev[i] = Rev[j];
		Rev[j] = temp;
		i++;
		j--;
	}
}

/*
	*Function name: pathFind (int*, int, int*)
	*Input: An array that stores the parent of each node as returned by dStar,
			The destination we need to reach and a pointer which will be
			updated to store the number of nodes required to reach the
			destination
	*Output: An array starting from the source at zeroth index and contains
			the path all the way to the destination.
	*Logic: Traces back tha parent from the destination until it reaches the
			the source (where the index of the parent is -1.) In the process,
			it also keeps updating the number of nodes in the path from source
			to the destination.	
	*Example Call: int *path = pathFind (parent, destination, &pathSize);
*/
int* pathFind (int *parent, int destination, int *pathSize) {
	int *path = (int*) malloc (size*sizeof(int));
	*pathSize = 1;			//There is at least one element (destination)
	int i = destination, j = 1;
	path[0] = destination;
	//Creates a path 
	while (parent[i] != -1) {
		(*pathSize)++;
		path[j++] = parent[i];
		i = parent[i];
	}
	path[j] = -1;
	//Corrects the order of the path
	reverse (path, *pathSize);
	/*for (int i = 0; i < *pathSize; i++)
		cout << path[i] << "\t";
	cout << endl;
	*/
	return path;
}

#endif		//__DSTAR_REQUIREMENT__