/*********************************POSSIBLE ISSUES******************************
	*Reliability of BFS in case of weighted graph
	*Initialize min
	*Check the value of pathSize at the end of pathFind
******************************************************************************/

#include "QueueDynamic.h"
#include <cstdlib>
#define size 49
extern int map[][];
extern int map_link[][];
extern int map_angle[][];
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
		Rev[i] = Rev[j];
		Rev[j] = temp;
		i++;
		j--;
	}
}

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
