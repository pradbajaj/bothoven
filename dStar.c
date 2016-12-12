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
int* move (int path[], int count){

}

//Returns distance of each point from the searching point. 
int* BFS (int search){
	struct Queue Q = NewQueue();	//Initializing the queue required to 
									//maintan nodes to be visited
	int *bfs = new bfs [size], source = search;
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
int* heuristic (int destination, int source, int *sDistance){
	int *heuris = BFS (destination);	//Calculates heuristics of the node
	sDistance = BFS (source);		//Returns source distance for dStar
	return heuris;
}

//Executes dStar and moves the bot from source to destination. Returns if move 
//was successful
int dStar (int source, int dest){
	int *sDistance = new int [size];
	int *heuris = heuristic (dest, source, sDistance);
	int tDistance[size];
	int path [size], count;
	//Initializing total distance (from source and destination) which
	//needs to be minimised
	for (int i = 1; i < size; i++) {
		tDistance[i] = sDistance[i] + heuris[i];
	}
	while (1) {
		int *condition = move (path, count);
		//Movement is successful
		if (condition[0] == 0) {
			break;
		}
		source = 
	}
	return 0;
}