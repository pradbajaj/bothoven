// 7 , 29 , 26 , 18 , 24 , 13 , 30 , 16 , 20
//Check abs function in embeded c
#include <iostream>
#include <cmath>
using namespace std;
#define size 49
#define INF 10000
int map [size][size];
int map_link[34][6];
//Mapping angles
int map_angle[48][48];
int initMap () {
	//Initializes each nodes as unconnected
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
			map [i][j] = INF;
	//Connects nodes 1 through 24 in sequential order
	for (int i = 1; i < 25; i++)
		for (int j = 1; j < 25; j++)
			if (abs(i-j) == 1)
				map[i][j] = 1;
	//Connects nodes 25 through 38 in sequential order
	for (int i = 25; i < 39; i++)
		for (int j = 25; j < 39; j++)
			if (abs(i-j) == 1)
				map[i][j] = 1;
	//Manually connecting nodes that are connected in the map
	//Replace it with soft code if you ever find a method
	map[35][42] = map[42][35] = 1;
	map[42][41] = map[41][42] = 1;
	map[41][32] = map[32][41] = 1;

	map[32][48] = map[48][32] = 1;
	
	map[47][31] = map[31][47] = 1;
	map[31][40] = map[40][31] = 1;
	map[40][39] = map[39][40] = 1;
	map[39][28] = map[28][39] = 1;
	map[28][46] = map[46][28] = 1;
	map[45][27] = map[27][45] = 1;
	map[27][38] = map[38][27] = 1;
	map[36][25] = map[25][36] = 1;
	map[36][44] = map[44][36] = 1;
	map[43][35] = map[35][43] = 1;

	map[24][1]  = map[1][24]  = 1;

	map[23][44] = map[44][23] = 1;
	map[23][43] = map[43][23] = 1;
	map[19][34] = map[34][19] = 1;
	map[19][33] = map[33][19] = 1;
	map[15][48] = map[48][15] = 1;
	map[15][47] = map[47][15] = 1;
	map[11][30] = map[30][11] = 1;
	map[11][29] = map[29][11] = 1;
	map[7][46]  = map[46][7]  = 1;
	map[7][45]  = map[45][7]  = 1;
	map[3][26]  = map[26][3]  = 1;
	map[3][25]  = map[25][3]  = 1;

	map[35][36] = map[36][35] = 1;

	map[11][12] = map[12][11] = 1;
	//Mapping notes to nodes
	for (int i = 0; i < 25; i++)
		for (int j = 0; j < 6; j++)
			map_link[i][j] = -1;
	for (int i = 1; i < 25; i++)
		for (int j = 0; j < 6; j++)
			map_link[i][j] = (j == 0 ? i : -1);
	//Manually linking nodes to notes
	//Replace hard code with a better method if and when available
	map_link[25][0] = 1;
	map_link[25][1] = 25;
	map_link[25][2] = 36;
	map_link[25][3] = 44;
	map_link[27][0] = 5;
	map_link[27][1] = 26;
	map_link[27][2] = 27;
	map_link[27][3] = 45;
	map_link[28][0] = 9;
	map_link[28][1] = 28;
	map_link[28][2] = 29;
	map_link[28][3] = 46;
	map_link[30][0] = 13;
	map_link[30][1] = 30;
	map_link[30][2] = 31;
	map_link[30][3] = 47;
	map_link[31][0] = 17;
	map_link[31][1] = 32;
	map_link[31][2] = 33;
	map_link[31][3] = 48;
	map_link[33][0] = 21;
	map_link[33][1] = 34;
	map_link[33][2] = 35;
	map_link[33][3] = 43;
	map_link[26][0] = 25;
	map_link[26][1] = 26;
	map_link[26][2] = 27;
	map_link[26][3] = 36;
	map_link[26][4] = 37;
	map_link[26][5] = 38;
	map_link[29][0] = 28;
	map_link[29][1] = 29;
	map_link[29][2] = 30;
	map_link[29][3] = 31;
	map_link[29][4] = 39;
	map_link[29][5] = 40;
	map_link[32][0] = 32;
	map_link[32][1] = 33;
	map_link[32][2] = 34;
	map_link[32][3] = 35;
	map_link[32][4] = 41;
	map_link[32][5] = 42;

	map_angle[3][25]  = -150;
    map_angle[25][3]  = 30;
	map_angle[3][26]  = -90;
	map_angle[26][3]  = 90;
	map_angle[7][45]  = 150;
	map_angle[45][7]  = -30;
	map_angle[7][46]  = -150;
	map_angle[46][7]  = 30;
	map_angle[11][29] = 90;
	map_angle[29][11] = -90;
	map_angle[11][30] = 150;
	map_angle[30][11] = -30;
	map_angle[15][47] = 30;
	map_angle[47][15] = -150;
	map_angle[15][48] = 90;
	map_angle[48][15] = -90;
	map_angle[19][33] = -30;
	map_angle[33][19] = 150;
	map_angle[19][34] = 30;
	map_angle[34][19] = -150;
	map_angle[23][43] = -90;
	map_angle[43][23] = 90;
	map_angle[23][44] = -30;
	map_angle[44][23] = 150;
	map_angle[25][26] = -30;
	map_angle[26][25] = 150;
	map_angle[29][30] = -150;
	map_angle[30][29] = 30;
	map_angle[35][43] = 90;
	map_angle[43][35] = -90;
	map_angle[36][44] = 150;
	map_angle[44][36] = -30;
	map_angle[27][45] = -30;
	map_angle[45][27] = 150;
	map_angle[28][46] = 30;
	map_angle[46][28] = -150;
	map_angle[31][47] = -150;
	map_angle[47][31] = 30;
	map_angle[32][48] = -90;
	map_angle[48][32] = 90;
	map_angle[27][38] = -150;
	map_angle[38][27] = 30;
	map_angle[28][39] = 150;
	map_angle[39][28] = -30;
	map_angle[31][40] = 90;
	map_angle[40][31] = -90;
	map_angle[32][41] = 30;
	map_angle[41][32] = -150;
	map_angle[35][42] = -30;
	map_angle[42][35] = 150;
	map_angle[36][37] = -90;
	map_angle[37][36] = 90;
	map_angle[39][40] = -150;
	map_angle[40][39] = 30;
	map_angle[37][38] = 150;
	map_angle[38][37] = -30;
	map_angle[41][42] = 90;
	map_angle[42][41] = -90;
		
	return 0;
}




/*********************************POSSIBLE ISSUES******************************
	*Reliability of BFS in case of weighted graph
	*Initialize min
	*Check the value of pathSize at the end of pathFind
******************************************************************************/

#include "QueueDynamic.h"
#include <cstdlib>
#define size 49

/******************************************************************************
	*Follows the path and moves accordingly and Returns 3 integers with first
	*corresponding to if the movement was successful 2nd corresponds to 
	*the node the bot departed from and 3rd to the intermediate destination
******************************************************************************/
int* move (int path[], int count) {
	int *res = new int[3];
	res[0] = res[1] = res[2] = 0;
	cout << path[0] << "\t";
	for (int i = 0; i < count-1; i++) {
		if ((path[i] == 32 && path[i+1] == 48) || (path[i] == 48 && 
			path[i+1] == 32) || (path[i] == 24 && path[i+1] == 1) ||
			(path[i] == 1 && path[i+1] == 24) || (path[i] == 35 && 
			path[i+1] == 36) || (path[i] == 36 && path[i+1] == 35) || 
			(path[i] == 11 && path[i+1] == 12) || (path[i] == 11 && 
			path[i+1] == 12)) {
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
		source = result[1];
		free (heuris);
		free (pathSize);
	}
}
// 7 , 29 , 26 , 18 , 24 , 13 , 30 , 16 , 20
int main () {
	initMap ();
	int Nodes[10];
	Nodes[0] = 1;
	Nodes[1] = 7;
	Nodes[2] = 28;
	Nodes[3] = 27;
	Nodes[4] = 18;
	Nodes[5] = 24;
	Nodes[6] = 13;
	Nodes[7] = 30;
	Nodes[8] = 16;
	Nodes[9] = 20;
	for (int i = 0; i < 9; i++) {
		dStar (Nodes[i], Nodes[i+1]);
		cout << endl;
	}
}