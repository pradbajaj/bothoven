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

#ifndef __BFS_PATH_FIND__
#define __BFS_PATH_FIND__

#include "DynamicQueue.h"
#include "mapRun.h"
#include <stdlib.h>
#include <math.h>

//const int size = 49;
// #define INF 600000

#include "adjacency.h"

/*
	*Function name: reverse (int *, int)
	*Input: Array to be reversed and the size of the array
	*Output: Reversed array (due to the use of pointer)
	*Logic: Maintains a pointer at both ends of the array and while the left
			index is strictly less than the right one, it swaps those. It then
			increases the left pointer while decreasing the right one.
	*Example Call: reverse (arr, size);
*/
void reverse (signed int *Rev, signed int path_Size) {
	lcd_cursor(1,1);
	lcd_string("   Waiting012   ");
	signed int j = path_Size-1, i = 0, temp;
	while(i < j) {
		temp = Rev[i];
		Rev[i] = Rev[j];
		Rev[j] = temp;
		i++;
		j--;
		lcd_cursor(1,1);
		lcd_string("   Waiting013   ");
	}
	lcd_cursor(1,1);
	lcd_string("   Waiting014   ");
}
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
signed int* Move (signed int path[], signed int pathSize) {
	lcd_cursor(1,1);
	lcd_string("   Waiting220   ");
	signed int *angle = (signed int*) malloc((pathSize - 1)*sizeof(signed int)); // addded the prefix so pathSize needs to be -= 1
	// I am not understanding the role of i in for loop
	// What if pathSize == 2
	// that way it won't enter the loop
	// for example if we want to move from node 25 to 26
	// path would be 25,26,-1 and pathSize would be 2
	// Suggestion :  I think j < pathSize-1 and add a prefix which will tell initial direction of robot
	lcd_cursor(1,1);
	lcd_string("   Waiting020   ");
	for (signed int j = 0, i = 1; i < pathSize-1; i++, j++) {
		//This could be buggy
		signed int initial_angle, final_angle;
		for (signed int index = 0; index < 5; ++index) {
			if (map[path[i]][index] == path[i+1]) {
				final_angle = map_angle[path[i]][index];
				break;
			}
			lcd_cursor(1,1);
			lcd_string("   Waiting021   ");
		}
		for (signed int index = 0; index < 5; ++index) {
			if (map[path[i-1]][index] == path[i]) {
				initial_angle = map_angle[path[i-1]][index];
				break;
			}
			lcd_cursor(1,1);
			lcd_string("   Waiting022   ");
		}
		angle[j] = final_angle - initial_angle;
		lcd_cursor(1,1);
		lcd_string("   Waiting023   ");
	}
	angle[pathSize-2] = -1;
	for (int i = 0; i < pathSize-1; ++i) {
		lcd_print(2,(i+1)*3,abs(angle[i]),3);
	}
	_delay_ms(5000);
	lcd_cursor(1,1);
	lcd_string("   Waiting024   ");
	signed int *res = mapRun (angle, pathSize);
	if (res[0] != 0) {
		//Assuming that mapRun returns the index of the nodes in res[1] and res[2]	
		// I think count+1 will be the index
		res[1] = path[res[1] + 1];
		res[2] = path[res[2] + 1];
	}
	lcd_cursor(1,1);
	lcd_string("   Waiting025   ");
	free(angle);
	return res;
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
signed int* pathFind (signed int *parent, signed int destination, signed int *pathSize) {
	lcd_cursor(1,1);
	lcd_string("   Waiting010   ");
	signed int *path = (signed int*) malloc (size*sizeof(signed int));
	*pathSize = 1;			//There is at least one element (destination)
	signed int i = destination, j = 1;
	path[0] = destination;
	//Creates a path 
	while (parent[i] != -1) {
		(*pathSize)++;
		path[j++] = parent[i];
		i = parent[i];
		lcd_cursor(1,1);
		lcd_string("   Waiting011   ");
	}
	path[j] = -1;
	lcd_cursor(1,1);
	lcd_string("   Waiting111   ");
	//Corrects the order of the path
	reverse (path, *pathSize);
	/*for (int i = 0; i < *pathSize; i++)
		cout << path[i] << "\t";
	cout << endl;
	*/
	lcd_cursor(1,1);
	lcd_string("   Waiting015   ");
	return path;
}

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
signed int* BFS (signed int source, signed int destination, signed int *pSize) {
	lcd_cursor(1,1);
	lcd_string("   Waiting004   ");
	struct Queue *Q = NewQueue();	//Initializing the queue required to 
									//maintain nodes to be visited
	signed int *bfs, current = source, *parent;
	parent = (signed int *) malloc (size*sizeof(signed int));
	bfs = (signed int *) malloc (size*sizeof (signed int));
	signed int visited[size];				//Maintains if the node is visited
	for (signed int i = 1; i < size; i++) {
		bfs[i] = 0;
		visited[i] = 0;
		parent[i] = -1;
		lcd_cursor(1,1);
		lcd_string("   Waiting005   ");
	}

	bfs[current] = 1;
	visited[current] = 1;
	parent[source] = -1;
	
	EnQueue (Q, current);
	signed int flag = 1;	
	while (!IsEmpty (Q) && flag == 1) {
		//Accessing the first node in the queue and marking it's neighbours to
		//be searched 
		current = DeQueue (Q);
		for (signed int i = 0, j = map[current][0]; map[current][i] != -1; ++i, j = map[current][i]) {
			// I think i should be replaced by j in the following lines
			if (visited[j] == 0) {
				//Adding weight to bfs if the node is connected
				bfs[j] = 1 + bfs[current];
				//In that case, neighbours of this node needs to be scanned
				//too
				parent[j] = current;			
				EnQueue (Q, j);
				visited [j] = 1;				
				if (j == destination) {			
					flag = 0;	//Found a path to destination
					break;
				}	
			}
			lcd_cursor(1,1);
			lcd_string("   Waiting006   ");
		}
		lcd_cursor(1,1);
		lcd_string("   Waiting007   ");
	}
	lcd_cursor(1,1);
	lcd_string("   Waiting008   ");
	free (bfs);
	EmptyQueue(Q);
	if (parent[destination] == -1){
		signed int *path = (signed int*) malloc (sizeof(signed int));
		path[0] = -2;
		return path;
	}
	lcd_cursor(1,1);
	lcd_string("   Waiting009   ");
	signed int *path = pathFind(parent, destination, pSize);
	lcd_cursor(1,1);
	lcd_string("   Waiting016   ");
	return path; 
}

signed int* BFSPathFind (signed int source, signed int destination, signed int prefix) {		//prefix will be the additional node which has to be added
	lcd_cursor(1,1);
	lcd_string("   Waiting002   ");
	signed int *ret = (signed int *) malloc (2*sizeof(signed int));						//in front of the path to calculate correct angles
	while (1) {
		signed int *pSize;	
		lcd_cursor(1,1);
		lcd_string("   Waiting003   ");
		signed int *path = BFS (source, destination, pSize);
		//In case no path is possible, return 1 for failure and the node the bot is standing on.	
		if (path[0] == -2){
			ret[0] = source;			// If failed it will return current node at 0th index
			ret[1] = prefix;			// and previous node at 1st index
			free(path);
			break;
		}
		lcd_cursor(1,1);
		lcd_string("  Waiting017");
		int xyz = *pSize;
		lcd_print(1,11,xyz,5);
		_delay_ms(1000);
		(*pSize)++;										//adding prefix into the path
		for (signed int i = pSize-1; i >= 0; --i) {
			path[i+1] = path[i];
			lcd_cursor(1,1);
			lcd_string("   Waiting018   ");
		}

		path[0] = prefix;
		lcd_cursor(1,1);
		lcd_string("   Waiting019   ");
		
		signed int *res = Move (path, pSize);
		// free (pSize);		//We need pSize and path when res[0] == 0
		// free (path);			//So copying this statement to if-else condition seperately
		//If movement is successful, the function will return 0, and the destination
		//node that the bot has reached.
		lcd_cursor(1,1);
		lcd_string("   Waiting026   ");
		if (res[0] == 0) {
			ret[0] = 0;
			// ret[1] = destination;
			ret[1] = path[*(pSize)-2];		//We need a last node robot reached before reaching destination
			free (res);
			free (pSize);
			free (path);
			break;
		} else { //Otherwise try to run it again.
			for (signed int i = 0, cur = res[1]; map[cur][i+1] != -1; i++){
				if (map[cur][i] == res[2]) {
					while (map[cur][i] != -1) {
						map[cur][i] = map[cur][i+1];
						map_angle[cur][i] = map_angle[cur][i+1];		//Along with map we need to change the angles
						++i;
						lcd_cursor(1,1);
						lcd_string("   Waiting027   ");
					}
					break;
				}
				lcd_cursor(1,1);
				lcd_string("   Waiting028   ");
			}
			lcd_cursor(1,1);
			lcd_string("   Waiting029   ");
			for (signed int i = 0, cur = res[2]; map[cur][i+1] != -1; i++){
				if (map[cur][i] == res[1]) {
					while (map[cur][i] != -1) {
						map[cur][i] = map[cur][i+1];
						map_angle[cur][i] = map_angle[cur][i+1];		//Along with map we need to change the angles
						++i;
						lcd_cursor(1,1);
						lcd_string("   Waiting030   ");
					}
					break; 
				}
				lcd_cursor(1,1);
				lcd_string("   Waiting031   ");
			}
			lcd_cursor(1,1);
			lcd_string("   Waiting032   ");
		}
		lcd_cursor(1,1);
		lcd_string("   Waiting032   ");
		source = res[1];
		free (pSize);
		free (path);
		free (res);
		free (ret);
	}
	lcd_cursor(1,1);
	lcd_string("   Waiting033   ");
	return ret;
}

#endif		//__BFS_PATH_FIND__

