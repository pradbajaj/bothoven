struct vertice;
int distance_Sharp=0;
struct edge { int obstacle, angle, endA, endB, distance; };
struct vertice { int n, x, y, edgeCount, edgePoints[6]; }; //x,y are xy coordinates
struct pathNode { int node, angle, distance; }; //user -> 1: Master, 0: Slave
struct vertice verticeList[49];
struct edge edgeList[63];
struct pathNode pathM[40], pathS[40], path[40]; //path for storing shortest distance
int pathLenM, pathLenS, pathLen; //pathLen for storing length of path

/*
* Function Name:	addToArr
* Input:			finalDest[] (array to store final array), a, b, c, d, e, f (variables to store in array)
* Output:			values will be stored to array
* Logic:			Initialization
* Example Call:		addToArr(finalDest[],1,2,3,4,5,6);
*/

void addToArr(int finalDest[], int a, int b, int c, int d, int e, int f) {
	//a,b,c,d,e,f are elements to be stored in finalDest array
	finalDest[0] = a; finalDest[1] = b; finalDest[2] = c;
	finalDest[3] = d; finalDest[4] = e; finalDest[5] = f;
}

/*
* Function Name:	getFinalDest
* Input:			dest (MNP number), finalDest[] (array to store final array)
* Output:			length of the resultant array will be returned after storing result in final array
* Logic:			Store the values to array
* Example Call:		getFinalDest(3,finalDest[]);
*/

int getFinalDest(int dest, int finalDest[]) {
	if (dest < 25) {
		finalDest[0] = dest;
		return 1;
	} else {
		switch(dest) {
			case 25: addToArr(finalDest,1,25,26,27,0,0); return 4;
			case 26: addToArr(finalDest,25,26,28,29,30,31); return 6;
			case 27: addToArr(finalDest,5,28,29,32,0,0); return 4;
			case 28: addToArr(finalDest,9,33,34,35,0,0); return 4;
			case 29: addToArr(finalDest,33,34,36,37,38,39); return 6;
			case 30: addToArr(finalDest,13,36,37,40,0,0); return 4;
			case 31: addToArr(finalDest,17,41,42,43,0,0); return 4;
			case 32: addToArr(finalDest,41,42,44,45,46,47); return 6;
			case 33: addToArr(finalDest,21,44,45,48,0,0); return 4;
		}
	}
	return 0;
}

/*
* Function Name:	getOtherSide
* Input:			tempE (edge number), tempG (vertice number)
* Output:			Other side of edge's vertice number will be returned
* Logic:			Take the edge, if the one end is same as given vertice, return other vertice, else return current vertice
* Example Call:		getOtherSide(1,1);
*/

int getOtherSide(int tempE, int tempG) {
	if (edgeList[tempE].endA != tempG) return edgeList[tempE].endA;
	return edgeList[tempE].endB;
}

/*
* Function Name:	dequeue
* Input:			q[] (queue array), qLen (current length of queue), n (location to dequeue)
* Output:			Element will be dequeued from the queue
* Logic:			Normal dequeue logic
* Example Call:		dequeue(q,5,3);
*/

void dequeue(int q[],int qLen,int n) { // n -> position to dequeue
	int i; //for iteration
	for (i = n; i < qLen-1; i++) {
		q[i] = q[i+1];
	}
}

/*
* Function Name:	enqueue
* Input:			q[] (queue array), qLen (current length of queue), n (location to enqueue), x (element to enqueue)
* Output:			Element will be enqueued to the queue
* Logic:			Normal enqueue logic
* Example Call:		enqueue(q,5,3,23);
*/

void enqueue(int q[],int qLen,int n,int x) { // n -> position to enqueue, x -> element to enqueue
	int i; //for iteration
	for (i = qLen-1; i >= n; i--) {
		q[i+1] = q[i];
	}
	q[n] = x;
}

/*
* Function Name:	addObstacle
* Input:			a & b (two vertices between which obstacle is detected)
* Output:			The obstacle variable of the edge will be set to 1
* Logic:			The edge will be searched by iterating through the edge list & the variable is set 1
* Example Call:		addObstacle(1,24);
*/

void addObstacle(int a, int b) { //vertice numbers, the edge between whose is an obstacle
	int i; //for iteration
	a--; b--;
	for (i = 0; i < 63; i++) {
		if ((edgeList[i].endA==a && edgeList[i].endB==b) || (edgeList[i].endA==b && edgeList[i].endB==a))
			edgeList[i].obstacle = 1;
	}
}

/*
* Function Name:	init_graph
* Input:			None
* Output:			Initialize the track using Graph data structure
* Logic:			The vertices are numbered from 0 to 48 and edges are numbered from 0 to 62
*					Each vertice is assigned their x,y coordinates
*					Each edge is assigned their end vertices, presence of obstacles (initially 0 for all edges) and distance
* Example Call:		init_graph();
*/

void init_graph() {
	//verticeXY -> XY coordinates of all vertices
	//edgeEnds -> The end points of every edge
	//edgeAngles -> The angle with x-axis of each edge
	//edgeDist -> The distance of the edge
	//i -> for iteration
	int verticeXY[98] = {57,275,63,219,84,169,117,121,164,89,216,65,271,59,328,63,381,88,426,125,463,168,480,221,489,276,480,330,458,384,425,425,381,458,328,484,273,489,214,485,161,461,116,428,83,382,63,333,116,223,146,278,114,329,147,167,209,167,241,217,206,272,240,112,397,164,333,166,302,115,429,225,397,277,336,276,304,223,430,329,305,439,334,382,398,386,241,437,214,382,240,333,302,328,145,382,271,275},
	edgeEnds[126] = {1,2,1,24,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,22,23,23,24,3,25,25,26,26,27,27,23,25,28,28,29,29,30,30,31,31,26,3,28,32,7,32,29,7,35,35,34,34,33,33,11,33,36,36,37,37,38,38,39,39,34,11,36,37,40,40,15,43,15,43,42,41,42,41,19,42,47,47,46,46,45,45,44,44,41,19,44,45,48,48,23,26,45,29,34,37,42},
	edgeAngles[63] = {82,-82,67,52,37,22,7,-7,-22,-37,-52,-67,-82,-97,-112,-127,-142,-157,-172,172,157,142,127,112,-60,-60,-120,-120,60,0,-60,-120,180,0,60,-120,-60,-60,0,0,-60,-120,180,120,60,-120,-60,-60,0,180,60,-120,120,180,-120,-60,0,120,180,180,-60,0,-120},
	edgeDist[63] = {20,21,19,21,20,20,20,20,21,21,20,20,20,19,21,18,20,21,20,21,21,20,20,18,22,21,21,22,21,21,21,23,21,23,22,23,23,21,23,24,24,22,22,22,23,24,22,22,21,23,23,21,22,22,20,22,23,22,25,22,45,45,44},
	i;
	for (i = 0; i < 49; i++) { //Vertice Initialization
		verticeList[i].n = i;
		verticeList[i].x = verticeXY[2*i];
		verticeList[i].y = verticeXY[2*i+1];
		verticeList[i].edgeCount = 0;
	}
	for (i = 0; i < 63; i++) { //Edge Initialization
		edgeList[i].obstacle = 0;
		edgeList[i].angle = edgeAngles[i];
		edgeList[i].endA = edgeEnds[2*i]-1;
		edgeList[i].endB = edgeEnds[2*i+1]-1;
		edgeList[i].distance = edgeDist[i];
		verticeList[edgeList[i].endA].edgePoints[(verticeList[edgeList[i].endA].edgeCount)++] = i;
		verticeList[edgeList[i].endB].edgePoints[(verticeList[edgeList[i].endB].edgeCount)++] = i;
	}
}

/*
* Function Name:	mainFun
* Input:			src (vertice number), dest (MNP number), compass (angle of bot with respect to x axis)
* Output:			It will store the shortest path from src to dest in global array path
* Logic:			Breadth First Search Algorithm is used to find the shortest distance
* Example Call:		mainFun(1,28,0);
*/

void mainFun(int src,int dest,int compass, int user) {
	int i, j, x, y, z = -1, finalDest[6], finalDestCount, prevPts[49], dist[49], q[50], qLen;
	//q[] for queue and qLen for queue length
	//x, y, z are temporary variables which have multiple functions
	//i & j for iteration
	struct vertice tempV; //used for temporary vertice
	struct pathNode tempP;
	finalDestCount = getFinalDest(dest,finalDest);
	for (i = 0; i < 49; i++) prevPts[i] = -1;
	for (i = 0; i < 49; i++) dist[i] = 20000;
	dist[src - 1] = 0;
	qLen = 1; q[0] = src - 1;
	while ( qLen>0 ) {
		for (i = 0; i < finalDestCount; i++)
			if (prevPts[finalDest[i]-1] != -1) {
				for (j = 0; j < qLen; j++) {
					if ( dist[q[j]] >= dist[finalDest[i]-1] ) {
						dequeue(q,qLen,j); j--; qLen--;
					}
				}
			}
		for (i = 0; i < qLen; i++) {
			x = q[i];
			dequeue(q,qLen,i); i--; qLen--;
			for (j = 0; j < verticeList[x].edgeCount; j++) {
				if (edgeList[verticeList[x].edgePoints[j]].obstacle == 1) continue;
				y = getOtherSide(verticeList[x].edgePoints[j],x);
				if (dist[y] > dist[x] + edgeList[verticeList[x].edgePoints[j]].distance) {
					dist[y] = dist[x] + edgeList[verticeList[x].edgePoints[j]].distance;
					prevPts[y] = x;
					enqueue(q,qLen,i+1,y); qLen++; i++;
				}
			}
		}
	}
	for (x = 0, i = 1; i < finalDestCount; i++) {
		if (dist[finalDest[i]-1] < dist[finalDest[x]-1])
			x = i;
	}
	z = finalDest[x] - 1;
	pathLen = 1;
	path[0].node = finalDest[x];
	while (prevPts[z] != -1) {
		pathLen++;
		tempV = verticeList[z];
		for (i = 0; i < tempV.edgeCount; i++) {
			x = tempV.edgePoints[i];
			if (getOtherSide(x,z) == prevPts[z]) break;
		}
		path[pathLen-1].distance = edgeList[x].distance;
		if (z == edgeList[x].endA) path[pathLen-1].angle = edgeList[x].angle + 180;
		else path[pathLen-1].angle = edgeList[x].angle;
		if (path[pathLen-1].angle > 180) path[pathLen-1].angle -= 360;
		else if (path[pathLen-1].angle <= -180) path[pathLen-1].angle +=360;
		path[pathLen-1].angle = -path[pathLen-1].angle;
		z = prevPts[z];
		path[pathLen-1].node = z+1;
	}
	path[pathLen++].node = src;
	for (i = 0; i < pathLen/2; i++) {
		tempP = path[i];
		path[i] = path[pathLen-i-1];
		path[pathLen-i-1] = tempP;
	}
	for (i = 1; i < pathLen; i++) {
		x = path[i].angle;
		path[i].angle -= compass;
		if (path[i].angle >= 180) path[i].angle -= 360;
		else if (path[i].angle <= -180) path[i].angle += 360;
		compass = x;
	}
	if (user) {
		for (i = 0; i < pathLen; i++) pathM[i] = path[i];
		pathLenM = pathLen;
	} else {
		for (i = 0; i < pathLen; i++) pathS[i] = path[i];
		pathLenS = pathLen;
	}
}
