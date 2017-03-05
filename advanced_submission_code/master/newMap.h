/*
	*Team ID: eYRC-BV#1651
	*Author List: Aayush, Pradyumna, Pranjal, Shashwat
	*filename: newMap.c 
	*Theme: Bothoven
	*Functions: setUp (int), setLink (struct Triangle*, struct Triangle*),
				updateTriange (struct Triangle*, int, int, int), 
				updateTriangleLink (struct TriangleLink*, int, int, int)
				dStar (struct Triangle*, struct Triangle*, struct TriangleLink*
				, int, int)
	*Global Variable: struct Triangle, struct TrianlgeLink
*/

/*
	*********************************NOTE**************************************
	*This file is not actually used at this stage
	*We currently think that this will be a better implementation, if we
	*succeed with it. Here we noticed that the map can be broken down into 
	*6 triangles. Of those triangles, 3 are exactly identical and other 3 are
	*mirror image of these. We also need to maintain a method to link between
	*these two triangles which will be stored in TriangleLink.
	*We think that this method is better because this way our dStar will need
	*to be applied on even smaller set of nodes and that will result in some
	*serious time complexity improvements. However, we don't think this will
	*result in any memory improvement as we'll still need to maintain data of
	*every node (although in a different manner) because the position of the 
	*obstacles is not already known.
	*********************************END***************************************
*/

#define size 5
#define INF 100000

struct Triangle {
	int index;
	int type;					//1 signifies l type triangle and 2 signifies r type
	int nodes[size][size];		//Stores connectivity of nodes within the triangle
};

//Setting up triangle as l type or r type
struct Triangle* setUp (int type) {
	struct Triangle *temp = (struct Triangle*) malloc (sizeof (struct Triangle));	//Check if the syntax is correct 
	//Maintains links of nodes that are connected	
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
			temp->nodes[i][j] = INF;
	temp->nodes[1][2] = temp->nodes[2][1] = 1;
	temp->nodes[2][3] = temp->nodes[3][2] = 1;
	temp->nodes[2][4] = temp->nodes[4][2] = 1;
	temp->type = type;												//Assigns type to the triangle
	return temp;
}

//Maintains links in l type to r type transition
struct triangleLink {
	int links[size][size];
	//linkType maintains the type of link, lID maintains the id of triangle towards the left
	//And rID maintains the ID of triangle towards the right
	//For linkType, 1 indicates l to r and 2 indicates r to l
	int linkType, lID, rID;		
};

struct triangleLink* setLink (struct Triangle *left, struct Triangle *right) {
	struct triangleLink *temp = new struct triangleLink ();
	temp->linkType = left->type;
	temp->lID = left->index;
	temp->rID = right->index;
	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
			temp->links[i][j] = INF;
	//Stores links based on which triangle being converted
	switch (temp->linkType) {
		case 1:
			temp->links[4][1] = 1;
			temp->links[3][3] = 1;
			break;
		case 2:
			temp->links[2][2] = 1;
			break;
	}
	return temp;
}

//Updates triangle if obstacle in encounterd
void updateTriangle (struct Triangle *update, int nodeL, int nodeR, int weight) {
	update->nodes[nodeL][nodeR] = weight;
}

//Updates link weights if obstacle is encountered
void updateLink (struct triangleLink* update, int nodeL, int nodeR,int weight) {
	update->links[nodeL][nodeR] = weight;
}

void dStar (struct Triangle *Left, struct Triangle *Right, 
			struct triangleLink *connect, int nodeL, int nodeR) {
	//Assigning move type from left to right or right to left
	int pathType = ((Left->type == 1) ? 1 : 2);

}

#undef size
#undef INF