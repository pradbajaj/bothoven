#define size 5
#define INF 100000

struct Triangle {
	int index;
	int type;					//1 signifies l type triangle and 2 signifies r type
	int nodes[size][size];		//Stores connectivity of nodes within the triangle
};

//Setting up triangle as l type or r type
struct Triangle* setUp (int type) {
	struct Triangle temp = (struct Triangle*) malloc (sizeof (struct Triangle));	//Check if the syntax is correct 
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

#undef size