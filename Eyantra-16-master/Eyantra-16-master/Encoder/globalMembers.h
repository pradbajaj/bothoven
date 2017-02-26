struct vertices {
	int x, y; // co-ordinates
	int edgeCount;
	int edgeIndices[6];
} nodes[49];

struct edges {
	int start, end;
	int obstacle;
	int startCompass;
	int leftSpeedRatio, rightSpeedRatio;
	int distance;
} edgeList[63];

int notes[] = {1,7,29,26,18,24,13,30,16,20};
int noteCount = 10;

struct path {
	int nextNode;
	int subPath[40];
	int subPathCount;
} botA[10], botB[10];
int pathLenA, pathLenB;

struct botStatus {
	int ready;
	int node;
} botAstat, botBstat;