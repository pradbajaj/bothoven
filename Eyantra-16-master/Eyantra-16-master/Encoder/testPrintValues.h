void testPrintValues() {
	int i, j;
	printf("Nodes array:\n");
	for (i = 0; i < 49; i++) {
		printf("\tIndex %d:\n", i);
		printf("\t\tx: %d\n", nodes[i].x);
		printf("\t\ty: %d\n", nodes[i].y);
		printf("\t\tedge Count: %d\n", nodes[i].edgeCount);
		printf("\t\tedge Indices:");
		for (j = 0; j < nodes[i].edgeCount; j++) {
			printf(" %d", nodes[i].edgeIndices[j]);
		}
		printf("\n\n");
	}
	printf("Edges array:\n");
	for (i = 0; i < 63; i++) {
		printf("\tIndex %d:\n", i);
		printf("\t\tStart: %d\n", edgeList[i].start);
		printf("\t\tEnd: %d\n", edgeList[i].end);
		printf("\t\tObstacle: %d\n", edgeList[i].obstacle);
		printf("\t\tStart Compass: %d\n", edgeList[i].startCompass);
		printf("\t\tLeft Speed Ratio: %d\n", edgeList[i].leftSpeedRatio);
		printf("\t\tRight Speed Ratio: %d\n", edgeList[i].rightSpeedRatio);
		printf("\t\tDistance: %d\n\n", edgeList[i].distance);
	}
}