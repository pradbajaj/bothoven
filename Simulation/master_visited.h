#ifndef __MASTER_VISITED__
#define __MASTER_VISITED__

void markVisited (int *visited){
	signed int *visited = (signed int *) calloc (size,sizeof(signed int));
	for (int i = 8; i <= 18; i++){
		visited[i] = 1;
	}
	for (int i = 28; i <= 32; i++){
		visited[i] = 1;
	}
	visited[39] = 1;
	visited[40] = 1;
	visited[46] = 1;
	visited[47] = 1;
	visited[48] = 1;
}

#endif	//__MASTER_VISITED__
