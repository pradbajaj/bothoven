#include <stdio.h>
#include "graph_temp.h"

int main() {
	int musicNodes[] = {1,7,29,26,18,24,13,30,16,20}, noteCount = 10,
	mNotes[10], sNotes[10], mNoteCount, sNoteCount,
	i, j, masterNode = 1, masterCompass = 0, slaveNode = 12, slaveCompass = 180, distM, distS, tempM, tempS;
	init_graph();
	distM = distS = mNoteCount = sNoteCount = 0;
	for (i = 0; i < noteCount; i++) {
		mainFun(masterNode, musicNodes[i], masterCompass, 1);
		mainFun(slaveNode, musicNodes[i], slaveCompass, 0);
		tempM = tempS = 0;
		for (j = 1; j < pathLenM-1; j++)
			tempM += pathM[j].distance;
		for (j = 1; j < pathLenS-1; j++)
			tempS += pathS[j].distance;
		if (distM + tempM < distS + tempS) {
			mNotes[mNoteCount++] = musicNodes[i];
			distM += tempM;
			masterNode = pathM[pathLenM-1].node;
		} else {
			sNotes[sNoteCount++] = musicNodes[i];
			distS += tempS;
			slaveNode = pathS[pathLenS-1].node;
		}
		printf("Iteration %d : %d %d\n", i, tempM, tempS);
	}
	printf("Master\n");
	for (i = 0; i < mNoteCount; i++)
		printf("%d ", mNotes[i]);
	printf("\nSlave\n");
	for (i = 0; i < sNoteCount; i++)
		printf("%d ", sNotes[i]);
	/*for (i =  0; i < noteCount; i++) {
		mainFun(masterNode, musicNodes[i], masterCompass);
		for (j = 1; j < pathLen-1; j++) {
			printf("%d %d %d\n", path[j].node, path[j].angle, path[j].distance);
			masterCompass += path[j].angle;
			if (masterCompass > 180) masterCompass -= 360;
			if (masterCompass < -180) masterCompass += 360;
			printf("Compass = %d\n", masterCompass);
		}
		printf("Beep %d\n", path[j].node);
		masterNode = path[j].node;
	}*/
	return 0;
}