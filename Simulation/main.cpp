#include "master_visited.h"
#include "adjacency.h"
#include "BFSPathFind.h"
#include <iostream>
using namespace std;
/*
	*Team ID: eYRC-BV#1651
	*Author List: Aayush, Pradyumna, Pranjal, Shashwat
	*filename: main.c
	*Theme: Bothoven
	*Functions: main()
	*Global Variable: NIL
*/

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
	int prefix = 24;
	for (int i = 0; i < 9; i++) {
		int * res = BFSPathFind (Nodes[i], Nodes[i+1], prefix);
		if (res[0] == 0){
			prefix = Nodes[i];
		} else{
			prefix = res[1];
		}
		cout << endl;
	}
	return 0;
}
