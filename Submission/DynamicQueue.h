/*
	*Team ID: eYRC-BV#1651
	*Author List: Aayush, Pradyumna, Pranjal, Shashwat
	*filename: DynamicQueue.h
	*Theme: Bothoven
	*Functions: NewNode(), NewQueue(), EnQueue(), DeQueue(), EmptyQueue(),
				IsEmpty()
	*Global Variable: NIL
*/

#ifndef __QUEUE_DYNAMIC_H__
#define __QUEUE_DYNAMIC_H__

#include <stdlib.h>

/*
	**********************************NOTES************************************
	*This is the header file which has the interface for dynamic queue.
	*Implementation of each function is defined in the DynamicQueue.cpp
	**********************************END**************************************
*/

int choice;
struct Node {
	int data;
	struct Node* next;
};
struct Queue {
	struct Node *head, *tail;  
};

struct Node * NewNode();
struct Queue * NewQueue();
void EnQueue(struct Queue *Q, int data);
int DeQueue(struct Queue *Q);
void EmptyQueue(struct Queue *Q);
int IsEmpty(struct Queue *Q);

#endif			//__QUEUE_DYNAMIC_H__