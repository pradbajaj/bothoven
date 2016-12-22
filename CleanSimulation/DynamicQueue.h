#ifndef __QUEUE_DYNAMIC_H__
#define __QUEUE_DYNAMIC_H__

#include <cstdlib>

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
bool IsEmpty(struct Queue *Q);

#endif			//__QUEUE_DYNAMIC_H__