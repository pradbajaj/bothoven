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

struct Node * NewNode() {
	struct Node *temp = (struct Node*) malloc (sizeof(struct Node));
	temp->next=NULL;
	return temp;
}
struct Queue * NewQueue() {
	struct Queue *Q = (struct Queue*) malloc (sizeof(struct Queue));
	Q->head=NULL;
	Q->tail=NULL;
	return Q;
}

void EnQueue(struct Queue *Q, int data) {
	struct Node * temp= NewNode();
	temp->data=data;
	if(Q->head==NULL && Q->tail==NULL) {
		Q->head=temp;
		Q->tail=temp;
		return;
	}
	Q->tail->next=temp;
	Q->tail=temp;
}

int DeQueue(struct Queue *Q) {
	if(Q->head==NULL && Q->tail==NULL) {
		return -1;
	}
	struct Node *temp = Q->head;
	int data = temp->data;
	Q->head=Q->head->next;
	if(Q->head==NULL)
		Q->tail=NULL;
	free (temp);	
	return data;
}

void EmptyQueue(struct Queue *Q) {
	while(Q->head!=NULL)
		DeQueue(Q);
}

int IsEmpty(struct Queue *Q){
	return (Q->head==NULL) ? 1 : 0 ;
}

#endif			//__QUEUE_DYNAMIC_H__
