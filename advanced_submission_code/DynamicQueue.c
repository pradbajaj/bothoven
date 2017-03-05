/*
	*Team ID: eYRC-BV#1651
	*Author List: Aayush, Pradyumna, Pranjal, Shashwat
	*File name: DynamicQueue.c
	*Theme: Bothoven
	*Functions: NewNode(), NewQueue(), EnQueue(), DeQueue(), EmptyQueue(),
				IsEmpty()
	*Global Variables: NIL
*/
#include "DynamicQueue.h"

/*
	******************************NOTES****************************************
	*The functions in this file simply implements a dynamic queue.
	*NewNode() creates a new Node for the queue.
	*NewQueue() initializes a new queue.
	*Enqueue() adds a new node into the queue.
	*Dequeue() removes the node from the queue in FIFO order.
	*EmptyQueue() deletes every single element from the queue.
	*IsEmpty() checks and returns if the queue is empty.
	******************************END******************************************
*/

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