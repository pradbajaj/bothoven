#include <cstdlib>

int choice;
struct Node {
	int data;
	struct Node* next;
};
struct Queue {
	struct Node *head, *tail;  
};

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

bool IsEmpty(struct Queue *Q){
	return (Q->head==NULL) ? true : false ;
}