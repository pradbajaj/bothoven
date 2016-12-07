
#include <iostream.h>
using namespace std;

void enqueue(int queue[], int &front, int &rear, int element, int size)
{
	if(front < size-1 && front +1 != rear)
	{
		front++;
		queue[front] = element;
		cout<<"element is added"<<endl;
	}
	else
	if(front = size-1 && rear != -1)
	{
		front = 0;
		queue[front] = element;
		cout<<"element is added"<<endl;
	}
	else
	{
		cout<<"Queue is full"<<endl;
	}
}
void dequeue(int queue[], int &front, int &rear, int size)
{
	if(front == -1 && rear == -1)
	{
		cout<<"Queue is Empty"<<endl;
	}
	else
	if (rear < size-1)
	{
		rear++;
		cout<<"element is deleted"<<endl;
	}
	else
	if (rear == size-1)
	{
		rear = 0;
		cout<<"element is deleted"<<endl;
	}
	if (rear == front)
	{
		rear = -1;
		front = -1;
		cout<<"element is deleted"<<endl;
	}
}
void print(int queue[], int front, int rear, int size)
{
	if (front == -1 && rear == -1)
	{
		cout<<"Queue is Empty"<<endl;
	}
	
}
int main()
{
	int option;
	char ch = 'y';
	int size,front = -1,rear = -1;
	cout<<"Enter the max size of queue"<<endl;
	cin>>size;
	int queue[size];

	while(ch == 'y')
	{
		system("cls");
		cout<<"Implementation of :-\n\t1. Enqueue\n\t2. Dequeue\n\t3. Exit"<<endl;
		cin>>option;
		if (option == 1)
		{
			int element;
			cout<<"Enter the element"<<endl;
			cin>>element;
			enqueue(queue,front,rear,element,size);
		}
		else
		if (option == 2)
		{
			dequeue(queue,front,rear,size);
		}
		else
			exit(0);
		cin.ignore();
		cout<<"Do you want to test another Implementation [y/n]"<<endl;
		cin>>ch;
	}
	system("pause");
	return 0;
}
