#include "QueueDynamic.h"
int main()
{
	struct Queue *Q = NewQueue();
	int data;
	do
	{
		system("cls");
		cout<<"\n\n  Current Queue : ";
		Print(Q);
		cout<<"\n\n        Menu";
		cout<<"\n 1. EnQueue ";
		cout<<"\n 2. DeQueue ";
		cout<<"\n 3. Empty Stack ";
		cout<<"\n 4. Free Stack and exit";
		cout<<"\n\n Enter your choice  => ";
		cin>>choice;
		switch(choice)
		{
			case 1 : 	cout<<"\n Enter the value to be enqueued  => ";
						cin>>data;
						EnQueue(Q,data);
						break;
			case 2 : 	data=DeQueue(Q);
						(data!=-1) ? cout<<"\n The dequeued value is : "<<data : cout<<" ";  
						break;
			case 3  :	EmptyQueue(Q);
						break;
			case 4  : 	FreeQueue(Q);
						break;
			default :	cout<<"\n\n  Please enter a valid option (-_- ) ";
						break;			
		}
		
	} while(choice != 4);
	
	
}
