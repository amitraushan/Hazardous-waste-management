#include <stdio.h>
#include <stdlib.h>
struct queue
{
    struct queue *next;
    struct queue *prev;
    int num;
}*root=NULL,*end=NULL;
int que_length=0;
void enqueue(int a)
{
    struct queue *newnode;
    if(root==NULL)
    {
        root=(struct queue*)malloc(sizeof(struct queue));
        root->num=a;
        root->next=NULL;
        root->prev=NULL;
        end=root;
    }
    else
    {
        newnode=(struct queue*)malloc(sizeof(struct queue));
        newnode->num=a;
        root->next=newnode;
        newnode->prev=root;
        root=root->next;
        newnode->next=NULL;
    }
    que_length++;
}
void enq_end(int a)
{
    struct queue *newnode;
    if(end==NULL)
    {
        end=(struct queue*)malloc(sizeof(struct queue));
        end->num=a;
        end->next=NULL;
        end->prev=NULL;
        root=end;
    }
    else
    {
        newnode=(struct queue*)malloc(sizeof(struct queue));
        newnode->num=a;
        end->prev=newnode;
        newnode->next=end;
        end=end->prev;
        newnode->prev=NULL;
    }
    que_length++;
}
void dequeue(void)
{
    if(que_length!=0)
    {
        struct queue *newnode;
        newnode=end;
        end=end->next;
        free(newnode);
        que_length--;
    }
    else
        return;
}
main()
{
    enqueue(14);
	enqueue(17);
	enqueue(14);
	enqueue(0);
	enqueue(5);
	enqueue(25);
    dequeue();
    dequeue();
    dequeue();
    dequeue();
    dequeue();
    printf("%d",root->num);
}
