#define max_node 24
#include <stdio.h>
#include <conio.h>
char graph[max_node][max_node];
char p[max_node+1],path[max_node+1];
int path_index,found=0;
int pre_node=1;
void def(void);
void def(void)
{
    int i=0,j=0;
    for(i=0;i<=max_node;i++)
        for(j=0;j<=max_node;j++)
        graph[i][j]='\0';
}
int path_logic(int curr_node, int req_node)
{
    if(curr_node==req_node && found==0)
    {
        found++;
        return(1);
    }
    int j,r=0;
    for(j=0;j<max_node;j++)
    {
        if(graph[curr_node][j]!='\0' && pre_node!=j )
            {
                p[path_index]=graph[curr_node][j];
                path_index=path_index+1;
                pre_node=curr_node;
                r=r+path_logic(j,req_node);
            }
    }
    if(r>0)
    {
        return(1);
    }
    else
    {
        path_index--;
        return(0);
    }
}
void path_find(curr_node,req_node)
{
    int i;
   pre_node=curr_node;
   for(i=0;i<max_node+1;i++)
    path[i]='\0';
   path_index=1;
   found=0;
   path_logic(curr_node,req_node);
   path_index--;
    for(i=0;i<=max_node+1;i++)
    {
        path[i]='\0';
    }
   for(i=1;i<=path_index;i++)
    {
        path[i]=p[i];
    }
    path_index=1;
    i=1;
    while(path[path_index]!='\0')
    {
        if((path[i]=='E' && path[i+1]=='W') || (path[i]=='W' && path[i+1]=='E'))
            path_index=path_index+2;
        path[i]=path[path_index];
        i++;
        path_index++;
    }
    for(path_index=i;path_index<=max_node+1;path_index++)
        path[path_index]='\0';
        path_index=1;
    i=1;
    while(path[path_index]!='\0')
    {
        if((path[i]=='S' && path[i+1]=='N'))
            path_index=path_index+2;
        path[i]=path[path_index];
        i++;
        path_index++;
    }
    for(path_index=i;path_index<=max_node+1;path_index++)
        path[path_index]='\0';

    path_index=1;
}
main()
{
    int i;
    def();
   graph[0][1]='N';
	graph[0][2]='S';
	graph[0][6]='E';
	graph[0][10]='W';
	graph[1][0]='S';
	graph[2][0]='N';
	graph[2][3]='S';
	graph[2][4]='W';
	graph[3][2]='N';
	graph[3][5]='W';
	graph[4][2]='E';
	graph[5][3]='E';
	graph[6][0]='W';
	graph[6][19]='N';
	graph[6][18]='S';
	graph[7][19]='W';
	graph[7][20]='E';
	graph[8][20]='N';
	graph[9][18]='W';
	graph[10][0]='E';
	graph[10][11]='N';
	graph[10][13]='S';
	graph[10][16]='W';
	graph[11][14]='E';
	graph[11][10]='S';
	graph[11][12]='N';
	graph[12][15]='W';
	graph[12][11]='S';
	graph[13][10]='N';
	graph[13][17]='W';
	graph[14][11]='W';
	graph[15][12]='E';
	graph[16][10]='E';
	graph[17][13]='E';
	graph[19][7]='E';
	graph[19][6]='S';
	graph[18][6]='N';
	graph[18][9]='E';
	graph[20][8]='S';
	graph[20][7]='W';
	graph[7][21]='N';
	graph[8][22]='E';
	graph[9][23]='S';
	graph[21][7]='S';
	graph[22][8]='W';
	graph[23][9]='N';
	    path_find(7,14);
    for(i=1;i<=max_node+1;i++)
        printf("%c",path[i]);
}
