#include "dijkstra.h"
#include "map.h"
#include "map_message.h"
#include "usmart.h"
#include "main.h"
#include "stdio.h"

#define inf 65535

Graph PhEleTechnology;
Graph PhEleTechnology_Test;


void CreatGraph(Graph *G)
{
	G->numVexs = numvexs;
	
	//��ʼ��
	for(int i = 0;i < G->numVexs;i++)
	{
		G->path[i] =  0;
		for(int j = 0;j < G->numVexs;j++)
		{
			G->matrix[i][j] = inf;
		}
	}
	
	for(int i = 0;i < G->numVexs;i++)
	{
		for(int j = 0;j < ConnectionNum[i];j++)
		{
			G->matrix[i][Node[j+Address[i]].nodenum] = Node[j+Address[i]].step;
		}
	}
}


//headΪԴ��㣬targetΪĿ��ڵ�
int	dijk(Graph *G,int head,int target)
{
	int first = 0;
	int num = 0;
	int dis = 0;
	int distance[numvexs]; //head������ľ���
	int visited[numvexs];  //1Ϊ�ѷ��ʸýڵ㣬0Ϊδ���ʸýڵ�
	
	 //��ʼ������
	for(int i = 0;i < G->numVexs;i++)
	{
		visited[i] = 0;
		distance[i] = G->matrix[head][i]; //����Դ�ڵ������ߵĶ������Ȩֵ
		if(distance[i] != inf  && distance[i] != 1000)
		{
			num++;
		}
	}
	
	visited[head] = 1;
	
	//����һ��ѭ������δѡ���б��У�ѡ��һ����Դ��������Ķ���
	
	for(int a = 0;a < G->numVexs-1;a++)
	{	
		int min = 0;
		int path_Min = inf;
		for(int j = 0;j < G->numVexs;j++)
		{
			if(visited[j] == 0 && distance[j] <= path_Min)
			{ 
          		min = j;
          		path_Min = distance[j];
        	}
		}
		
		visited[min]=1; //��С����ı�ѡ��
		
		if(first < num)
		{
			G->father[min] = head;
			first++;
		}
		
		for(int k = 0;k<G->numVexs;k++)
		{ //���ݱ�ѡ���Ķ������distance����
	        if(visited[k] == 0 && distance[k] > distance[min]+G->matrix[min][k])
			{ //��ΪĿǰ����С�����������仯������ѡ���Ľ�������µ�������û�м��뵽����������Ľڵ�����·��
	            distance[k]=distance[min]+G->matrix[min][k];
	            G->father[k] = min;
	        }
    	}
	}
	
	dis = distance[target];
	
	int Transition[200] = {0};
    Transition[0] = target;
    int f,g;
    for(f = 1;target != head;f++)
    {
    	Transition[f] = G->father[target];
    	target = G->father[target];
	}
	int p = 0;
	for(g = f-1;g >= 0;g--)
	{
		G->route[p++] = Transition[g];
	}
	G->route[p] = 0XFF;
	
//	for(g = 0;g < f;g++)
//	{
//		printf("%d ",G->route[g]);
//	}
//	printf("\r\n");
	return dis;
}
