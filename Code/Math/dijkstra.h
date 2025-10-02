#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_

#include "main.h"

#define numvexs 97

typedef struct
{
	int matrix[numvexs][numvexs];//�ڽӾ���
	int numVexs;       //������
	int father[200];    //���ڵ�
	int path[200];      //���·�� 
	int route[200];
}Graph;

extern Graph PhEleTechnology_Test;
extern Graph PhEleTechnology;

void CreatGraph(Graph *G);
int dijk(Graph *G,int head,int target);

#endif
