#ifndef Astar_H
#define Astar_H

#include <vector>
#include <cmath>
#include "PRM_USS.h"


/*
// pose struct point 
*/
/*
// 
// struct point 
// {
// 	int x;
// 	int y;
// 	point(int x=0,int y=0){this->x=x;this->y=y;};
// };
// 
// / *
// // struct point node
// * /
// 
// struct p_node
// {
// 	point pose;
// 	std::vector<p_node *> neighbors;
// 	p_node(point p){this->pose=p;};
// };
// 
// / *
// // struct graph node
// * /
// 
// struct g_node
// {
// 	p_node *  pose;
// 	int start_cost;
// 	int goal_cost;
// 	int cost;
// 	//std::vector<g_node *> neighbors; 
// 	g_node * father;
// 	g_node(p_node * p ,int cost_start,int cost_goal,g_node * fa=NULL)
// 	{
// 		this->pose = p;
// 		if (fa == NULL)
// 		{
// 			this->start_cost = 0;
// 		}
// 		else
// 		{
// 			start_cost =  cost_start;
// 		}
// 		goal_cost = cost_goal;
// 		cost = start_cost + goal_cost;
// 		this->father = fa;
// 	}
// };
*/


/*
// Astar class
*/

class Astar
{
public:
	bool getPath(p_node * start,p_node * goal);
	Astar(){};
	std::vector<p_node * > path;
protected:
	std::vector<g_node *> open;
	std::vector<g_node *> closed;
	int getManDistance(p_node * p1,p_node * p2)
	{
		using namespace std;
		return( abs(p1->pose.x - p2->pose.x)+abs(p1->pose.y - p2->pose.y) );
	}
	int getDistance(p_node * p1,p_node *p2)
	{
		using namespace std;
		return (int)sqrt(double((p1->pose.x-p2->pose.x)*(p1->pose.x-p2->pose.x)+(p1->pose.y-p2->pose.y)*(p1->pose.y-p2->pose.y)));
	}
	
};

#endif