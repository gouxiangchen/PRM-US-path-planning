#ifndef PRM_USS_H
#define PRM_USS_H

#include "map_pgm.h"
#include <opencv2/opencv.hpp>
#include <vector>

struct point 
{
	int x;
	int y;
	point(int x=0,int y=0){this->x=x;this->y=y;}
	bool operator==(const point p)
	{
		return (x==p.x &&y == p.y);
	}
};


struct p_node
{
	point pose;
	std::vector<p_node *> neighbors;
	p_node(point p){this->pose=p;};
};


struct g_node
{
	p_node *  pose;
	int start_cost;
	int goal_cost;
	int cost;
	//std::vector<g_node *> neighbors; 
	g_node * father;
	g_node(p_node * p ,int cost_start,int cost_goal,g_node * fa=NULL)
	{
		this->pose = p;
		if (fa == NULL)
		{
			this->start_cost = 0;
		}
		else
		{
			start_cost =  cost_start;
		}
		goal_cost = cost_goal;
		cost = start_cost + goal_cost;
		this->father = fa;
	}
};



class PRM_map
{
public:
	PRM_map(const char * filename);
	bool getPath(point start,point goal);
	std::vector<p_node * >allnode;
	void showResult(const char * filename);
private:
	const int steplength;
	int sizeX;
	int sizeY;
	bool isCreated;
	int getDistance(const point p1,const point p2);
	double getDistance(int x1,int y1,int x2,int y2);
	std::vector<point> line;
	bool if_line(const point p1,const point p2);
	const PRM_map operator=(const PRM_map &);
	bool collisionCheck(int x,int y);
	std::vector<p_node * > path;
	Pgm_map map;
	int test;
	int ** buff;
//	int ** graph;
//	std::vector <point> output_path;
	cv::Mat img;
};

#endif