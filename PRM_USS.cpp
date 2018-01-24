#include "PRM_USS.h"
#include <stdint.h>
#include "Astar.h"
#include <fstream>


bool PRM_map::if_line(const point a,const point b)
{
	int ** walkability = buff;
	line.clear();
	int dx=abs(a.x-b.x);
	int dy=abs(a.y-b.y);
	double k;
	if (dx != 0)
	{
		k=double(b.y-a.y)/double(b.x-a.x);
	}
	else if (dy ==0)
	{
		k=1;
	}
	else
	{
		k=99999;
	}

	point temp;

	if (dx>=dy)
	{
		if (a.x<b.x)
		{
			temp =a;
		}
		else 
			temp=b;
		for (int i=1;i<dx+1;i++)
		{
			int x=temp.x+i;
			int y=temp.y+k*i;
			point pt(x,y);
			line.push_back(pt);
		}
	}
	else
	{
		if (a.y<b.y)
		{
			temp =a;
		}
		else 
			temp=b;
		for (int i=1;i<dy+1;i++)
		{
			int x=temp.x+i/k;
			int y=temp.y+i;

			point pt(x,y);
			line.push_back(pt);
		}
	}
	int len=line.size();
	for (int i=0;i<len;i++)
	{
		if (1 == walkability[line[i].x][line[i].y])
		{
			return false;
		}
	}
	return true;
}

bool PRM_map::collisionCheck(int x,int y)
{
	using namespace cv;
	bool ok=false;
	int xm=sizeX/steplength +1;
	int ym=sizeY/steplength;
	if ( !(x<sizeX && x>=0 && y<sizeY && y>=0 ) )
	{
		return false;
	}

	if (buff[x][y] == 1)
	{
		p_node *p=new p_node(point(x,y));
		allnode.push_back(p);
		return false;
	}
	
	p_node *p=new p_node(point(x,y));
	allnode.push_back(p);
	if (x-steplength>=0 && y-steplength >=0)
	{
		point p1(x,y);
		point p2(x-steplength,y-steplength);
		if (if_line(p1,p2))
		{
			int i=(y-steplength) / steplength * xm + (x-steplength) /steplength ;
/*
// 			if ( ! (p2 == allnode[i]->pose))
// 			{
// 				test++;
// 				std::cout<<p2.x<<","<<p2.y<<" and "<<allnode[i]->pose.x<<","<<allnode[i]->pose.y<<std::endl;
// 			}
*/
			p->neighbors.push_back(allnode[i]);
			allnode[i]->neighbors.push_back(p);
			circle( img,Point(p1.x,p1.y), 1 ,  Scalar(255,10,0), 1, 1, 0 );
			circle( img,Point(p2.x,p2.y), 1 ,  Scalar(255,10,0), 1, 1, 0 );

			for (int i=0;i<line.size();i++)
			{
				circle( img,Point(line[i].x,line[i].y), 0.1,  Scalar(0,10,255), 0.1, 1, 0 );
			}
		}
		ok=true;
	}
	if (x-steplength >=0)
	{
		point p1(x,y);
		point p2(x-steplength,y);
		if (if_line(p1,p2))
		{
			int i=(y) / steplength * xm + (x-steplength) /steplength ;
/*
// 			if ( ! (p2 == allnode[i]->pose))
// 			{
// 				test++;
// 				std::cout<<p2.x<<","<<p2.y<<" and "<<allnode[i]->pose.x<<","<<allnode[i]->pose.y<<std::endl;
// 			}
*/
			p->neighbors.push_back(allnode[i]);
			allnode[i]->neighbors.push_back(p);
			circle( img,Point(p1.x,p1.y), 1 ,  Scalar(255,10,0), 1, 1, 0 );
			circle( img,Point(p2.x,p2.y), 1 ,  Scalar(255,10,0), 1, 1, 0 );
			for (int i=0;i<line.size();i++)
			{
				circle( img,Point(line[i].x,line[i].y), 0.1,  Scalar(0,10,255), 0.1, 1, 0 );
			}
		}
		ok=true;
	}
	if (y-steplength >=0)
	{
		point p1(x,y);
		point p2(x,y-steplength);
		if (if_line(p1,p2))
		{
			int i=(y-steplength) / steplength * xm + (x) /steplength ;
/*
// 			if ( ! (p2 == allnode[i]->pose))
// 			{
// 				test++;
// 				std::cout<<p2.x<<","<<p2.y<<" and "<<allnode[i]->pose.x<<","<<allnode[i]->pose.y<<std::endl;
// 			}
*/
			p->neighbors.push_back(allnode[i]);
			allnode[i]->neighbors.push_back(p);
			circle( img,Point(p1.x,p1.y), 1 ,  Scalar(255,10,0), 1, 1, 0 );
			circle( img,Point(p2.x,p2.y), 1 ,  Scalar(255,10,0), 1, 1, 0 );
			for (int i=0;i<line.size();i++)
			{
				circle( img,Point(line[i].x,line[i].y), 0.1,  Scalar(0,10,255), 0.1, 1, 0 );
			}
		}
		ok=true;
	}
	if (x+steplength < sizeX && y-steplength >=0)
	{
		point p1(x,y);
		point p2(x+steplength,y-steplength);
		if (if_line(p1,p2))
		{
			int i=(y-steplength) / steplength * xm + (x+steplength) /steplength ;
/*
// 			if ( ! (p2 == allnode[i]->pose))
// 			{
// 				test++;
// 				std::cout<<p2.x<<","<<p2.y<<" and "<<allnode[i]->pose.x<<","<<allnode[i]->pose.y<<std::endl;
// 			}
*/
			p->neighbors.push_back(allnode[i]);
			allnode[i]->neighbors.push_back(p);
			circle( img,Point(p1.x,p1.y), 1 ,  Scalar(255,10,0), 1, 1, 0 );
			circle( img,Point(p2.x,p2.y), 1 ,  Scalar(255,10,0), 1, 1, 0 );
			for (int i=0;i<line.size();i++)
			{
				circle( img,Point(line[i].x,line[i].y), 0.1,  Scalar(0,10,255), 0.1, 1, 0 );
			}
		}
		ok=true;
	}
	return ok;
}

int PRM_map::getDistance(const point p1,const point p2)
{
	return (int)sqrt(double((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)));
}

double PRM_map::getDistance(int x1,int y1,int x2,int y2)
{
	return (double)sqrt(double((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
}

bool PRM_map::getPath( point start_, point goal_ ) 
{
	using namespace cv;
	Astar pathfinder;
	p_node * s=NULL;
	p_node * g=NULL;
	int dis=99999;
	for (int i=0;i<allnode.size();i++)
	{
		int d = getDistance(allnode[i]->pose,goal_);
		if (dis > d	)
		{
			dis = d;
			g=allnode[i];
		}
	}
	dis=99999;
	for (int i=0;i<allnode.size();i++)
	{
		int d = getDistance(allnode[i]->pose,start_);
		if (dis > d	)
		{
			dis = d;
			s=allnode[i];
		}
	}

	//std::cout<<test<<std::endl;

	if (  (g==NULL) || !(if_line(g->pose,goal_))  )
	{
		std::cout<<"path find failed!"<<std::endl;
		return false;
	}
	//circle( img,Point(goal_.x,goal_.y), 1,  Scalar(0,0,0), 3, 1, 0 );
	for (int i=0;i<line.size();i++)
	{
		//circle( img,Point(line[i].x,line[i].y), 1,  Scalar(255,10,255), 3, 1, 0 );
	}
	//cv::line(img,Point(line[0].x,line[0].y),Point(line[line.size()-1].x,line[line.size()-1].y),Scalar(255,0,0),2);
	if (  (s==NULL) || !(if_line(s->pose,start_))  )
	{
		std::cout<<"path find failed!"<<std::endl;
		return false;
	}
	//circle( img,Point(start_.x,start_.y), 1,  Scalar(0,0,0), 3, 1, 0 );
	for (int i=0;i<line.size();i++)
	{
		//circle( img,Point(line[i].x,line[i].y), 1,  Scalar(255,10,255), 3, 1, 0 );
	}
	//cv::line(img,Point(line[0].x,line[0].y),Point(line[line.size()-1].x,line[line.size()-1].y),Scalar(255,0,0),2);
	pathfinder.getPath(s,g);
	if (pathfinder.path.size() == 0)
	{
		std::cout<<"path find failed!"<<std::endl;
		return false;
	}
	else
	{
		//circle( img,Point(pathfinder.path[0]->pose.x,pathfinder.path[0]->pose.y), 3,  Scalar(0,0,0), 3, 1, 0 );
		//circle( img,Point(pathfinder.path[pathfinder.path.size()-1]->pose.x,pathfinder.path[pathfinder.path.size()-1]->pose.y), 3,  Scalar(0,0,0), 3, 1, 0 );
		double distan = 0.0;
		for (int i=0;i<pathfinder.path.size()-1;i++)
		{
			path.push_back(pathfinder.path[i]);
			cv::line(img,Point(pathfinder.path[i]->pose.x,pathfinder.path[i]->pose.y),Point(pathfinder.path[i+1]->pose.x,pathfinder.path[i+1]->pose.y),Scalar(255,0,0),2);
			distan += getDistance(pathfinder.path[i]->pose.x,pathfinder.path[i]->pose.y,pathfinder.path[i+1]->pose.x,pathfinder.path[i+1]->pose.y);
		}
		std::cout << " total distance : " << distan << std::endl;
		return true;
	}
}



void PRM_map::showResult(const char * filename)
{
	cv::namedWindow("map");
	cv::imshow("map",img);
	cv::imwrite("map.jpg",img);
	cv::waitKey();

	using namespace cv;
	using namespace std;
	//	namedWindow("map");
	//imshow("map",img);
	//imwrite("map_RSS.jpg",img);

	ofstream out;

	out.open(filename,ios::out);
	if (out.is_open())
	{
		//char a;
		//in>>a;
		for (int i=0;i<path.size();i++)
		{
			out<<path[i]->pose.x<<" "<<path[i]->pose.y<<"\n";
		}


		out.close();
	}

}


PRM_map::PRM_map(const char * filename):map(filename),steplength(20)	//steplength 撒点的分度
{
	img=cv::imread(filename);
	test=0;
	if (isCreated = map.transformMapToGrid())
	{
		sizeX=map.getSizeX();
		sizeY=map.getSizeY();
		if (sizeX % steplength == 0 && sizeX > 0)
		{
			sizeX --;
		}
		if (sizeY %steplength ==0 && sizeY >0)
		{
			sizeY --;
		}
/*		graph = new int *[sizeX / steplength +1 ];
		for (int i=0;i<sizeX  / steplength +1;i++)
		{
			graph[i]=new int [sizeY	/ steplength +1 ];
		}
		*/
		buff=map.getGridMap2D();
		int xt=0;
		int yt=0;
		
		while( (yt  ) < sizeY)
		{
			
			xt=0;
			
			while((xt ) < sizeX)
			{
				//graph[xt / steplength][yt / steplength]=buff[xt][yt];
				if (collisionCheck(xt,yt) )
				{
					//graph[xt / steplength][yt / steplength] = 0;
				}
				else
				{

				}
					//graph[xt / steplength][yt / steplength] = 1;
				xt+= steplength;
			}
			yt+= steplength;
		}


	}
	
}