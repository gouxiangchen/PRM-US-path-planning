#include "PRM_USS.h"
#include <time.h>
#include <iostream>


int main()
{
	PRM_map map("intel_binary.jpg");
	//map.getPath(point(398,358),point(266,802));
	map.getPath(point(652,1158),point(498,312));
	map.showResult("3.txt");
	return 0;
}