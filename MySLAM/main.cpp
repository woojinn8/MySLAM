#include <iostream>
#include "VirtualWorld2D.h"


int main(int argc, char** argv)
{
	// Create Virtual world for SLAM
		// 1. create map and landmarks from image
		// 2. load waypoint from txt
	VirtualWorld2D world2d;
	
	world2d.Initialize(argv[1], argv[2]);
	world2d.run();
	

	return 0;
}