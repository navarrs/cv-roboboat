#ifndef INCLUDES_H
#define INCLUDES_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <librealsense2/rs.hpp> 

struct point_XYZL {
	float x; 
	float y; 
	float z; 
	int label; 
};

#endif
