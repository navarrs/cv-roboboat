#include "pointcloud.h"

// Store pointcloud in struct to be processed by GSA
void convert_pointcloud(rs2::points rs_pc, std::vector<point_XYZL>& gs_pc) {
	
	if (!rs_pc) { // If point cloud is empty, exit
		std::cout << "Empty point cloud. " << std::endl;
		return;
	} 

	auto vertices = rs_pc.get_vertices();
	for (int i = 0; i < rs_pc.size(); i++) {
		if (vertices[i].z) { // If point is not (0, 0, 0)
			gs_pc.push_back({
				vertices[i].x, 
				vertices[i].y,
				vertices[i].z,
				0 // Initially, all labels are 0
			});
		}
	}
}

// Print point cloud coordinates
void print_pointcloud(const std::vector<point_XYZL>& pc, int num) {

	if (!(pc.size() > 0)) { // If point cloud is empty, exit. 
		std::cout<< "Empty point cloud" << std::endl;
		return;
	} else if (pc.size() < num) { // If user specified more points than there are in point cloud. 
		num = pc.size();
		std::cout << "[INFO] Not enough points to print. Printing " << num << " instead. " << std::endl;
	}

	// Print coordinates of each point
	for (int i = 0; i < num; i++) {
		std::cout << "Point [" << i << "] with coordinates (x, y, z, l): (" 
		          << pc[i].x << ", "
		          << pc[i].y << ", "
		          << pc[i].z << ", "
		          << pc[i].label << ") " << std::endl;
	}
	std::cout << std::endl;
}

// Helper methods to sort the vectors 
bool compareY(point_XYZL& p1, point_XYZL& p2) {
	return p1.y < p2.y;	
} 
bool compareZ(point_XYZL& p1, point_XYZL& p2) {
	return p1.z < p2.z;
}
bool compareX(point_XYZL& p1, point_XYZL& p2) {
	return p1.x < p2.x;
}

// Sorts point cloud on specified axis
void sort_pointcloud(std::vector<point_XYZL>& pc, std::string axis) {

	if (!(pc.size() > 0)) {
		std::cout<< "Empty point cloud" << std::endl;
		return;
	}

	if (axis == "z") {
		sort(pc.begin(), pc.end(), compareZ);
	} else if (axis == "y") {
		sort(pc.begin(), pc.end(), compareY);
	} else if (axis == "x") {
		sort(pc.begin(), pc.end(), compareX);
	} else {
		std::cout << "[ERROR] Axis does not exist. " << std::endl;
		return;
	}
}
