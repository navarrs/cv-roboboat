#include "groundseg.h"

// Extract initial seeds to estimate ground plane
void extract_initial_seeds(const std::vector<point_XYZL>& gs_pc, std::vector<point_XYZL>& seeds, int num_lha, float seed_thresh) {

	// Check num_lha is in order
	if (num_lha < 0) {
		std::cout<< "[ERROR] Number of points for LHA cannot be <0." << std::endl;
		return;
	// AVOID this, num_lha should be a small number. Number of points in pointcloud is usually a high number ~100k
	} else if (num_lha > gs_pc.size()) { 
		std::cout << "Number of points for LHA exceeds number of points int point cloud." << std::endl;
		std::cout << "It is recommended that LHA is a relatively small number, i.e. <=100." << std::endl;
		num_lha = gs_pc.size();  
	}

	// Compute lowest height average (LHA)
	float LHA, sum = 0.0;
	for (int i = 0; i < num_lha; i++) {
		sum += gs_pc[i].z;
	}
	LHA = num_lha != 0 ? sum / num_lha : 0.0;

	// Determine seeds based on LHA
	for (int i = 0; i < gs_pc.size(); i++){
		if (gs_pc[i].z < LHA + seed_thresh) {
			seeds.push_back(gs_pc[i]);
		}
	}
}