#ifndef GROUNDSEG_H
#define GROUNDSEG_H

#include "includes.h"

/* Computes the initial seeds used to estimate the ground plane. First, it
 * calculates the average representative of the ground (LHA) from an arbitrary
 * number of points. Then it uses this value as a threshold to determine if a 
 * point is a seed or not. 
 * 		@params
 *			gs_pc (std::vector<point_XYZL>) - Input Point cloud
 *  		seeds (std::vector<point_XYZL>) - Computed seeds
 *			num_lha (int) 					- Number of seeds needed to compute LHA
 *			seed_thresh (float) 			- Threshold to consider a point a to be a seed
 *		@returns void
 */
void extract_initial_seeds(const std::vector<point_XYZL>& gs_pc, std::vector<point_XYZL>& seeds, int num_lha, float seed_thresh);


#endif