#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "includes.h"

/* Converts point clouds from rs2::points format into a cloud vector
 * needed to perform the Ground Segmentation Algorithm.   
 *      @params 
 *          rs_pc (rs2::points) 			 - Point cloud from RealSense camera
 *   		gs_pc (std::vector<point_XYZL>)  - Converted point cloud
 *      @return void
 */
void convert_pointcloud(rs2::points rs_pc, std::vector<point_XYZL>& gs_pc);

/* Prints point cloud coordinates
 *  @params 
 * 		gs_pc (std::vector<point_XYZL>)  - Input point cloud
 *      num (int)  						 - Number of points to print
 *  @return void
 */
void print_pointcloud(const std::vector<point_XYZL>& gs_pc, int num);


/* Sorts point cloud based on specified axis, i.e. x, y z
 *  @params 
 * 		gs_pc (std::vector<point_XYZL>)  - Input point cloud
 *      axis (std::string) 				 - Sorting axis
 *  @return void
 */
void sort_pointcloud(std::vector<point_XYZL>& gs_pc, std::string axis);

#endif