
#include "planning.h"

using namespace Planner;

/*************************************************
				MAP GENERATOR METHODS
 *************************************************/
Map::Map() {
	std::vector<int> w_width(GRID_WIDTH, 0);
	std::vector<std::vector<int> > temp_world(GRID_HEIGHT, w_width);
	world = temp_world;
}

/* @method: sets_input_map(...)
   @brief:  sets input map to file in given path. 
   @param:
		input_path (std::string)
   @returns:
		1 if succesfull, 0 otherwise */
int Map::set_input_map(std::string input_path) {
	input_map = cv::imread(input_path, CV_LOAD_IMAGE_COLOR);
	if (!input_map.data) {
		std::cout << "No image data. " << std::endl;
		return 0;
	}
	cv::resize(input_map, input_map, cv::Size(IM_WIDTH, IM_HEIGHT));
	return 1;
}

/*  @method: draw_grid(...)
	@brief: Draws a grid on input map based on user-specified window size.
	@params: 
		map (&Mat)    input map. */
void Map::dilate_obstacles(const cv::Mat& dilated_map) {
	cv::Mat temp_obstacle_map(input_map.size(), CV_8UC3);
	for (int i = 0; i < dilated_map.rows; i++) {
		for (int j = 0; j < dilated_map.cols; j++) {
			uchar color = dilated_map.at<uchar>(i, j);
			temp_obstacle_map.at<cv::Vec3b>(cv::Point(j, i)) = color == THRESH_MAX ? VBLUE : VWHITE;
		}
	}
	obstacle_map = temp_obstacle_map;
}

/*  @method: draw_grid(...)
	@brief: Draws a grid on input map based on user-specified window size. */
void Map::draw_obstacle_map_grid() {
	int i = 0;

	// Draw vertical lines
	for (i = 0; i < obstacle_map.cols; i += IM_WINDOW) {
		cv::line(obstacle_map, cv::Point(i, 0), cv::Point(i, obstacle_map.rows-1), SBLACK, LINE_THICK);
	}
	// Draw horizontal lines
	for (i = 0; i < obstacle_map.rows; i += IM_WINDOW) {
	 	cv::line(obstacle_map, cv::Point(0, i), cv::Point(obstacle_map.cols-1, i), SBLACK, LINE_THICK);
	}
}

/* @method: create_obstacle_map(...)
   @brief:  returns content of input_map (Mat)
   @param:  
		dilation size (int)
	@returns:
		1 if succesfull, 0 otherwise */
int Map::create_obstacle_map(int dilation) {
	if (!input_map.data) {
		return 0;
	}

	// Convert input map to grayscale
	cv::Mat gray_map(input_map.size(), CV_8UC1);
	cv::cvtColor(input_map, gray_map, CV_BGR2GRAY);

	// Binarize grayscale map 
	cv::Mat binary_map(gray_map.size(), CV_8UC1);
	cv::threshold(gray_map, binary_map, THRESH_MIN, THRESH_MAX, CV_THRESH_BINARY_INV);

	// Dilate binarized map
	cv::Mat dilated_map(binary_map.size(), CV_8UC1); 
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*dilation+1, 2*dilation+1), cv::Point(dilation, dilation));
	cv::dilate(binary_map, dilated_map, element, cv::Point(-1, -1), 3);
	
	// Create obstacle map
	dilate_obstacles(dilated_map);

	return 1;
}

/*	@method: average(...)
	@brief: Computes the (integer) average of an image roi.
	@param:
		roi (Mat)  Image roi
	@returns
		avg (int)   Average of image roi. 
*/
int Map::roi_average(cv::Mat roi) {
	int num_elements = roi.rows * roi.cols;
	int sum = 0;
	for (int i = 0; i < roi.rows; i++) {
		for (int j = 0; j < roi.cols; j++) {
				sum += (int)roi.at<uchar>(i, j);
		}
	}
	return (sum == 0 ? 0 : sum / num_elements);
}

/*	@method: generate_world(...)
	@brief:  Creates obstacle graph to be used by the 
			 path planning algorithm. 
	@returns:
		1 if succesfull, 0 otherwise*/
int Map::generate_world() {

	if (!obstacle_map.data) {
		return 0;
	}

	cv::Rect bounds(0, 0, obstacle_map.cols, obstacle_map.rows);
	cv::Mat map_crop = cv::Mat(IM_WINDOW, IM_WINDOW, CV_8UC3, cv::Scalar(0,0,0));
	for (int x = 0; x < obstacle_map.cols; x += IM_WINDOW) {
		for (int y = 0; y < obstacle_map.rows; y += IM_WINDOW) {
			cv::Rect roi(x, y, IM_WINDOW, IM_WINDOW);
			map_crop = obstacle_map(roi & bounds);
			int avg = roi_average(map_crop);
			world[y / IM_WINDOW][x / IM_WINDOW] = (avg < FREE ?  1 : 0);
		}
	}
	
	// Draw grid on obstacle map
	draw_obstacle_map_grid();

	return 1;
}

/*	@method: print_world(...)
	@brief:  returns obstacle world needed for path finding. */
void Map::print_world() {
	for (int i = 0; i < world.size(); i++) {
		for (int j = 0; j < world[i].size(); j++) {
			std::cout << world[i][j] << " ";
		}
		std::cout << std::endl;
	}
}

/* @method: trace_path(...)
   @brief: Takes path computed by astar and traces it on 
   		   the world map. 
*/
//C++: void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
void Map::trace_path(std::stack<coord> path) {
	coord prev = {INT_MAX, INT_MAX};
	while(!path.empty()) {
		coord p = path.top();
		if (!(prev.c == INT_MAX)) {
			cv::circle(input_map, cv::Point(p.c * IM_WINDOW, p.r * IM_WINDOW), RADIUS, SRED, POINT_THICK);
		}
		path.pop();
		std::cout << "->("<< p.r <<","<<p.c <<")";
		world[p.r][p.c] = 7;
		prev = p;
	}
	std::cout << std::endl;
}