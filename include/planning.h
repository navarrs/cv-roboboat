
#ifndef PLANNING
#define PLANNING

#include <opencv2/opencv.hpp>
#include <functional>
#include <algorithm>
#include <stack> 
#include <iostream>

// Image Settings
#define IM_WIDTH  640 
#define IM_HEIGHT 480
#define IM_WINDOW 10

// Grid size
#define GRID_WIDTH  IM_WIDTH  / IM_WINDOW
#define GRID_HEIGHT IM_HEIGHT / IM_WINDOW

// Threshold to create binary maps
#define THRESH_MIN 225
#define THRESH_MAX 255

// Colors
#define SBLACK cv::Scalar(0,    0,   0)
#define SRED   cv::Scalar(0,    0, 255)
#define VBLUE  cv::Vec3b(255,   0,   0)
#define VWHITE cv::Vec3b(255, 255, 255)

// Common parameters
#define LINE_THICK   1
#define POINT_THICK  3
#define FREE  255
#define RADIUS  2

namespace Planner {

	using uint = unsigned int;

	struct coord {
		int r, c;
		// Overloading operators 
		bool operator == (const coord &co) { return r == co.r && c == co.c; }
		coord operator + (const coord &co) { return {r + co.r, c + co.c}; }
	};

	struct node {
		coord parent;
		uint h;
	};

	using node_list = std::set<node*>;
	using direction_list = std::vector<coord>;
	using heuristic_func = std::function<uint(coord, coord)>;

	class Map {

		public:
			Map();
			int  set_input_map(std::string input_path);
			int  create_obstacle_map(int dilation);
			int  generate_world();
			void print_world();
			void trace_path(std::stack<coord> path);

			cv::Mat get_input_map()    { return input_map;    }
			cv::Mat get_obstacle_map() { return obstacle_map; }
			std::vector<std::vector<int> > get_world() { return world; } 
			void set_world(std::vector<std::vector<int> > w) { world = w; }
			
		private:
			cv::Mat input_map;
			cv::Mat obstacle_map;
			std::vector<std::vector<int> > world;
			std::stack<coord> path;

			void dilate_obstacles(const cv::Mat& dilated_map);
			void draw_obstacle_map_grid();
			int  roi_average(cv::Mat roi);
	};

	class PathGenerator {
		public:
			PathGenerator();

			void set_heuristic(heuristic_func h);
			void set_diagonal_movement(bool enable_full_movement);
			void a_star_search(std::vector<std::vector<int> > graph, std::stack<coord> &path);
			void set_src(coord co) { src = co; }
			void set_dst(coord co) { dst = co; }
			coord get_src() { return src; }
			coord get_dst() { return dst; }

		private:
			heuristic_func heuristic;
			direction_list directions;
			int num_directions;
			coord src, dst;

			bool is_coord_valid(coord co);
			bool is_coord_destination(coord s, coord d);
			bool is_coord_blocked(const std::vector<std::vector<int> > &graph, coord co);
			void compute_path(std::vector<std::vector<node> > &nodes, std::stack<coord> &path);
	};


	class Heuristic {
		public:
			static uint manhattan(coord src, coord dst);
			static uint euclidian(coord src, coord dst);
			static uint octagonal(coord src, coord dst);
		private:
			static coord get_delta(coord src, coord dst);
	};
}

#endif