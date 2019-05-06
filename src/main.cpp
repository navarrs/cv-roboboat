#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>

#include "planning.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;
using namespace cv;

#define DEBUG 0

int main(int argc, char* argv[]) {
	//Parse arguments
	po::options_description description("Usage");
	description.add_options()
		("help", "Program usage.")
		("inpath", po::value<string>()->default_value("../maps/map1.png"), "Path to input file.")
		("dilation", po::value<int>()->default_value(10), "Size of dilation kernel. ");
	po::variables_map opts;
	po::store(po::command_line_parser(argc, argv).options(description).run(), opts);
	try {
		po::notify(opts);
	} catch (exception& e) {
		cerr << "Error: " << e.what() << endl;
		return 1;
	}
	if (opts.count("help")) {
		cout << description;
		return 1;
	}
	string input_path = opts["inpath"].as<string>();
	int dilation = opts["dilation"].as<int>();

	// Set map 
	cout << "[INFO] Generating world from input map..." << endl;
	Planner::Map map;
	map.set_input_map(input_path);
	map.create_obstacle_map(dilation);
	map.generate_world();
	if (DEBUG) {
		Mat input_map = map.get_input_map();
		imshow("Input Map", input_map);
		Mat obstacle_map = map.get_obstacle_map();
		imshow("Obstacle Map", obstacle_map);
		map.print_world();
		waitKey(0);
	}
	cout << "[DONE]" << endl;

	// Set source and destination 
	cout << "[INFO] Setting source and destination..." << endl;
	Planner::PathGenerator astar;
	astar.set_src({40, 44});
	astar.set_dst({4,  20});
	cout << "[DONE]" << endl;	

	// Path planning 
	cout << "[INFO] Finding path using AStar algorithm..." << endl;
	std::stack<Planner::coord> path;
	astar.set_heuristic(Planner::Heuristic::euclidian);
	astar.set_diagonal_movement(true); // set to false if using Heuristic::manhattan
	astar.a_star_search(map.get_world(), path);

	if (path.empty()) 
		cout << "No path was found" << endl;
	else {
		map.trace_path(path);
		map.print_world();
		Mat final_map = map.get_input_map();
		imshow("Final Map", final_map);
		waitKey(0);
	}
	cout << "[DONE] " << endl;

	return 0;
}

