#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;
using namespace cv;

#define THRESH 235
#define WINDOW  20

// Computes average of image patch 
int average(Mat patch) {
	int num_elements = patch.rows * patch.cols;
	int sum = 0;
	for (int i = 0; i < patch.rows; i++) {
		for (int j = 0; j < patch.cols; j++) {
				sum += (int)patch.at<uchar>(i, j);
		}
	}
	return (sum == 0 ? 0 : sum / num_elements);
}

// Print graph
void printg(vector<vector<int> > v) {
	for (int i = 0; i < v.size(); i++) {
		for (int j = 0; j < v[i].size(); j++) {
			cout << v[i][j] << " ";
		}
		cout << endl;
	}
}

int main(int argc, char* argv[]) {
	//Parse arguments
	po::options_description description("Usage");
	description.add_options()
		("help", "Program usage.")
		("in_path", po::value<string>()->default_value("../maps/map1.png"), "Path to input file.");
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
	string input_path = opts["in_path"].as<string>();

	// Read input image
	Mat map = imread(input_path, 0);
	if (!map.data) {
		cout << "No image data." << endl;
		return -1;
	}
	resize(map, map, Size(640, 480));
	Rect bounds(0, 0, map.cols, map.rows);
	Mat map_crop = Mat(WINDOW, WINDOW, CV_8UC3, Scalar(0,0,0));

	// Generate graph 
	// int graph[map.rows / WINDOW][map.cols / WINDOW];
	vector<int> g(map.cols / WINDOW, 0);
	vector<vector<int> > graph(map.rows / WINDOW, g);

	int c = 0;
	for (int x = 0; x < map.cols; x += WINDOW) {
		int r = 0;
		for (int y = 0; y < map.rows; y += WINDOW) {
			Rect roi(x, y, WINDOW, WINDOW);
			map_crop = map(roi & bounds);
			int avg = average(map_crop);
			//graph[r++][c] = (avg < THRESH ? 1 : 0);
			graph[r++][c] = (avg < THRESH ? 1 : 0);
		}
		c++;
	}
	// Print graph
	printg(graph);
	return 0;
}