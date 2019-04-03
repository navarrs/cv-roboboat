/*
    @date:   Mar 25, 2019
    @author: IngridNavarroA
    @brief:  Point cloud ground segmentation system. 
    @file:   cuco_main.cpp
*/
#include "utils.hpp"
#include "sensor_check.hpp"

#include "includes.h"
#include "groundseg.h"
#include "pointcloud.h"

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <sstream>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>
#include <librealsense2/rs.hpp> 

namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace std;
DIR *dir;
struct dirent *ent;
struct stat sb;

/* Saves point clouds into text files 
 * 		@params 
 *  		pointcloud ( vector<point_XYZL> )
 * 			filepath ( string )	 
 *  	@return void
 */
void pointcloud_to_txt(const vector<point_XYZL> pc, string filepath);

int main(int argc, char * argv[]) try {
    
    // Parse arguments
    po::options_description description("Usage");
    description.add_options()
        ("help", "Program usage.")
        ("outpath", po::value<string>()->default_value("../tests/pointclouds"), "Path to save output point clouds.")
        ("lha",     po::value<int>()->default_value(20),    "Number of seeds to obtain the LHA.") // Lowest height average
        ("thseed",  po::value<float>()->default_value(0.2), "Threshold to consider a point to be a seed.") // 
        ("n",       po::value<int>()->default_value(10),    "Number of pointcloud reads."); // This is just for testing right now
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
    string outpath = opts["outpath"].as<string>();
    int N = opts["n"].as<int>();
    int num_lha = opts["lha"].as<int>();
    float seed_thresh = opts["thseed"].as<float>();

    // Make sure device is connected
    rs2::device dev = sensor_check::get_a_realsense_device();

    // If so, start ground segmentation algorithm
    if (dev) {
        cout << " ------------------------------------------------- " << endl 
             << "|           Ground Segmentation Algorithm         |" << endl
             << " ------------------------------------------------- " << endl
             << "[ CONFIGURATION ] " << endl
             << "  >> Reading point clouds from: " << sensor_check::get_device_name(dev) << endl
             << "  >> Saving annotated files in: " << outpath << endl
             << "  >> Number of seeds to get the LHA: " << num_lha << endl
             << "  >> Seed threshold: " << seed_thresh << endl
             //<< "  >> Num of iterations: " << numIters << endl
             //<< "  >> Num of segments along the x-axis: " << numSegments << endl
             //<< "  >> Distance threshold: " << distThresh << endl << endl
             << "[ START ] " << endl; 
    
        // Create output directory
        int version = 0;
        string new_dir;
        do { 
            new_dir = outpath + boost::lexical_cast<string>(++version);
        } while (stat(new_dir.c_str(), &sb) == 0); // Create new version if directory exists
        //cout << "  >> Creating directory: " << new_dir << endl;
        mkdir(new_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 
    
        // Pointcloud stuff
        rs2::pointcloud pc;
        rs2::points points;
        rs2::pipeline pipe;
        pipe.start();
        string file = "pointcloud_";
        
        while(N) {

            // Get frames and compute pointcloud
            auto frames = pipe.wait_for_frames();
            auto color = frames.get_color_frame();
            pc.map_to(color);

            // Compute depth map 
            auto depth = frames.get_depth_frame();
            points = pc.calculate(depth);

            // Process point clouds
            if (points) {
                vector<point_XYZL> pointcloud;

                // Convert pointcloud from RealSense format to my version
                // TODO: see if I can avoid doing this. 
                convert_pointcloud(points, pointcloud);
                // print_pointcloud(pointcloud, 10);

                // Sort it about the z-axis
                sort_pointcloud(pointcloud, "z");
                // print_pointcloud(pointcloud, 10);

                // Extract initial seeds
                vector<point_XYZL> seeds;
                extract_initial_seeds(pointcloud, seeds, num_lha, seed_thresh);
                cout << " Pointcloud " << pointcloud.size() << " Seeds " << seeds.size() << endl;
                // print_pointcloud(seeds, seeds.size());

                // Save to text file 
                string filepath = new_dir + "/" + file + to_string(N) + ".txt";
                pointcloud_to_txt(pointcloud, filepath);
                N--;
            }
        }
    }
    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    cerr << "RealSense error calling " << e.get_failed_function() 
              << "(" << e.get_failed_args() << "):\n    " 
              << e.what() << endl;
    return EXIT_FAILURE;

} catch (const exception& e) {
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}

// Saves final point cloud to text file 
void pointcloud_to_txt(const vector<point_XYZL> pc, string filepath) {

	ofstream textfile; 	
	textfile.open(filepath.c_str(), fstream::app);
	
	cout << "Processing " << pc.size() << " points" << endl; 
	
	for (int i = 0; i < pc.size(); i++) {
        textfile << pc[i].x << " " 
                 << pc[i].y << " " 
                 << pc[i].z << " " 
                 << pc[i].label << " "
                 << endl;    
	}
	textfile.close();
}
