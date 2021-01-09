// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <string>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "vis/graph_vis.hpp"
#include "map/square_grid.hpp"
#include "graph/algorithms/astar.hpp"



using namespace cv;
using namespace librav;

int main(int argc, char** argv )
{

	/*** 1. Read from config file: map.ini ***/
	int64_t num_row = 4;
    int64_t num_col = 4;
	std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(num_row, num_col, 1);


	/*** 2. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, false, true);


	/*** Visualize the map and graph ***/
	Mat vis_img;

	GraphVis::VisSquareGrid(*grid, vis_img);
	GraphVis::VisSquareGridGraph(*graph, vis_img, vis_img, true);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);
	return 0;
}