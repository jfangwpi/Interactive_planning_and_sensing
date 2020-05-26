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
#include "cbta/graph_lift.hpp"

using namespace cv;
using namespace librav;

int main(int argc, char** argv )
{
	Mat input_image;
	/*** 1. Create a empty square grid ***/
	int row_num = 3;
	int col_num = 3;

	std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(row_num,col_num,50);

	/*** 2. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, false, true);

    /*** 4. Construct a lifted graph ***/
	int historyH = 2;
    std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph = GraphLifter::BuildLiftedGraph(historyH, graph);

	waitKey(0);
	return 0;
}