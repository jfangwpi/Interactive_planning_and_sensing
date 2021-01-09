/*
 * test_cbba.cpp
 *
 *  Created on: July 25, 2019
 *      Author: jfang
 */

/*** This code is only for independent tasks ***/
// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
// opencv
#include "opencv2/opencv.hpp"

// self-defined library
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"

#include "auto_vehicle/auto_vehicle.hpp"
#include "vehicle/auto_team.hpp"
#include "vehicle/agent.hpp"
#include "task_assignment/cbba_impl.hpp"

using namespace librav;
using namespace cv;


int main(int argc, char** argv )
{   
    //===============================================================================================//
    //===========================================Task - Agent========================================//
    //===============================================================================================//
    // Parameters
    int64_t num_row = 30;
    int64_t num_col = 30;

    // Construct the TasksSet
    TasksSet tasks_ = IPASMeasurement::ConstructLTLTasks();
    // Construct auto vehicle team
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team_ = IPASMeasurement::ConstructAutoTeam();

    //===============================================================================================//
    //============================================= MAP =============================================//
    //===============================================================================================//
    std::shared_ptr<SquareGrid> true_grid = GridGraph::CreateSquareGrid();
    std::shared_ptr<Graph_t<SquareCell *>> true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false, false);
    //===============================================================================================//
    //============================================= CBBA ============================================//
    //===============================================================================================//      
    for(auto&agent: vehicle_team_->auto_team_){
        agent->vehicle_.SetLocalMap(true_grid);
    } // Initialize the local grid map for each auto vehicle

    CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);
    std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);
    Path_t<SquareCell*> path_origin[4];
    for (int i = 0; i < 4; i++){
        path_origin[i] = path_ltl_[i];
        std::cout << "Path: " ;
        for (auto e: path_origin[i]){
            std::cout << e->id_<< ", ";
        }
        std::cout << std::endl;
    }

    // Visualization
	// Image Layouts: square grid -> graph -> path
	GraphVis vis;
	Mat vis_img;
	vis.VisSquareGrid(*true_grid, vis_img);
	vis.VisSquareGridGraph(*true_graph, vis_img, vis_img, true);
	// // put the path on top of the graph
	for (int i = 0; i < 4; i++){
		vis.VisSquareGridPath(path_origin[i], vis_img, vis_img, i);
	}
	
	// display visualization result
	//namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	//imshow("Processed Image", vis_img);
	imwrite("result_cbba.jpg",vis_img);

	waitKey(0);


  
 

	return 0;
}