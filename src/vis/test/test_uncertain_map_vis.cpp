// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <cmath>
#include <tuple>
#include <string>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "vis/graph_vis.hpp"
#include "map/square_grid.hpp"
#include "cbba/cbba_agent.hpp"
#include "cbba/cbba_task.hpp"

// For lcm message
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/uncertain_map_lcm_msgs.hpp"

using namespace cv;
using namespace librav;


int main(int argc, char** argv )
{
    srand(time(NULL));
    lcm::LCM lcm;
	/*** 1. Read from config file: map.ini ***/
    int64_t num_row = 15;
    int64_t num_col = 15;
	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid(num_row, num_col, 1);

	for (int i = 0; i < num_row * num_col; i++){
		grid->SetCellOccupancy(i,OccupancyType::UNKNOWN);
	}
	grid->SetInterestedRegionLabel(45,2);

	/*** 2. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, true);
    
	// Send graph info for visualization
	std::vector<Vertex_t<SquareCell*, double>*> v_list = graph->GetAllVertex();
	vis_data::graph_data graph_data_;
	graph_data_.num_row = num_row;
	graph_data_.num_col = num_col;
	graph_data_.num_cells = v_list.size();
	for (auto& v: v_list){
		vis_data::vertex new_vertex;
		new_vertex.idx = v->state_->id_;
		new_vertex.pos[0] = v->state_->coordinate_.x;
		new_vertex.pos[1] = v->state_->coordinate_.y;
		new_vertex.info_gain = 0;

		switch(v->state_->occu_){
			case OccupancyType::FREE: new_vertex.occupancy = "FREE"; break;
			case OccupancyType::INTERESTED: new_vertex.occupancy = "INTERESTED"; break;
			case OccupancyType::OCCUPIED: new_vertex.occupancy = "OCCUPIED"; break;
			case OccupancyType::UNKNOWN: new_vertex.occupancy = "UNKNOWN"; break;
		}

		graph_data_.cells.push_back(new_vertex);
	}
	lcm.publish("GraphData", &graph_data_);


	std::vector<Agent> agents_group = TaskAssignment::InitializeAgents();
	// Transfer agents info
	vis_data::agents_data agents_data_;
	agents_data_.num_agents = agents_group.size();
	for (auto&agent: agents_group){
		vis_data::agent ag;
		ag.idx = agent.idx_;
		ag.init_pos = agent.init_pos_;
		agents_data_.agents.push_back(ag);
	}
	lcm.publish("AgentData", &agents_data_);


	

	
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