// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <cmath>
#include <tuple>
#include <string>
#include <fstream>


// opencv
#include "opencv2/opencv.hpp"

// user
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"
#include "vis/graph_vis_lcm.hpp"
#include "cbba/cbba_agent.hpp"
#include "cbba/cbba_task.hpp"


using namespace librav;
using namespace cv;

int main(int argc, char** argv )
{   
	// Dimension of grid 
	int num_row = 10;
    int num_col = 10;

    int NumPerCase = 100;

    // Number of sensors
    double sensor_dsty = 0.03;
    // Number of agents
    int num_agents_ = 12;
    // Number of tasks
    int num_tasks_inde_ = 10;
    int num_Tasks_de = 0;
    // Obstacle percentage
    double Obstacle_density = 0.4;
    
    int num_sensors = (int)num_col * num_row * sensor_dsty;
    int num_obstacles = (int)num_col * num_row * Obstacle_density;

    //====================================== Random Occupancy Grid Map ==========================================//
    // Initialize the parameters
    bool flag_valid = false;
    std::vector<Agent> agents;
    std::shared_ptr<SquareGrid> true_grid;
    std::shared_ptr<Graph_t<SquareCell*>> true_graph;
    TasksList tasks;

    /********************************* Generate True Occupancy Grid Map ***********************************/
    while (flag_valid != true){
        // CBBA with full knowledge of map
        true_grid = GridGraph::CreateSquareGrid(num_row,num_col,1);
        // Generate ramdom occupancy grid map
        true_grid->SetRandomOccupanyMap(Obstacle_density);

        // Determine the tasks position randomly 
        tasks = true_grid->SetRandomTasks(num_tasks_inde_, num_Tasks_de);
    
        // Determine the position of agents randomly 
        agents = true_grid->SetRandomAgents(num_agents_,num_tasks_inde_,num_Tasks_de);
        
        // Generate the graph corresponding to the information shown above
        true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid, false, false);
        
        // Check whether the occupancy grid map is valid
        flag_valid = true_grid->OccupancyGridMapValidity(agents, tasks, true_graph);
        std::cout << "The flag is " << flag_valid << std::endl; 
    }

    std::cout << "Test edge of graph" << std::endl;
    auto edges = true_graph->GetAllEdges();

    for(auto&ee: edges){
        std::cout << ee->src_->state_->id_ << " -> " << ee->dst_->state_->id_ << std::endl; 
    }


    /*** Plot the true occupancy grid map ***/
    std::vector<Path_t<SquareCell*>> path_vis;
    for(int i=0; i < num_agents_; i++){
        int64_t init_pos = agents[i].init_pos_;
        auto agent_cell = true_graph->GetVertexFromID(init_pos);
        Path_t<SquareCell*> path = {agent_cell->state_};
        path_vis.push_back(path);
    }
    GraphVis vis;
    Mat vis_img;
    vis.VisSquareGrid(*true_grid, vis_img);
    vis.VisSquareGridGraph(*true_graph, vis_img, vis_img, true);
    for (int i = 0; i < num_agents_; i++){
        vis.VisSquareGridPath(path_vis[i], vis_img, vis_img, i);
    }

    imwrite("test_availability_map.jpg",vis_img);


    return 0;
}