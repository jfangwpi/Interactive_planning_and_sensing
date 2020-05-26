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
	int num_row = 50;
    int num_col = 50;

    int NumPerCase = 20;

    // Number of sensors
    std::vector<double> sensors_density = {0.01,0.02,0.03,0.04,0.05,0.1,0.2,0.3,0.4,0.5};
    // std::vector<double> sensors_density = {0.5};
    // Number of agents
    // std::vector<int> Agents_density = {2,4,8,10,12,14};
    std::vector<int> Agents_density = {4};
    // Number of tasks
    //std::vector<int> Tasks_density = {8,10,12,14,16,18};
    std::vector<int> Tasks_density = {10};
    int num_Tasks_de = 0;
    // Obstacle percentage
    double Obstacle_density = 0.4;
    
    int64_t caseT = 1;
    std::ofstream file("/home/jfang/Workspace/GP_map/json/ipas-data.json");
    Json::Value root;
    for(auto& sensor_dsty: sensors_density){
        for(auto& num_agents_: Agents_density){
            for(auto& num_tasks_inde_: Tasks_density){
                for(int num_case_ = 0; num_case_ < NumPerCase; num_case_++){
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
                    std::string filename_true = "true_occupancy_grid_map_case_" + std::to_string(caseT) + ".jpg";
                    imwrite(filename_true,vis_img);


                    /********************************* Generate Occupancy Grid Map ***********************************/
                    std::shared_ptr<SquareGrid> grid_ipas = GridGraph::CreateSquareGrid(num_row, num_col, 1);
                    std::shared_ptr<SquareGrid> grid_standard = GridGraph::CreateSquareGrid(num_row, num_col, 1);
                    for (int i = 0; i < num_row * num_col; i++){
                        grid_ipas->SetCellProbability(i,0.5);
                        grid_standard->SetCellProbability(i,0.5);
                    }

                    // Set Task for grid
                    for(int ti = 0; ti < tasks.tasks_.size(); ti++){
                        grid_ipas->SetInterestedRegionLabel(tasks.tasks_[ti].pos_.front(),ti+2);
                        grid_standard->SetInterestedRegionLabel(tasks.tasks_[ti].pos_.front(),ti+2);
                    }
                    // Set Agent for grid
                    for(auto& agent:agents){
                        grid_ipas->SetCellOccupancy(agent.init_pos_, OccupancyType::FREE);
                        grid_standard->SetCellOccupancy(agent.init_pos_, OccupancyType::FREE);
                    }

                    /*** 2. Construct a graph from the uncertain square grid ***/
                    std::shared_ptr<Graph_t<SquareCell*>> graph_ipas = GridGraph::BuildGraphFromSquareGrid(grid_ipas, false, true);
                    std::shared_ptr<Graph_t<SquareCell*>> graph_standard = GridGraph::BuildGraphFromSquareGrid(grid_standard, false, true);

                    //====================================== IPAS =======================================// 
                    IGdata ipas_data = GridGraph::IPAS(agents,tasks,num_sensors,grid_ipas,graph_ipas,true_graph);
                    Json::Value root = GridGraph::WriteResultToJson("IPAS",caseT,num_row,num_col,num_agents_,num_tasks_inde_,num_obstacles,num_sensors,ipas_data);
                    file << root; 

                    std::cout << "IPAS COMPLETED" << std::endl;
                    //====================================== Standard Information Gain =======================================//
                    IGdata standardig_data = GridGraph::StandardIG(agents,tasks,num_sensors,grid_standard,graph_standard,true_graph);
                    Json::Value root_standard = GridGraph::WriteResultToJson("Standard IG",caseT,num_row,num_col,num_agents_,num_tasks_inde_,num_obstacles,num_sensors,standardig_data);
                    file << root_standard;
                    std::cout << "STANDARD IG COMPLEDTED" << std::endl;
                    caseT++;
                    // break;
                }
                // break;
            }
            // break;
        }
        break;
    }
    
    return 0;
}