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
	int num_row = 30;
    int num_col = 30;

    // Number of sensors
    int num_sensors = 5;
    // Number of dependent tasks
    int num_Tasks_de = 0;

    std::ofstream file("/home/jfang/Workspace/uncertain_map/json/threshold_analysis.json");
    Json::Value root;
    
    // Initialize the parameters
    bool flag_valid = false;
    std::vector<double> thresholds;
    double thd = 0.2655;
    while (fabs(thd) > 1e-3){
        // double thd = - prob * log10(prob) - (1.0-prob)*log10(1.0-prob);
        thresholds.push_back(thd);
        thd = thd - 0.0015;
        std::cout << "threshold is " << thd << std::endl;
        // prob = prob + 0.01;
    }
    
    /********************************* Generate True Occupancy Grid Map ***********************************/
    for (auto thred: thresholds){
        std::cout << "The threshold is " << thred << std::endl;
        root["threshold"] = thred;
        std::shared_ptr<SquareGrid> true_grid = GridGraph::CreateSquareGrid("true_map");

        // Define LTL specification
        LTLFormula Global_LTL;
        for (int i = 0; i < Global_LTL.task_info.size();i++){
            true_grid->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);	
        }
        // Decompose the global LTL_expression
        LTLDecomposition::GlobalLTLDecomposition(Global_LTL);
        TasksList tasks(Global_LTL);
        tasks.GetAllTasks();

        /*** 4. Initialize agents ***/
        std::vector<Agent> agents = TaskAssignment::InitializeAgents();
        int num_agents = agents.size();

        std::shared_ptr<Graph_t<SquareCell*>> true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid, false, true);
        /********************************* Generate Occupancy Grid Map ***********************************/
        std::shared_ptr<SquareGrid> grid_ipas = GridGraph::CreateSquareGrid(num_row, num_col, 1);
        std::shared_ptr<SquareGrid> grid_standard = GridGraph::CreateSquareGrid(num_row, num_col, 1);
        for (int i = 0; i < num_row * num_col; i++){
            grid_ipas->SetCellProbability(i,0.5);
            grid_standard->SetCellProbability(i,0.5);
        }

        // Set Task for grid
        for (int i = 0; i < Global_LTL.task_info.size();i++){
            grid_ipas->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);	
            grid_ipas->SetCellOccupancy(Global_LTL.task_info[i].pos_, OccupancyType::INTERESTED);
            grid_standard->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);	
            grid_standard->SetCellOccupancy(Global_LTL.task_info[i].pos_, OccupancyType::INTERESTED);
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
        IGdata ipas_data = GridGraph::IPAS(agents,tasks,num_sensors,grid_ipas,graph_ipas,true_graph,thred);
        root["IPAS costs"] = ipas_data.cost_paths_;
        root["IPAS entropy red"] = ipas_data.entropy_reduction_;
        root["IPAS iterations"] = ipas_data.num_iteration_;


        std::cout << "IPAS COMPLETED" << std::endl;
        //====================================== Standard Information Gain =======================================//
        IGdata standardig_data = GridGraph::StandardIG(agents,tasks,num_sensors,grid_standard,graph_standard,true_graph,thred);
        root["Standard IG costs"] = standardig_data.cost_paths_;
        root["Standard IG entropy red"] = standardig_data.entropy_reduction_;
        root["Standard IG iterations"] = standardig_data.num_iteration_;
        file << root;
        std::cout << "STANDARD IG COMPLEDTED" << std::endl;
    }
    
                 
    return 0;
}