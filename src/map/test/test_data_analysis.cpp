// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <cmath>
#include <tuple>
#include <string>
#include <fstream>
#include "json/json.h"

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

    // Number of sensors
    int64_t num_sensors = 5;
    // Number of agents
    int num_Agents = 2;
    // Number of tasks
    int num_Tasks_inde = 8;
    int num_Tasks_de = 0;
    // Obstacle percentage
    double obs_percentage = 0.3;
    
    // Generate random true grid map
    lcm::LCM lcm;
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
        true_grid->SetRandomOccupanyMap(obs_percentage);

        // Determine the tasks position randomly 
        tasks = true_grid->SetRandomTasks(num_Tasks_inde, num_Tasks_de);
        tasks.GetAllTasks();

        // Determine the position of agents randomly 
        agents = true_grid->SetRandomAgents(num_Agents,num_Tasks_inde,num_Tasks_de);
        
        // Generate the graph corresponding to the information shown above
        true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid, false, false);
        
        // Check whether the occupancy grid map is valid
        flag_valid = true_grid->OccupancyGridMapValidity(agents, tasks, true_graph);
    }
    
    
    std::vector<Path_t<SquareCell*>> path_vis;
    for(int i=0; i < num_Agents; i++){
        int64_t init_pos = agents[i].init_pos_;
        auto agent_cell = true_graph->GetVertexFromID(init_pos);
        Path_t<SquareCell*> path = {agent_cell->state_};
        path_vis.push_back(path);
    }

    // Generate plot
    GraphVis vis;
	Mat vis_img;
	vis.VisSquareGrid(*true_grid, vis_img);
	vis.VisSquareGridGraph(*true_graph, vis_img, vis_img, true);
    for (int i = 0; i < num_Agents; i++){
		vis.VisSquareGridPath(path_vis[i], vis_img, vis_img, i);
	}
	imwrite("result_occupancy_grid_map.jpg",vis_img);


    /********************************* Generate Occupancy Grid Map ***********************************/
    std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(num_row, num_col, 1);
	for (int i = 0; i < num_row * num_col; i++){
		grid->SetCellProbability(i,0.5);
	}

    // Set Task for grid
    for(int ti = 0; ti < tasks.tasks_.size(); ti++){
        grid->SetInterestedRegionLabel(tasks.tasks_[ti].pos_.front(),ti+2);
    }
    // Set Agent for grid
    for(auto& agent:agents){
        grid->SetCellOccupancy(agent.init_pos_, OccupancyType::FREE);
    }

	/*** 2. Construct a graph from the uncertain square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, false, true);
    
    std::vector<Path_t<SquareCell*>> path_vis_u;
    for(int i=0; i < num_Agents; i++){
        int64_t init_pos = agents[i].init_pos_;
        auto agent_cell = true_graph->GetVertexFromID(init_pos);
        Path_t<SquareCell*> path = {agent_cell->state_};
        path_vis_u.push_back(path);
    }

    // Generate plot
    GraphVis vis1;
	Mat vis_img1;
	vis.VisSquareGrid(*grid, vis_img1);
	vis.VisSquareGridGraph(*graph, vis_img1, vis_img1, true);
    for (int i = 0; i < num_Agents; i++){
		vis.VisSquareGridPath(path_vis[i], vis_img1, vis_img1, i);
	}
	imwrite("result_occupancy_grid_map_uncertain.jpg",vis_img1);


    /**************************************************************************/
	/********************************* IPAS ***********************************/
	/**************************************************************************/
    // The flag of task assignment
	bool Flag_ts = 0;
	// The flag of ipas
	bool Flag_ipas = 0;
	
	int T = 0;

	clock_t cbba_time;
	cbba_time = clock();
	std::vector<double> entropy_trend;
	std::vector<int64_t> range_trend;
	std::map<int64_t, std::vector<double>> entropy_trend_paths;
    // while(T < 1){
	while(Flag_ipas == false){
		T++;
		/***************************************************************************************/
		/********************************** Task Assignment ************************************/
		/***************************************************************************************/
		std::cout << "**********************************************" << std::endl;
		std::cout << "**********************************************" << std::endl;
		std::cout << "**********************************************" << std::endl;
		std::cout << "Iteration T " << T << std::endl;
		TaskAssignment::ResetTaskAssignmentInfo(agents);
		Flag_ts = 0;
		// Compute the entropy of map 
		entropy_trend.push_back(GridGraph::EntropyMap(graph));
		
		while (Flag_ts != 1){
			for (int i = 0; i < agents[0].num_agents_; i++)
				agents[i].iter_ = 0;

			/*** 6. Communication among neighbors ***/
			TaskAssignment::communicate(agents);
			//Update the history of iteration neighbors
			for (int i = 0; i < agents[0].num_agents_; i++)
				agents[i].history_.iter_neighbors_his.push_back(agents[i].iteration_neighbors_);
			
			/*** 7. Bundle Operations ***/
			/*** 7.1 Remove the out-bid task from the bundle ***/
			TaskAssignment::bundle_remove(agents);
			/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
			TaskAssignment::bundle_add(tasks, graph, agents);
			/*** 7.3 Check whether the assignment converge or not ***/
			Flag_ts = TaskAssignment::success_checker(agents);
			/*** 7.4 Update the number of interation ***/
			// Increase the iteration
			for (int i = 0; i < agents[0].num_agents_; i++)
				agents[i].iter_++;
		}
		std::cout << "====================================================" << std::endl;
		std::cout << "Result of Task Assignment "<<std::endl;
		std::cout << "CBBA iteration is " << agents[0].iter_ << std::endl;
		
		std::cout << "Task assignment result is: " << std::endl;
		for(auto& agent: agents){
			std::cout << "Vehicle " << agent.idx_ << std::endl;
			std::cout << "The task assignment is: "; 
			for (auto &t: agent.cbba_path_){
				std::cout << t << ", ";
			}
			std::cout << std::endl;
		}
		std::cout << "====================================================" << std::endl;


		// Compute the paths for each vehicle while statisfying the local specification
		std::map<int64_t, Path_t<SquareCell*>> paths = {};
		for(auto& agent: agents){
			Path_t<SquareCell*> path = GridGraph::PathComputationAStar(tasks, graph, agent.cbba_path_, agent.init_pos_);
			paths[agent.idx_] = path;
		}
        for(auto& pp: paths){
            std::cout << "For vehicle " << pp.first << ", the path length is " << pp.second.size() << std::endl; 
        }
        std::vector<Path_t<SquareCell*>> path_collections = TaskAssignment::PathsDecomposition(tasks,agents,paths);

		// Determine the subregion with given paths
        std::vector<int64_t> sub_domain = {};
		std::vector<int64_t> domain = GridGraph::SubRegionFromPaths(path_collections, grid, graph);
		for(auto& cell: domain){
			std::vector<int64_t>::iterator it_v = std::find(sub_domain.begin(), sub_domain.end(), cell);
			if(it_v == sub_domain.end()){
				sub_domain.push_back(cell);
			}
		}
		std::cout << "The dimension of sub region is " << sub_domain.size() << std::endl;
	
		// GraphVisLCM::TransferInfoLCM(graph, paths, {});
        GraphVisLCM::TransferRandomInfoLCM(graph,paths,{},tasks,agents);
		
		// Check the convergence condition
		std::pair<bool, std::vector<double>> flag_entropy_pair = GridGraph::ConvergenceCheck(paths, graph);
		Flag_ipas = flag_entropy_pair.first;
		if(Flag_ipas == true){
			std::cout << "Convergence is achieved. " << std::endl;
			for(int agent_idx = 0; agent_idx < flag_entropy_pair.second.size(); agent_idx++){
				entropy_trend_paths[agent_idx].push_back(flag_entropy_pair.second[agent_idx]);
			}
			// Print out the final cost of path after convergence
			for(auto& path: paths){
				double rewards = 0.0;
				for(int idx_c = 1; idx_c < path.second.size(); idx_c++){
					rewards = rewards + 200.0 * path.second[idx_c]->p_ + (1.0 - path.second[idx_c]->p_);
				}
				std::cout << "Now the cost of path for " << "agent " << path.first << " is " << rewards << std::endl;
			}
			break;
		}

		// Store the entropy of current paths
		for(int agent_idx = 0; agent_idx < flag_entropy_pair.second.size(); agent_idx++){
			entropy_trend_paths[agent_idx].push_back(flag_entropy_pair.second[agent_idx]);
		}

		// Determine the set of vertices which has non zero information gain
		std::vector<int64_t> vertex_ig;
		for(auto& c_v: sub_domain){
			Vertex_t<SquareCell*>* vert = graph->GetVertexFromID(c_v);
			std::vector<int64_t>::iterator it1 = std::find(vertex_ig.begin(), vertex_ig.end(), vert->state_->id_);
			if(it1 == vertex_ig.end()){
				vertex_ig.push_back(vert->state_->id_);
			}
			
			std::vector<int64_t> neighbors = vert->GetNeighbourIDs();
			for(auto& neighb_id: neighbors){
				std::vector<int64_t>::iterator it = std::find(vertex_ig.begin(), vertex_ig.end(), neighb_id);
				if(it == vertex_ig.end()){
					vertex_ig.push_back(neighb_id);
				}
			}
		}
		range_trend.push_back(vertex_ig.size());
		
		// Update the information gain
		for(int cell = 0; cell < true_grid->num_row_ * true_grid->num_col_; cell++){
			std::vector<int64_t>::iterator it = std::find(vertex_ig.begin(), vertex_ig.end(), cell);
			Vertex_t<SquareCell*>* current_v = graph->GetVertexFromID(cell);
			if(it == vertex_ig.end()){
				current_v->state_->ig_ = 0.0;
			}
			else{
				current_v->state_->ComputeIG(grid, graph, sub_domain);
			}	
		}

		// Select the sensors position  
		std::vector<int64_t> sensor_pos = GridGraph::SelectSensorsPos(num_sensors, graph);
        std::cout << "The sensor pos is ";
        for(auto&sensor: sensor_pos){
            std::cout << sensor << ", ";
        }
        std::cout << std::endl;

		GraphVisLCM::TransferRandomInfoLCM(graph,paths,sensor_pos,tasks,agents);
		// Update the uncertain map correspondingly
		GridGraph::UpdateUncertainMap(sensor_pos, graph, true_graph);
		std::cout << std::endl;
	}
	cbba_time = clock()-cbba_time;
	std::cout << "Total running time required is " << double(cbba_time)/CLOCKS_PER_SEC << std::endl;
	
	double decrease_rate = (entropy_trend[0] - entropy_trend.back())/entropy_trend[0]; 
	std::cout << "The Uncertainty of map is decrease by " << decrease_rate << std::endl;
	// Plot the information of entropy of map, dimension of subregion along iterations
	GraphVisLCM::TransferDataTrendLCM(entropy_trend, range_trend, entropy_trend_paths);

    return 0;
}