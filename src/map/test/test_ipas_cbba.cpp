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
#include "map/square_grid.hpp"
#include "vis/graph_vis_lcm.hpp"
#include "cbba/cbba_agent.hpp"
#include "cbba/cbba_task.hpp"
using namespace librav;

int main(int argc, char** argv )
{
	// Needs to be defined:
	int64_t num_sensors = 3;
	// Dimension of grid 
	int64_t num_row = 10;
    int64_t num_col = 10;

	srand(time(NULL));
    lcm::LCM lcm;
	/********************************* Certain Map ***********************************/
    // CBBA with full knowledge of map
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
	std::vector<Agent> agents_group = TaskAssignment::InitializeAgents();
	int num_agents = agents_group.size();

	std::shared_ptr<Graph_t<SquareCell*>> true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid, false, true);
	std::vector<Path_t<SquareCell*>> paths = {};
	// UncertainMap::TransferInfoLCM(true_graph, paths, {});

	/********************************* Uncertain Map ***********************************/
	// /*** 1. Read from config file: map.ini ***/
	std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(num_row, num_col, 1);

	for (int i = 0; i < num_row * num_col; i++){
		grid->SetCellProbability(i,0.5);
	}

    // Set Task for grid
    for (int i = 0; i < Global_LTL.task_info.size();i++){
		grid->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);	
		grid->SetCellOccupancy(Global_LTL.task_info[i].pos_, OccupancyType::INTERESTED);
	}
    // Set Agent for grid
    for(auto& agent:agents_group){
        grid->SetCellOccupancy(agent.init_pos_, OccupancyType::FREE);
    }

	/*** 2. Construct a graph from the uncertain square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, false, true);
	// UncertainMap::TransferInfoLCM(graph, paths, {});


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
		TaskAssignment::ResetTaskAssignmentInfo(agents_group);
		Flag_ts = 0;
		// Compute the entropy of map 
		entropy_trend.push_back(GridGraph::EntropyMap(graph));
		
		while (Flag_ts != 1){
			for (int i = 0; i < agents_group[0].num_agents_; i++)
				agents_group[i].iter_ = 0;

			/*** 6. Communication among neighbors ***/
			TaskAssignment::communicate(agents_group);
			//Update the history of iteration neighbors
			for (int i = 0; i < agents_group[0].num_agents_; i++)
				agents_group[i].history_.iter_neighbors_his.push_back(agents_group[i].iteration_neighbors_);
			
			/*** 7. Bundle Operations ***/
			/*** 7.1 Remove the out-bid task from the bundle ***/
			TaskAssignment::bundle_remove(agents_group);
			/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
			TaskAssignment::bundle_add(tasks, graph, agents_group);
			/*** 7.3 Check whether the assignment converge or not ***/
			Flag_ts = TaskAssignment::success_checker(agents_group);
			/*** 7.4 Update the number of interation ***/
			// Increase the iteration
			for (int i = 0; i < agents_group[0].num_agents_; i++)
				agents_group[i].iter_++;
		}
		std::cout << "====================================================" << std::endl;
		std::cout << "Result of Task Assignment "<<std::endl;
		std::cout << "CBBA iteration is " << agents_group[0].iter_ << std::endl;
		
		std::cout << "Task assignment result is: " << std::endl;
		for(auto& agent: agents_group){
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
		for(auto& agent: agents_group){
			Path_t<SquareCell*> path = GridGraph::PathComputationAStar(tasks, graph, agent.cbba_path_, agent.init_pos_);
			paths[agent.idx_] = path;
		}
        std::vector<Path_t<SquareCell*>> path_collections = TaskAssignment::PathsDecomposition(tasks,agents_group,paths);

		// Determine the subregion with given paths
		std::vector<int64_t> sub_domain = GridGraph::SubRegionFromPaths(path_collections, grid, graph);
		std::cout << "The dimension of sub region is " << sub_domain.size() << std::endl;
	
		GraphVisLCM::TransferInfoLCM(graph, paths, {});
		
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
			std::vector<SquareCell *> neig_verts= grid->GetNeighbors(vert->state_->id_,false);
			std::vector<int64_t> neighbors;
			for(auto& vv: neig_verts){
				neighbors.push_back(vv->id_);
			}
			// std::vector<int64_t> neighbors = vert->GetNeighbourIDs();
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

		GraphVisLCM::TransferInfoLCM(graph, paths, sensor_pos);
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