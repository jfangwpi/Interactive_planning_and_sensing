/*
 * test_gpc_ig.cpp
 *
 *  Created on: July 8, 2019
 *      Author: jfang
 */

/*** This code is implemented for IPAS by using Guassian process occupancy grid map ***/
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
#include "gplib/log_likelihood_classification.hpp"

using namespace librav;

int main(int argc, char** argv )
{
	// Needs to be defined:
	int64_t num_sensors = 5;
    // Initialize the gaussian process occupancy grid map
    std::vector<Eigen::VectorXd> training_x_;
    std::vector<int> training_y_;
    std::vector<int64_t> certain_cells;

	// Dimension of grid 
	int64_t num_row = 30;
    int64_t num_col = 30;

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
	std::vector<int64_t> interested_id = {0,17,29,125,199,284,398,409,476,510,701,684,870,899};
	// std::vector<int64_t> interested_id = {0,99,7,50,95};
	for(auto& kk: interested_id){
		Vertex_t<SquareCell*>* vv = graph->GetVertexFromID(kk);
		vv->state_->p_ = 0.0;
		vv->state_->occupancy_ = OccupancyType::FREE;
		certain_cells.push_back(vv->state_->id_);
		Eigen::VectorXd targ_(kDimension);
        targ_(0) = vv->state_->coordinate_.y;
        targ_(1) = vv->state_->coordinate_.x;
		training_x_.push_back(targ_);
		training_y_.push_back(0);
	}

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
	Eigen::VectorXd pi_pred = Eigen::VectorXd(training_x_.size());
	Eigen::VectorXd W_sr_pred = Eigen::VectorXd(training_x_.size());
	Eigen::MatrixXd LLT_pred = Eigen::MatrixXd(training_x_.size(),training_x_.size());
	Eigen::VectorXd kernel_opt = Eigen::VectorXd(kDimension);

	std::vector<int64_t> measurements = interested_id;
    // while(T < 26){
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
		entropy_trend.push_back(GridGraph::EntropyMap(graph,measurements));
		
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
		std::cout << "The reward of each task is " << std::endl;
		std::cout << agents_group[0].cbba_reward_ << std::endl;
		
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
        std::vector<int64_t> sub_domain = {};
		std::vector<int64_t> domain = GridGraph::SubRegionFromPaths(path_collections, grid, graph);
		for(auto& cell: domain){
			std::vector<int64_t>::iterator it_v = std::find(sub_domain.begin(), sub_domain.end(), cell);
			if(it_v == sub_domain.end()){
				sub_domain.push_back(cell);
			}
		}
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
		std::vector<int64_t> sensor_pos = GridGraph::SelectSensorsPos(num_sensors, certain_cells, graph);
        std::cout << "The sensor pos is ";
        for(auto&sensor: sensor_pos){
            std::cout << sensor << ", ";
			auto ss_c = graph->GetVertexFromID(sensor);
			// std::cout << "(" << ss_c->state_->coordinate_.y << ", " << ss_c->state_->coordinate_.x << ")" << std::endl;
            std::vector<int64_t>::iterator it_cc = std::find(certain_cells.begin(), certain_cells.end(), sensor);
            
            if(it_cc == certain_cells.end()){
                certain_cells.push_back(sensor);
            }
        }
        std::cout << std::endl;

		GraphVisLCM::TransferInfoLCM(graph, paths, sensor_pos);
		
		// Update the uncertain map correspondingly
		// GridGraph::UpdateUncertainMap(sensor_pos, graph, true_graph);
        if (certain_cells.size() <= 10){
            std::cout << "Not enough training data" << std::endl;
            GridGraph::UpdateUncertainMap(sensor_pos, graph, true_graph);
            for(auto&ss: sensor_pos){
                Vertex_t<SquareCell*>* ss_v = graph->GetVertexFromID(ss);
                Eigen::VectorXd ss_v_coord(kDimension);
                ss_v_coord(0) = ss_v->state_->position_.y;
                ss_v_coord(1) = ss_v->state_->position_.x;

                std::vector<Eigen::VectorXd>::iterator it_v = std::find(training_x_.begin(),training_x_.end(),ss_v_coord);
                if(it_v == training_x_.end()){
                    training_x_.push_back(ss_v_coord);
                    auto true_v = true_graph->GetVertexFromID(ss);
                    // std::cout << "true_v occupancy is ";
                    if(true_v->state_->occupancy_ == OccupancyType::OCCUPIED){
                        // std::cout << "Occupied" << std::endl;
                        training_y_.push_back(1);
                    }
                    else{
                        // std::cout << "FREE" << std::endl;
                        training_y_.push_back(0);
                    }
                }

				std::vector<Vertex_t<SquareCell*>*> neighbs = ss_v->GetNeighbours();
				for(auto&nn: neighbs){
					Eigen::VectorXd nn_v_coord(kDimension);
                	nn_v_coord(0) = nn->state_->position_.y;
                	nn_v_coord(1) = nn->state_->position_.x;

 
					std::vector<Eigen::VectorXd>::iterator it_n = std::find(training_x_.begin(),training_x_.end(),nn_v_coord);
					if(it_n == training_x_.end()){
						training_x_.push_back(nn_v_coord);
						auto true_nn = true_graph->GetVertexFromID(nn->state_->id_);
						// std::cout << "true_v occupancy is ";
						if(true_nn->state_->occupancy_ == OccupancyType::OCCUPIED){
							// std::cout << "Occupied" << std::endl;
							training_y_.push_back(1);
						}
						else{
							// std::cout << "FREE" << std::endl;
							training_y_.push_back(0);
						}
					}
				}
            }
        }
        else{
            std::cout << "START THE GP ESTIMATION!!!!!!!!!!!!!" << std::endl;
            GridGraph::UpdateUncertainMapGP(sensor_pos,certain_cells,training_x_,training_y_,pi_pred,W_sr_pred,LLT_pred,kernel_opt,graph,true_graph);
        
			// std::cout << "The optimal kernel length is " << std::endl;
			// std::cout << kernel_opt << std::endl;
		}
		std::cout << std::endl;
        
		measurements = {};
		for(auto& kk: training_x_){
			measurements.push_back(GridGraph::GetIDFromCoordinate(kk));
		}
	}

	// Build the gpc
	// std::cout << "GP GP GP GP GP. " << std::endl;
    Kernel::Ptr kernel_star_ = RBFKernel::Create(kernel_opt);
	Eigen::VectorXd *targets = new Eigen::VectorXd(training_y_.size());
    for(size_t kk = 0; kk < training_y_.size(); kk++){
        (*targets)(kk) = training_y_[kk];
    }
	std::shared_ptr<std::vector<Eigen::VectorXd>> points_ptr(new std::vector<Eigen::VectorXd>(training_x_));

	GaussianProcessClassification gpc(kernel_star_, 0.0, points_ptr, *targets, training_x_.size());
	std::vector<Vertex_t<SquareCell*>*> verts_tt = graph->GetAllVertex();
	std::cout << "The number of training data is " << training_x_.size() << std::endl;
	for(auto& vv: verts_tt){
		std::vector<int64_t>::iterator it_interested = std::find(interested_id.begin(),interested_id.end(),vv->state_->id_);
		if(it_interested == interested_id.end()){
			Eigen::VectorXd targ_(kDimension);
			targ_(0) = vv->state_->coordinate_.y;
			targ_(1) = vv->state_->coordinate_.x;
			std::vector<Eigen::VectorXd>::iterator it_training = std::find(training_x_.begin(),training_x_.end(),targ_);
			if(it_training == training_x_.end()){
				std::pair<bool,double> pred_p = gpc.Predict(targ_,&pi_pred,&W_sr_pred,&LLT_pred);
				vv->state_->p_ = pred_p.second; 
			}	
		}
	}

	std::vector<int64_t> training_id;
	for(auto& dt: training_x_){
		training_id.push_back(30*dt(0) + dt(1));
	}
	GraphVisLCM::TransferGraphLCM(graph,training_id);

	


	cbba_time = clock()-cbba_time;
	std::cout << "Total running time required is " << double(cbba_time)/CLOCKS_PER_SEC << std::endl;
	
	double decrease_rate = (entropy_trend[0] - entropy_trend.back())/entropy_trend[0]; 
	std::cout << "The Uncertainty of map is decrease by " << decrease_rate << std::endl;
	// Plot the information of entropy of map, dimension of subregion along iterations
	// GraphVisLCM::TransferDataTrendLCM(entropy_trend, range_trend, entropy_trend_paths);
    
	return 0;
}