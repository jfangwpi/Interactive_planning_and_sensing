/*
 * test_cbba.cpp
 *
 *  Created on: Feb 15, 2017
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

// self-defined library
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"

#include "cbba/cbba_agent.hpp"
#include "cbba/cbba_task.hpp"

using namespace librav;

int main(int argc, char** argv )
{   
    int64_t num_row = 10;
    int64_t num_col = 10;
    std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(num_row, num_col, 1);
    // Set the probability
	for (int i = 0; i < num_row * num_col; i++){
		grid->SetCellProbability(i,0.5);
	}

    // Construct the TasksSet
    int64_t num_tk = 3;
    std::vector<int64_t> task_list = {7,50,95};
    TasksList tasks = TasksList(task_list);
    
    std::cout << "The task set is done." << std::endl;

    // Construct the AutoVehicle
    int64_t num_v = 2;
    Eigen::VectorXd network_topo = Eigen::VectorXd::Ones(num_v);
    Agent agents[2] = {Agent(0,0,3,0,2),
                        Agent(1,99,3,0,2)};

    std::vector<Agent> agents_;
    for(int64_t ii = 0; ii < num_v; ii++){
        agents_.push_back(agents[ii]);
    }

    // Set Task for grid
    for (auto& tk: tasks.tasks_){
		grid->SetInterestedRegionLabel(tk.pos_.front(),tk.idx_);	
		grid->SetCellOccupancy(tk.pos_.front(), OccupancyType::INTERESTED);
	}

    // Set Agent for grid
    for(auto& qr:agents_){
        grid->SetCellOccupancy(qr.init_pos_, OccupancyType::FREE);
    }

	/*** 2. Construct a graph from the uncertain square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, false, true);
    
    // New lib
    // for(auto&qr: agents_){
    //     std::cout << "QuadRotor " << qr.idx_ << ": " << std::endl;
    //     qr.reward_update(tasks,graph);
    //     std::cout << qr.cbba_reward_ << std::endl;
    // }

    bool succFlag = 0;
	clock_t cbba_time;
	cbba_time = clock();
	while (succFlag != 1){
	//while(agent[0].Iter < 2){
		/*** 6. Communication among neighbors ***/
		std::cout << "Iteration is " << agents_[0].iter_ << "********************************" << std::endl;
		TaskAssignment::communicate(agents_);
		//Update the history of iteration neighbors
		for (int i = 0; i < agents_[0].num_agents_; i++)
			agents_[i].history_.iter_neighbors_his.push_back(agents_[i].iteration_neighbors_);

		std::cout << "================================= AFTER COMMUNICATION ================================== " << std::endl;
		for(int i = 0; i < agents_[0].num_agents_; i++){
			std::cout << "Vehicle " << i << std::endl;
			std::cout << "Winners for tasks are " << std::endl;
			std::cout << agents_[i].cbba_z_ << std::endl;
			std::cout << "Highest reward are " << std::endl;
			std::cout << agents_[i].cbba_y_ << std::endl;
			std::cout << "Path info is " << std::endl;
			for(auto &b: agents_[i].cbba_path_)
				std::cout << b << " ";
			std::cout << std::endl;
			std::cout << "Reward for each vehicle is " << std::endl;
			std::cout << agents_[i].cbba_reward_ << std::endl;
		
		}

		/*** 7. Bundle Operations ***/
		/*** 7.1 Remove the out-bid task from the bundle ***/
		TaskAssignment::bundle_remove(agents_);
		std::cout << "===================================== AFTER BUNDLE REMOVE ======================================" << std::endl;
		for(int i = 0; i < agents_[0].num_agents_; i++){
			std::cout << "Vehicle " << i << std::endl;
			std::cout << "Path info is " << std::endl;
			for(auto &b: agents_[i].cbba_path_)
				std::cout << b << " ";
			std::cout << std::endl;
		}
		/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
		TaskAssignment::bundle_add(tasks,graph,agents_);
		/*** 7.3 Check whether the assignment converge or not ***/
		succFlag = TaskAssignment::success_checker(agents_);
		std::cout << "The Flag for success is " << succFlag <<std::endl;
		/*** 7.4 Update the number of interation ***/
		
		// Increase the iteration
		for (int i = 0; i < agents_[0].num_agents_; i++)
			agents_[i].iter_++;

		std::cout << "====================================== AFTER INSERT NEW TASKS ========================================== " << std::endl;
		for(int i = 0; i < agents_[0].num_agents_; i++){
			std::cout << "Vehicle " << i << std::endl;
			std::cout << "Winners for tasks are " << std::endl;
			std::cout << agents_[i].cbba_z_ << std::endl;
			std::cout << "Highest reward are " << std::endl;
			std::cout << agents_[i].cbba_y_ << std::endl;
			std::cout << "Path info is " << std::endl;
			std::cout << "Path info is " << std::endl;
			for(auto &b: agents_[i].cbba_path_)
				std::cout << b << " ";
			std::cout << std::endl;
			std::cout << "Reward for each vehicle is " << std::endl;
			std::cout << agents_[i].cbba_reward_ << std::endl;

		}
	}
	cbba_time = clock()-cbba_time;
	std::cout << "CBBA iteration is " << agents_[0].iter_ << std::endl;
	std::cout << "The running time required is " << double(cbba_time)/CLOCKS_PER_SEC << std::endl;



	return 0;
}