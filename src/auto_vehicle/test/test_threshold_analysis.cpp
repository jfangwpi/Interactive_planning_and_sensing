/*
 * test_threshold_analysis.cpp
 *
 *  Created on: Sep 03, 2019
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

#include "auto_vehicle/auto_vehicle.hpp"
#include "vehicle/auto_team.hpp"
#include "vehicle/agent.hpp"
#include "task_assignment/cbba_impl.hpp"

#include "vis/graph_vis_lcm.hpp"

#include "config_reader/config_reader.hpp"

using namespace librav;

int main(int argc, char** argv )
{   
    int64_t num_row = 30;
    int64_t num_col = 30;
    // Number of sensors
    int64_t num_sensors = 4;
    // Number of agents
    int64_t num_actors = 4;
    // Number of tasks
    int64_t num_tasks = 10;
    int num_Tasks_de = 0;
    // Obstacle percentage
    double Obstacle_density = 0.4;
    int num_obstacles = (int)num_col * num_row * Obstacle_density;
    
    int64_t caseT = 1;
    int64_t NumPerCase = 50;
    std::ofstream file("/home/jfang/Workspace/GP_map/json/ipas-threshold-data.json");
    Json::Value root;
    
    double threshold_ = 0.1;
    while(threshold_ >= 0.0){
        //====================================== Random Occupancy Grid Map ==========================================//
        // Initialize the parameters
        std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team_;
        TasksSet tasks_;
        std::shared_ptr<SquareGrid> true_grid;
        std::shared_ptr<Graph_t<SquareCell*>> true_graph;
        std::vector<double> map_entropy;


        for(int num_case_ = 0; num_case_ < NumPerCase; num_case_++){
            Json::Value root;
            bool flag_valid = false;
            /********************************* Generate True Occupancy Grid Map ***********************************/
            while (flag_valid != true){
                // CBBA with full knowledge of map
                true_grid = GridGraph::CreateSquareGrid(num_row,num_col,1);
                // Generate ramdom occupancy grid map
                true_grid->SetRandomOccupanyMap(Obstacle_density);

                // Determine the tasks position randomly 
                tasks_ = true_grid->ConstructRandomLTLTasks(num_tasks);
            
                // Determine the position of agents randomly 
                vehicle_team_ = true_grid->ConstructRandomAutoTeam(num_actors,num_sensors,num_tasks);
                
                // Generate the graph corresponding to the information shown above
                true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid, false, false);
                
                // Check whether the occupancy grid map is valid
                flag_valid = true_grid->OccupancyGridMapValidity(vehicle_team_, tasks_, true_graph);
            }
            std::cout << "Random map is generated. " << std::endl;

            std::shared_ptr<SquareGrid> init_grid = GridGraph::CreateSquareGrid(num_row,num_col,1,vehicle_team_,tasks_);
	        std::shared_ptr<Graph_t<SquareCell*>> init_graph = GridGraph::BuildGraphFromSquareGrid(init_grid, false, true);
        
            for(auto&agent: vehicle_team_->auto_team_){
                agent->vehicle_.SetLocalMap(init_grid);
            }
            bool flag_dece_ = false;
            int64_t ipas_tt = 0;

            while(flag_dece_ != true){
                flag_dece_ = true;
                ipas_tt ++;
                CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);

                map_entropy.push_back(GridGraph::EntropyMap(vehicle_team_->auto_team_[0]->vehicle_.local_graph_));
                for(auto agent: vehicle_team_->auto_team_){
                    bool flag_agent = agent->vehicle_.IPASConvergence(tasks_,threshold_);
                    if (flag_agent == false){
                        flag_dece_ = false;
                        break;
                    }
                }

                if (flag_dece_ == true){break;}

                IPASMeasurement::ComputeHotSpots(vehicle_team_,tasks_);
                IPASMeasurement::HotSpotsConsensus(vehicle_team_);

                TasksSet sensing_tasks_ = IPASMeasurement::ConstructMeasurementTasks(vehicle_team_);
                CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,sensing_tasks_);
                for(auto&agent:vehicle_team_->auto_team_){
                    if(!agent->vehicle_.task_path_.empty()){
                        int64_t ss_task = sensing_tasks_.GetTaskPosFromID(agent->vehicle_.task_path_.back());
                        agent->vehicle_.pos_ = ss_task;
                    }
                }
                for(auto&agent:vehicle_team_->auto_team_){
                    agent->vehicle_.UpdateLocalMap(true_graph,sensing_tasks_);
                }
                IPASMeasurement::MergeLocalMap(vehicle_team_);
            }

            double opt_path_length_ = 0.0;
            double act_path_length_ = 0.0;
            int64_t success_threshold = 0;
            for(auto agent: vehicle_team_->auto_team_){
                if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
                    int64_t path_agent = agent->vehicle_.OPTPathComputation(tasks_,true_graph);
                    Path_t<SquareCell*> act_path = agent->vehicle_.PathComputation(tasks_);
                    act_path_length_ += double(act_path.size());
                    opt_path_length_ += double(path_agent);
                }
            }
            double relative_err = (act_path_length_ - opt_path_length_)/opt_path_length_;
            if(relative_err >= 0){success_threshold = 1;}
            else {success_threshold=0;}
            
            root["Threshold"] = threshold_;
            root["Actual path length"] = act_path_length_;
            root["Optimal path length"] = opt_path_length_;
            root["Relative error"] = relative_err;
            root["Success flag"] = int(success_threshold);
            root["Entropy reduction"] = (map_entropy[0] - map_entropy.back())/map_entropy[0];
            root["Iteration"] = int(ipas_tt);
            file << root;

            std::cout << "Actual path length: " << act_path_length_ << std::endl;
            std::cout << "Optimal path length: " << opt_path_length_ << std::endl;
            std::cout << "relative error: " << relative_err << std::endl;
            std::cout << "============================================== " << std::endl;
        }
        threshold_ -= 0.005;
    }
    
	return 0;
}