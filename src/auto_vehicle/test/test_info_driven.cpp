/*
 * test_ipas_decentralized.cpp
 *
 *  Created on: July 29, 2019
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

using namespace librav;

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
    std::shared_ptr<SquareGrid> init_grid = GridGraph::CreateSquareGrid(num_row,num_col,1,vehicle_team_,tasks_);
	std::shared_ptr<Graph_t<SquareCell*>> init_graph = GridGraph::BuildGraphFromSquareGrid(init_grid, false, true);

    std::shared_ptr<SquareGrid> true_grid = GridGraph::CreateSquareGrid();
    std::shared_ptr<Graph_t<SquareCell *>> true_graph = GridGraph::BuildGraphFromSquareGrid(true_grid,false, false);
    //===============================================================================================//
    //============================================= CBBA ============================================//
    //===============================================================================================//      
    for(auto&agent: vehicle_team_->auto_team_){
        agent->vehicle_.SetLocalMap(init_grid);
    }
    bool flag_IPAS = false;
    int64_t ipas_tt = 0;
    std::map<int64_t,Path_t<SquareCell*>> path_map_;
    std::map<int64_t,std::vector<double>> paths_entropy_; 
    std::vector<double> map_entropy_;
    std::vector<int64_t> nz_ig_domain_;
    while(flag_IPAS != true){
        ipas_tt ++;
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);
        
        std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);
        std::cout << "Convergence of CBBA is achieved." << std::endl;

        // Record the map entropy for each vehicle
        map_entropy_.push_back(GridGraph::EntropyMap(vehicle_team_->auto_team_[0]->vehicle_.local_graph_));
        // Record the path entropy for each vehicle
        for(auto p: path_ltl_){
            double en_p = GridGraph::EntropyPath(p.second, vehicle_team_->auto_team_[p.first]->vehicle_.local_graph_);
            paths_entropy_[p.first].push_back(en_p);
        }
        // Check the termination condition
        flag_IPAS = IPASMeasurement::IPASConvergence(vehicle_team_,path_ltl_);
        if(flag_IPAS == true){break;}
        //===============================================================================================//
        //============================================= IPAS ============================================//
        //===============================================================================================// 
        // Measurement
        IPASMeasurement::InformationDrivenHSpots(vehicle_team_);

        nz_ig_domain_.push_back(num_row*num_col);
    
        TasksSet sensing_tasks_ = IPASMeasurement::ConstructMeasurementTasks(vehicle_team_);
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,sensing_tasks_);

        for(auto&agent:vehicle_team_->auto_team_){
            if(!agent->vehicle_.task_path_.empty()){
                int64_t ss_task = sensing_tasks_.GetTaskPosFromID(agent->vehicle_.task_path_.back());
                agent->vehicle_.pos_ = ss_task;
                std::cout << "The initial pos for vehicle " << agent->vehicle_.idx_ << " is updated to " << agent->vehicle_.pos_ << std::endl;
            }
        }

        // for(auto& agent: vehicle_team_->auto_team_){
        std::map<int64_t,Path_t<SquareCell*>> map_path_ = {};
        std::vector<int64_t> hspts;
        for(auto cc: vehicle_team_->auto_team_[0]->vehicle_.hotspots_){
            hspts.push_back(cc.first);
        }
        GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,hspts,hspts);
        // }
        // GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,{});
        
        // Update the map
        for(auto&agent:vehicle_team_->auto_team_){
            agent->vehicle_.UpdateLocalMap(true_graph,sensing_tasks_);
        }
        std::cout << "Measurements are taken is updated." << std::endl;

        // for(auto& agent: vehicle_team_->auto_team_){
        //     std::map<int64_t,Path_t<SquareCell*>> map_path_ = {};
        //     std::vector<int64_t> hspts;
        //     for(auto cc: vehicle_team_->auto_team_[0]->vehicle_.hotspots_){
        //         hspts.push_back(cc.first);
        //     }
        //     GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,map_path_,hspts);
        // }

        IPASMeasurement::MergeLocalMap(vehicle_team_);
    }

    std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);
    for(auto& agent: vehicle_team_->auto_team_){
        GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_ltl_,{});
    }
    std::cout << "The total number of iteration for IPAS is " << ipas_tt << std::endl;


    double decrease_rate = (map_entropy_[0] - map_entropy_.back())/map_entropy_[0]; 
	std::cout << "The Uncertainty of map is decrease by " << decrease_rate << std::endl;
	// Plot the information of entropy of map, dimension of subregion along iterations
	GraphVisLCM::TransferDataTrendLCM(map_entropy_, nz_ig_domain_, paths_entropy_);

	return 0;
}