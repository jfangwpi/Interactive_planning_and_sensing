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
    while(flag_IPAS != true){
        ipas_tt ++;
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);
        
        std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);
        std::cout << "Convergence of CBBA is achieved." << std::endl;
        for (auto agent: vehicle_team_->auto_team_){
            if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
                if (path_ltl_[agent->vehicle_.idx_].size() == 0){
                    Path_t<SquareCell*> empty_path;
                    Vertex_t<SquareCell*>* init_pos = agent->vehicle_.local_graph_->GetVertexFromID(agent->vehicle_.pos_);
                    empty_path.push_back(init_pos->state_);
                    path_map_[agent->vehicle_.idx_] = empty_path;
                }
                else{
                    path_map_[agent->vehicle_.idx_] = path_ltl_[agent->vehicle_.idx_];
                }
            }
        }

        // Check the termination condition
        flag_IPAS = IPASMeasurement::IPASConvergence(vehicle_team_,path_ltl_);
        if(flag_IPAS == true){break;}
        //===============================================================================================//
        //============================================= IPAS ============================================//
        //===============================================================================================// 
        // Measurement
        IPASMeasurement::InformationDrivenHSpots(vehicle_team_);
        TasksSet sensing_tasks_ = IPASMeasurement::ConstructMeasurementTasks(vehicle_team_);
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,sensing_tasks_);

        std::map<int64_t,Path_t<SquareCell*>> path_sensing_ = IPASMeasurement::GeneratePaths(vehicle_team_,sensing_tasks_,TaskType::MEASURE);
        for (auto agent: vehicle_team_->auto_team_){
            if (agent->vehicle_.vehicle_type_ == TaskType::MEASURE){
                if (path_sensing_[agent->vehicle_.idx_].size() == 0){
                    Path_t<SquareCell*> empty_path;
                    Vertex_t<SquareCell*>* init_pos = agent->vehicle_.local_graph_->GetVertexFromID(agent->vehicle_.pos_);
                    empty_path.push_back(init_pos->state_);
                    path_map_[agent->vehicle_.idx_] = empty_path;
                }
                else{
                    path_map_[agent->vehicle_.idx_] = path_sensing_[agent->vehicle_.idx_];
                }
            }
        }

        // Update sensors' positions
        for(auto&agent:vehicle_team_->auto_team_){
            if(!agent->vehicle_.task_path_.empty()){
                int64_t ss_task = sensing_tasks_.GetTaskPosFromID(agent->vehicle_.task_path_.back());
                agent->vehicle_.pos_ = ss_task;
            }
        }

        std::vector<int64_t> hspts;
        for(auto cc: vehicle_team_->auto_team_[0]->vehicle_.hotspots_){
            hspts.push_back(cc.first);
        }
        GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,hspts,hspts);
     
        
        // Update the map
        for(auto&agent:vehicle_team_->auto_team_){
            agent->vehicle_.UpdateLocalMap(true_graph,sensing_tasks_);
        }

        IPASMeasurement::MergeLocalMap(vehicle_team_);
    }

    // Compute path cost - IPAS
    int64_t act_path_length_ = 0;
    for(auto agent: vehicle_team_->auto_team_){
        if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
            if(!agent->vehicle_.task_path_.empty()){
                path_map_[agent->vehicle_.idx_] = agent->vehicle_.PathComputation(tasks_,agent->vehicle_.local_graph_,agent->vehicle_.task_path_,agent->vehicle_.pos_);
            }
            else{
                Path_t<SquareCell*> empty_path_ = {};
                path_map_[agent->vehicle_.idx_] = empty_path_;
            }
            act_path_length_ += path_map_[agent->vehicle_.idx_].size();
        }
        else if (agent->vehicle_.vehicle_type_ == TaskType::MEASURE){
            Path_t<SquareCell*> empty_path_ = {};
            Vertex_t<SquareCell*>* vrt = agent->vehicle_.local_graph_->GetVertexFromID(agent->vehicle_.pos_);
            empty_path_.push_back(vrt->state_);
            path_map_[agent->vehicle_.idx_] = empty_path_;
        }   
    }
    // Compute path cost - Actucal path cost with perfect knowledge of environment
    int64_t opt_path_length_ = IPASMeasurement::GenerateTruePathsCost(vehicle_team_,tasks_,true_graph);
    // Display IPAS results
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++" <<std::endl;
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++" <<std::endl;
    std::cout << "++++++++++++++++++++IPAS CONVERGED++++++++++++++++++++++" <<std::endl;
    std::cout << "The actual paths length is " << act_path_length_ << std::endl;
    std::cout << "The optimal path length is " << opt_path_length_ << std::endl;

    GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,{});
    std::cout << "The total number of iteration for IPAS is " << ipas_tt << std::endl;
	return 0;
}