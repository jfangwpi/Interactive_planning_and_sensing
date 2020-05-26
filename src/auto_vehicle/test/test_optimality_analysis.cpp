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

#include "config_reader/config_reader.hpp"

using namespace librav;

int main(int argc, char** argv )
{   
    //===============================================================================================//
    //===========================================Task - Agent========================================//
    //===============================================================================================//
    // Parameters
    int64_t num_row = 20;
    int64_t num_col = 20;
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
    double threshold_ = 0.1;
    std::ofstream file("/home/jfang/Workspace/GP_map/json/ipas-data-threshold-30map.json");
    std::vector<double> cost_path_collection = {};
    std::map<int64_t,Path_t<SquareCell*>> path_map_ = {};
    std::map<int64_t,std::vector<double>> paths_entropy_  {}; 
    std::vector<double> map_entropy_ = {};
    std::vector<int64_t> nz_ig_domain_ = {};
    int64_t cont = 1;
    while (threshold_ > 0.0){
        Json::Value root;
        std::string CIDX = "Case" + std::to_string(cont); 
        root[CIDX]["threshold"] = threshold_;
        for(auto&agent: vehicle_team_->auto_team_){
            agent->vehicle_.SetLocalMap(init_grid);
        }
        bool flag_dece_ = false;
        int64_t ipas_tt = 0;
        path_map_ = {};
        paths_entropy_ = {}; 
        map_entropy_ = {};
        nz_ig_domain_ = {};
        while(flag_dece_ != true){
            flag_dece_ = true;
            ipas_tt ++;
            CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);
            
            for(auto agent: vehicle_team_->auto_team_){
                if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
                    if(!agent->vehicle_.task_path_.empty()){
                        path_map_[agent->vehicle_.idx_] = agent->vehicle_.PathComputation(tasks_);
                    }
                    else{
                        Path_t<SquareCell*> empty_path_ = {};
                        path_map_[agent->vehicle_.idx_] = empty_path_;
                    }
                }    
            }
            std::cout << "Convergence of CBBA is achieved." << std::endl;

            map_entropy_.push_back(GridGraph::EntropyMap(vehicle_team_->auto_team_[0]->vehicle_.local_graph_));
            for(auto& agent: vehicle_team_->auto_team_){
                bool flag_ve = agent->vehicle_.IPASConvergence(tasks_,threshold_);
                if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
                    Path_t<SquareCell*> path = agent->vehicle_.PathComputation(tasks_);
                    double entropy_path = GridGraph::EntropyPath(path, agent->vehicle_.local_graph_);
                    paths_entropy_[agent->vehicle_.idx_].push_back(entropy_path);
                }
                if(flag_ve == false){flag_dece_ = false;}
            }
            if(flag_dece_ == true){
                break;
            }
            //===============================================================================================//
            //============================================= IPAS ============================================//
            //===============================================================================================// 
            // Measurement
            IPASMeasurement::ComputeHotSpots(vehicle_team_,tasks_);
            std::vector<int64_t> local_hspots;
            for(auto agent: vehicle_team_->auto_team_){
                for(auto lhspt: agent->vehicle_.hotspots_){
                    local_hspots.push_back(lhspt.first);
                }
            }

            IPASMeasurement::HotSpotsConsensus(vehicle_team_);

            // Easy for simulation
            IPASMeasurement::MergeHSpots(vehicle_team_);

            nz_ig_domain_.push_back(vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_.size());

            TasksSet sensing_tasks_ = IPASMeasurement::ConstructMeasurementTasks(vehicle_team_);
            CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,sensing_tasks_);

            for(auto agent: vehicle_team_->auto_team_){
                if (agent->vehicle_.vehicle_type_ == TaskType::MEASURE){
                    if(!agent->vehicle_.task_path_.empty()){
                        path_map_[agent->vehicle_.idx_] = agent->vehicle_.PathComputation(sensing_tasks_);
                    }
                    else{
                        Path_t<SquareCell*> empty_path_ = {};
                        Vertex_t<SquareCell*>* vrt = agent->vehicle_.local_graph_->GetVertexFromID(agent->vehicle_.pos_);
                        empty_path_.push_back(vrt->state_);
                        path_map_[agent->vehicle_.idx_] = empty_path_;
                    }
                } 
            }

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
            GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,hspts,local_hspots);
            // }
            // GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,{});
            
            // Update the map
            for(auto&agent:vehicle_team_->auto_team_){
                agent->vehicle_.UpdateLocalMap(true_graph,sensing_tasks_);
            }
            std::cout << "Measurements are taken is updated." << std::endl;

            IPASMeasurement::MergeLocalMap(vehicle_team_);
        
            

            // Slow down for the image
            // bool key_ = false;
            // std::cout << "Enter 1 for next iteration. " << std::endl;
            // std::cin >> key_;
            // while(key_ != 1){continue;}
        }

        int64_t cost_path_ = 0;
        for(auto agent: vehicle_team_->auto_team_){
            if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
                if(!agent->vehicle_.task_path_.empty()){
                    path_map_[agent->vehicle_.idx_] = agent->vehicle_.PathComputation(tasks_);
                }
                else{
                    Path_t<SquareCell*> empty_path_ = {};
                    path_map_[agent->vehicle_.idx_] = empty_path_;
                }
                std::cout << "Vehicle " << agent->vehicle_.idx_ << "'s path length is " << path_map_[agent->vehicle_.idx_].size()<< std::endl;
                double path_cost = 0.0;
                if(!path_map_[agent->vehicle_.idx_].empty()){
                    for(int kk = 0; kk < path_map_[agent->vehicle_.idx_].size()-1; kk++){
                        path_cost += GridGraph::CalcHeuristicUncertain(path_map_[agent->vehicle_.idx_][kk],path_map_[agent->vehicle_.idx_][kk+1]);
                    }
                }
                cost_path_ += path_map_[agent->vehicle_.idx_].size();
                std::cout << "Path cost for vehicle " << agent->vehicle_.idx_ << " is " << path_cost << std::endl;
            }
            else if (agent->vehicle_.vehicle_type_ == TaskType::MEASURE){
                Path_t<SquareCell*> empty_path_ = {};
                Vertex_t<SquareCell*>* vrt = agent->vehicle_.local_graph_->GetVertexFromID(agent->vehicle_.pos_);
                empty_path_.push_back(vrt->state_);
                path_map_[agent->vehicle_.idx_] = empty_path_;
            }   
        }

        int64_t opt_path_length_ = 0;
        for(auto agent: vehicle_team_->auto_team_){
            if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
                int64_t path_agent = agent->vehicle_.OPTPathComputation(tasks_,true_graph);
                opt_path_length_ += path_agent;
            }
        }
        cost_path_collection.push_back(cost_path_);
        root[CIDX]["Total length"] = int(cost_path_);
        root[CIDX]["Entropy reduction"] = double((map_entropy_[0] - map_entropy_.back()))/map_entropy_[0];
        root[CIDX]["Iterations"] = int(ipas_tt);
        root[CIDX]["Optimal cost"] = int(opt_path_length_);
        file << root;
        std::cout << "==================================================" << std::endl;
        std::cout << "==================================================" << std::endl;
        std::cout << "==================================================" << std::endl;
        std::cout << "==================================================" << std::endl;
        std::cout << "The threshold is " << threshold_ << std::endl;
        std::cout << "The total cost of path is " << cost_path_ << std::endl;
        std::cout << "The optimal cost of path is " << opt_path_length_ << std::endl;
        std::cout << "The entropy reduction is " << double((map_entropy_[0] - map_entropy_.back()))/map_entropy_[0] << std::endl;
        std::cout << "The iteration of convergence is " << ipas_tt << std::endl;

        threshold_ -= 0.0005;
        cont++;
    }     
    

    // for(auto agent: vehicle_team_->auto_team_){
    //     if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
    //         if(!agent->vehicle_.task_path_.empty()){
    //             path_map_[agent->vehicle_.idx_] = agent->vehicle_.PathComputation(tasks_);
    //         }
    //         else{
    //             Path_t<SquareCell*> empty_path_ = {};
    //             path_map_[agent->vehicle_.idx_] = empty_path_;
    //         }
    //         std::cout << "Vehicle " << agent->vehicle_.idx_ << "'s path length is " << path_map_[agent->vehicle_.idx_].size()<< std::endl;
    //         double path_cost = 0.0;
    //         if(!path_map_[agent->vehicle_.idx_].empty()){
    //             for(int kk = 0; kk < path_map_[agent->vehicle_.idx_].size()-1; kk++){
    //                 path_cost += GridGraph::CalcHeuristicUncertain(path_map_[agent->vehicle_.idx_][kk],path_map_[agent->vehicle_.idx_][kk+1]);
    //             }
    //         }
    //         std::cout << "Path cost for vehicle " << agent->vehicle_.idx_ << " is " << path_cost << std::endl;
    //     }
    //     else if (agent->vehicle_.vehicle_type_ == TaskType::MEASURE){
    //         Path_t<SquareCell*> empty_path_ = {};
    //         Vertex_t<SquareCell*>* vrt = agent->vehicle_.local_graph_->GetVertexFromID(agent->vehicle_.pos_);
    //         empty_path_.push_back(vrt->state_);
    //         path_map_[agent->vehicle_.idx_] = empty_path_;
    //     }   
    // }
    // for(auto& agent: vehicle_team_->auto_team_){
    //     GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,{});
    // }
    // // std::cout << "The total number of iteration for IPAS is " << ipas_tt << std::endl;

    // double decrease_rate = (map_entropy_[0] - map_entropy_.back())/map_entropy_[0]; 
	// std::cout << "The Uncertainty of map is decrease by " << decrease_rate << std::endl;
	// // Plot the information of entropy of map, dimension of subregion along iterations
	// GraphVisLCM::TransferDataTrendLCM(map_entropy_, nz_ig_domain_, paths_entropy_);
    std::cout << "The total cost of paths is" << std::endl;
    for(auto mm: cost_path_collection){
        std::cout << mm << ", ";
    }
    std::cout << std::endl;

	return 0;
}