/*
 * test_bayesian_ipas.cpp
 * Use Baysian optimization to determine the hot spots.
 * Created on: March 05,2020
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

class BayesianFlag
{
    public: 
        bool flag_ = false;
        std::vector<int64_t> sensors_pos_;
        BayesianFlag(){flag_=false;};
        ~BayesianFlag(){};

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const graph_data::bayesian_opt_data* msg)
                {
                    flag_ = msg->bayesian_opt_flag_;
                    sensors_pos_ = msg->sensor_pos_;
                    std::cout << "Received the data from python" <<std::endl;
                    std::cout << "The sensor position from bayesian opt is " << std::endl;
                    for(auto &s: sensors_pos_){
                        std::cout << s << ", ";
                    }
                    std::cout << std::endl;
                }
        bool returnFlag(){return flag_;}
        std::vector<int64_t> returnSensorsPos(){return sensors_pos_;}

};

std::vector<int> readSensors(int64_t t){
    std::cout << "Read the sensor at iteration " << t << std::endl;
    ConfigReader config_reader("../../src/config/sensor-info.ini");
    if (config_reader.CheckError()){
        std::cout << "Reading config file failed." << std::endl;
    }
    std::string hspt_name = "hspt-"+std::to_string(t);
    std::vector<int> hspt = config_reader.GetVectorInt(hspt_name,{});
    std::cout << "The sensors read from file is: ";
    for (auto s: hspt){
        std::cout << s << ", ";
    }
    std::cout << std::endl;
    return hspt;
}

int main(int argc, char** argv )
{   
    //===============================================================================================//
    //===========================================Task - Agent========================================//
    //===============================================================================================//
    // Parameters
    int64_t num_row = 30;
    int64_t num_col = 30;

    // Initialize the bayesian optimization
    srand(time(NULL));
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    BayesianFlag bayesian_data_;
    lcm.subscribe("BayesianOpt",&BayesianFlag::handleMessage,&bayesian_data_);



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
    } // Initialize the local grid map for each auto vehicle
    bool flag_IPAS = false;
    int64_t ipas_tt = 0;// Iterations required by IPAS
    std::vector<double> map_entropy_; // Map entropy
    std::map<int64_t,std::vector<double>> paths_entropy_; // Path entropy for each vehicle
    std::map<int64_t,Path_t<SquareCell*>> path_map_;
    bool bflag = false;
    while(flag_IPAS != true){
        bflag = false;
        ipas_tt ++;
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);
        std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);
        for (auto agent: vehicle_team_->auto_team_){
            if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
                path_map_[agent->vehicle_.idx_] = path_ltl_[agent->vehicle_.idx_];
            }
        }
        //=============================================================//
        //============================== DEBUG ========================//
        //=============================================================//
        std::cout << "Convergence for task assignment is achieved." << std::endl;
        // for(auto p: path_ltl_){
        //     std::cout << "The path for vehicle " << p.first << " is: ";
        //     for(auto v: p.second){
        //         std::cout << v->id_ <<", ";
        //     }
        //     std::cout << std::endl;
        // }
        //=============================================================//
        //=============================================================//
        //=============================================================//

        //=============================================================//
        //============================== RECORD =======================//
        //=============================================================//
        // Record the map entropy for each vehicle
        map_entropy_.push_back(GridGraph::EntropyMap(vehicle_team_->auto_team_[0]->vehicle_.local_graph_));
        // Record the path entropy for each vehicle
        for(auto p: path_ltl_){
            double en_p = GridGraph::EntropyPath(p.second, vehicle_team_->auto_team_[p.first]->vehicle_.local_graph_);
            paths_entropy_[p.first].push_back(en_p);
        }
        //=============================================================//
        //=============================================================//
        //=============================================================//
        
        // Check the termination condition
        flag_IPAS = IPASMeasurement::IPASConvergence(vehicle_team_,path_ltl_);
        if(flag_IPAS == true){break;std::cout << "The convergence of IPAS is achieved." <<std::endl;}
        //===============================================================================================//
        //============================================= IPAS ============================================//
        //===============================================================================================// 
        // Measurement
        std::cout << "Iteration for IPAS: " << ipas_tt <<std::endl;
        // IPASMeasurement::ComputeHotSpots(vehicle_team_,tasks_);
        // Standard information driven 
        // Measurement
        IPASMeasurement::InformationDrivenHSpots(vehicle_team_);
        // std::cout << "Before consensus: =============================" <<std::endl;
        // for(auto& itnz: vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_){
        //     std::cout << "Vertex " << itnz.first << ", IG " << itnz.second << std::endl;
        // }
        IPASMeasurement::HotSpotsConsensus(vehicle_team_);
        // Easy for simulation
        IPASMeasurement::MergeHSpots(vehicle_team_);

        std::vector<int> sensor_pos = readSensors(ipas_tt);

        std::vector<std::pair<int64_t,double>> consensus_hspts_={};
        for (auto& spts: sensor_pos){
            std::pair<int64_t,double> ig_pair = std::make_pair(spts, vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_[spts]);
            consensus_hspts_.push_back(ig_pair);
        }
        for(auto &agent: vehicle_team_->auto_team_){
            agent->vehicle_.hotspots_ = consensus_hspts_;
            agent->vehicle_.history_hspots_.push_back(agent->vehicle_.hotspots_);
        }

        // // Easy for simulation
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

        for(auto&agent:vehicle_team_->auto_team_){
            if(!agent->vehicle_.task_path_.empty()){
                int64_t ss_task = sensing_tasks_.GetTaskPosFromID(agent->vehicle_.task_path_.back());
                agent->vehicle_.pos_ = ss_task;
                // std::cout << "The initial pos for vehicle " << agent->vehicle_.idx_ << " is updated to " << agent->vehicle_.pos_ << std::endl;
            }
        }

        // for(auto& agent: vehicle_team_->auto_team_){
        std::map<int64_t,Path_t<SquareCell*>> map_path_ = {};
        std::vector<int64_t> hspts;
        for(auto cc: vehicle_team_->auto_team_[0]->vehicle_.hotspots_){
            hspts.push_back(cc.first);
        }

        
        GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,hspts,{});
    
        // Update the map
        for(auto&agent:vehicle_team_->auto_team_){
            agent->vehicle_.UpdateLocalMap(true_graph,sensing_tasks_);
        }
        // std::cout << "Measurements are taken and map is updated." << std::endl;

        IPASMeasurement::MergeLocalMap(vehicle_team_);

    }
    std::cout << "The total number of iteration for IPAS is " << ipas_tt << std::endl;

	return 0;
}