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
#include "json/json.h"

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
    bool bflag = false;
    std::map<int64_t,Path_t<SquareCell*>> path_map_;
    while(flag_IPAS != true){
        bflag = false;
        ipas_tt ++;
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);
        std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);
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
        if(flag_IPAS == true){break;std::cout << "The convergence of IPAS is achieved." <<std::endl;}
        //===============================================================================================//
        //============================================= IPAS ============================================//
        //===============================================================================================// 
        // Measurement
        IPASMeasurement::ComputeHotSpots(vehicle_team_,tasks_);
        IPASMeasurement::HotSpotsConsensus(vehicle_team_);
        // Easy for simulation
        IPASMeasurement::MergeHSpots(vehicle_team_);

        // Transfer the nz_ig_zone and probability complexity to python to estimate the maximum IG cell
        // Select the training data randomly
        int64_t num_hspts = vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_.size();
        std::vector<int64_t> nzig_vt;
        std::map<int64_t,double>::iterator it_nzig = vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_.begin();
        for(;it_nzig!=vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_.end();it_nzig++){
            nzig_vt.push_back(it_nzig->first);
        }
        int64_t num_samples = round(num_hspts*0.02);
        std::vector<int64_t> tdata;
        while (tdata.size() < num_samples){
            int itg = rand() % num_hspts;
            std::vector<int64_t>::iterator it_trained = std::find(tdata.begin(),tdata.end(),nzig_vt[itg]);
            if(it_trained==tdata.end()){
                tdata.push_back(nzig_vt[itg]);
            }
        }

        std::vector<int64_t> rois = vehicle_team_->auto_team_[0]->vehicle_.rois_;
        // Apply Bayesian optimization to select the sesnors positions.
        GraphVisLCM::TransferBayesianInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,rois,nzig_vt,tdata); 
        std::vector<Vertex_t<SquareCell*>*> all_verts = vehicle_team_->auto_team_[0]->vehicle_.local_graph_->GetAllVertex();
        while(bflag==false && 0 == lcm.handle()){bflag=bayesian_data_.returnFlag();}

        // Update the sensing positions from Bayesian Optimization
        std::vector<int64_t> sensor_pos = bayesian_data_.returnSensorsPos();
        std::vector<std::pair<int64_t,double>> consensus_hspts_={};
        for (auto& spts: sensor_pos){
            std::pair<int64_t,double> ig_pair = std::make_pair(spts, vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_[spts]);
            consensus_hspts_.push_back(ig_pair);
        }
        for(auto &agent: vehicle_team_->auto_team_){
            agent->vehicle_.hotspots_ = consensus_hspts_;
            agent->vehicle_.history_hspots_.push_back(agent->vehicle_.hotspots_);
        }

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
            }
        }


        std::vector<int64_t> hspts;
        for(auto cc: vehicle_team_->auto_team_[0]->vehicle_.hotspots_){
            hspts.push_back(cc.first);
        }
        GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,hspts,{});
    
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