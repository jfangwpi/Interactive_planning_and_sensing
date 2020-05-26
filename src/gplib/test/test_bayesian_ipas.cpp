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
    std::vector<double> map_entropy_; // Map entropy
    std::map<int64_t,std::vector<double>> paths_entropy_; // Path entropy for each vehicle
    bool bflag = false;
    
    std::ofstream file("/home/jodie/Workspace/bayesian_opt/IPAS-Bayesian/BayesianResults.json");
    Json::Value root;
    while(flag_IPAS != true){
        bflag = false;
        ipas_tt ++;
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,tasks_);
        std::map<int64_t,Path_t<SquareCell*>> path_ltl_ = IPASMeasurement::GeneratePaths(vehicle_team_,tasks_,TaskType::RESCUE);

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
        IPASMeasurement::ComputeHotSpots(vehicle_team_,tasks_);
        // std::cout << "Before consensus: =============================" <<std::endl;
        // for(auto& itnz: vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_){
        //     std::cout << "Vertex " << itnz.first << ", IG " << itnz.second << std::endl;
        // }
        IPASMeasurement::HotSpotsConsensus(vehicle_team_);
        // Easy for simulation
        IPASMeasurement::MergeHSpots(vehicle_team_);
        // std::cout << "After consensus: =============================" <<std::endl;
        // for(auto& itnz: vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_){
        //     std::cout << "Vertex " << itnz.first << ", IG " << itnz.second << std::endl;
        // }
        // std::cout << "============================================" << std::endl;


        // Transfer the nz_ig_zone and probability complexity to python to estimate the maximum IG cell
        int64_t num_hspts = vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_.size();
        std::vector<int64_t> nzig_vt;
        std::map<int64_t,double>::iterator it_nzig = vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_.begin();
        for(;it_nzig!=vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_.end();it_nzig++){
            // std::cout << "The vertex " << it_nzig->first << " , the ig is " << it_nzig->second <<std::endl;
            nzig_vt.push_back(it_nzig->first);
        }
        int64_t num_samples = round(num_hspts*0.02);
        root["NZIG"] = double(num_hspts);
        // Select the training data randomly
        std::vector<int64_t> tdata;
        while (tdata.size() < num_samples){
            int itg = rand() % num_hspts;
            std::vector<int64_t>::iterator it_trained = std::find(tdata.begin(),tdata.end(),nzig_vt[itg]);
            if(it_trained==tdata.end()){
                tdata.push_back(nzig_vt[itg]);
            }
        }

        // std::cout << "The number of training data is " << num_samples <<std::endl;
        // for (auto& item: tdata){
        //     std::cout << item << ", ";
        // }
        // std::cout << std::endl;
        std::vector<int64_t> rois = vehicle_team_->auto_team_[0]->vehicle_.rois_;
        // std::cout << "The ROI is " <<std::endl;
        // for (auto &r: rois){
        //     std::cout << r << ", ";
        // }
        // std::cout << std::endl;
        GraphVisLCM::TransferBayesianInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,rois,nzig_vt,tdata); 
        std::vector<Vertex_t<SquareCell*>*> all_verts = vehicle_team_->auto_team_[0]->vehicle_.local_graph_->GetAllVertex();
        // std::cout << "The length of NZIG is " << nzig_vt.size() << std::endl;
        // for(auto vt: all_verts){
        //     std::vector<int64_t>::iterator it = std::find(nzig_vt.begin(),nzig_vt.end(),vt->state_->id_);
        //     if(it != nzig_vt.end()){
        //         std::cout << "Vertex " << vt->state_->id_ << " prob is " << vt->state_->p_ << std::endl;
        //     }
        // }
        

        while(bflag==false && 0 == lcm.handle()){bflag=bayesian_data_.returnFlag();}
        std::cout << "JUMP OUT OF WHILE LOOP"<<std::endl;

        std::cout << "The HotSpots for NORMAL IPAS is: " <<std::endl;
        double sum_ipas_ = 0.0;
        for(auto &it:vehicle_team_->auto_team_[0]->vehicle_.hotspots_){
            sum_ipas_ += it.second;
            std::cout << "Vertex: " << it.first << ", IG: " << it.second << std::endl;
            // Vertex_t<SquareCell*>* vt = vehicle_team_->auto_team_[0]->vehicle_.local_graph_->GetVertexFromID(it.first);
            // std::vector<Vertex_t<SquareCell*>*> neighbors = vt->GetNeighbours();
            // std::vector<double> nb_ig;
            // for(auto& nb: neighbors){
            //     double p = nb->state_->p_;
            //     double q = 1.0-p;
            //     if (p == 0.0 || p == 1.0){
            //         nb_ig.push_back(0.0);
            //     }
            //     else{
            //         double ig_p = -p*log(p) - q*log(q);
            //         nb_ig.push_back(ig_p);
            //     }
            // }
            // for(auto& b: nb_ig){
            //     std::cout << b << ", ";
            // }
            // std::cout << std::endl;
        }
        
        
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

        std::cout << "The HotSpot for BayesianOPT is: " <<std::endl;
        double sum_bayesian_ = 0.0;
        for(auto&it: consensus_hspts_){
            sum_bayesian_+= it.second;
            std::cout << "Vertex: " << it.first << ", IG: " << it.second <<std::endl;
        }
        // Write the result to Json.
        root["Iteration"] = double(ipas_tt);
        root["IPAS"] = sum_ipas_;
        root["Bayesian"] = sum_bayesian_;
        int sidx = 1;
        for (auto& spts: sensor_pos){
            std::string sensor_idx_ = "Sensor " + std::to_string(sidx);
            root[sensor_idx_] = double(spts);
            sidx++;
        }
        file << root;
         

        std::cout << "IPAS sum IG is " << sum_ipas_ << ". Bayeisan sum IG is "<< sum_bayesian_ << std::endl;
        std::cout << "=============================================="<<std::endl;
        std::cout << "=============================================="<<std::endl;
        std::cout << "=============================================="<<std::endl;
        std::cout << "=============================================="<<std::endl;

        // // Easy for simulation
        // IPASMeasurement::MergeHSpots(vehicle_team_);

        // nz_ig_domain_.push_back(vehicle_team_->auto_team_[0]->vehicle_.nz_ig_zone_.size());

        TasksSet sensing_tasks_ = IPASMeasurement::ConstructMeasurementTasks(vehicle_team_);
        CBBA::ConsensusBasedBundleAlgorithm(vehicle_team_,sensing_tasks_);

        std::map<int64_t,Path_t<SquareCell*>> path_sensing_ = IPASMeasurement::GeneratePaths(vehicle_team_,sensing_tasks_,TaskType::MEASURE);

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
        // GraphVisLCM::TransferInfoLCM(vehicle_team_->auto_team_[0]->vehicle_.local_graph_,path_map_,hspts,local_hspots);
    
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