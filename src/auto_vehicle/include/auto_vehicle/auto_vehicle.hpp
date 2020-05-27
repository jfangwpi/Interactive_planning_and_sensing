/*
 * sensors.hpp
 *
 *  Created on: July 19, 2019
 *      Author: jfang
 */

#ifndef AUTOVEHICLE_HPP
#define AUTOVEHICLE_HPP

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>

#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vehicle/agent_impl.hpp"
#include "vehicle/auto_team.hpp"
#include "vehicle/auto_team_impl.hpp"
#include "auto_vehicle/tasks.hpp"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/uncertain_map_lcm_msgs.hpp"


namespace librav{
    const double RW_BENEFIT_ = 1e5;
    const double MEASUREMENT_COST_ = 5;
    const double NOT_CAPABILITY_COST_ = RW_BENEFIT_ - MEASUREMENT_COST_;
    const int64_t MAX_TASKS = 1e10;
    const double EPS_ = 1e-2;
    const double ENTROPY_THRED_ = 0.0243;
    const double PENALTY_ = 200.0;

    typedef struct
    {
        std::vector<Eigen::Matrix<double,1,Eigen::Dynamic>> y_history_;
        std::vector<Eigen::Matrix<int,1,Eigen::Dynamic>> z_history_;
        std::vector<Eigen::Matrix<int,1,Eigen::Dynamic>> iteration_neighb_history_;
    }History;

    class AutoVehicle
    {
        public: 
            AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk);
            // IPAS
            AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk,int64_t num_ss);
            ~AutoVehicle(){};

            int64_t idx_;
            int64_t pos_;
            TaskType vehicle_type_;
            Eigen::Matrix<int,1,Eigen::Dynamic> network_topo_;
            double energy_capacity_;

            int64_t num_vehicles_;
            int64_t num_tasks_;

            /* IPAS */
            int64_t num_sensors_;
            std::shared_ptr<SquareGrid> local_grid_;
            std::shared_ptr<Graph_t<SquareCell*>> local_graph_;
            int64_t ipas_iter_;
            std::vector<std::pair<int64_t,double>> hotspots_; 
            std::map<int64_t,double> nz_ig_zone_;
            std::vector<int64_t> rois_;

            std::vector<std::vector<std::pair<int64_t,double>>> history_hspots_;
            

            /* CBBA */
            Eigen::Matrix<int,1,Eigen::Dynamic> opt_pos_;
        
            std::map<int64_t, double> reward_;

            std::vector<int64_t> task_bundle_;
            std::vector<int64_t> task_path_;

            std::vector<int64_t> h_avai_;

            Eigen::Matrix<double,1,Eigen::Dynamic> cbba_y_;
            Eigen::Matrix<int,1,Eigen::Dynamic> cbba_z_;

            Eigen::Matrix<int,1,Eigen::Dynamic> iteration_neighb_;
            History cbba_history_;

            int64_t cbba_iter_;

            /* CBBA Calculation */
            void InitCBBA(int64_t num_tk);

            // Bundle Construction Phase
            int64_t FindOptIndeTask();
            void AvailableTasks();
            void UpdateReward(TasksSet tasks);
            double PathCostComputation(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> tk_path, int64_t init_pos);
            void BundleAdd(TasksSet tasks);
            
            // Bundle Remove phase
            void BundleRemove();
            void PathRemove();


            /* IPAS */
            void SetLocalMap(std::shared_ptr<SquareGrid> grid);
            void UpdateLocalMap(std::shared_ptr<Graph_t<SquareCell*>> graph,TasksSet tasks);
            void ComputeLocalHotspots(TasksSet tasks);
            Path_t<SquareCell*> PathComputation(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> tk_path, int64_t init_pos);
            std::pair<int64_t,double> MinHotSpot();

            /* Bayesian Optimization */
            std::vector<int64_t> ComputeLocalROIs(TasksSet tasks);
    };

    namespace IPASMeasurement{
        std::shared_ptr<AutoTeam_t<AutoVehicle>> ConstructAutoTeam();
        std::shared_ptr<AutoTeam_t<AutoVehicle>> ConstructAutoTeam(std::vector<AutoVehicle>& teams);
        TasksSet ConstructLTLTasks();
        TasksSet ConstructRandomLTLTasks(int64_t num_tasks, int64_t num_row, int64_t num_col);
        std::shared_ptr<AutoTeam_t<AutoVehicle>> ConstructRandomAutoTeam(int64_t num_actors);

        std::map<int64_t,Path_t<SquareCell*>> GeneratePaths(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team,TasksSet tasks,TaskType task_type);
        bool IPASConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> team,std::map<int64_t,Path_t<SquareCell*>> paths_map);
        int64_t GenerateTruePathsCost(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team,TasksSet tasks,std::shared_ptr<Graph_t<SquareCell*>> true_graph);
        void TransferGraphInfoBayesian(std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<int64_t> ROIs,std::vector<int64_t> nzig,std::vector<int64_t> samples);
        
        void ComputeHotSpots(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team, TasksSet tasks);
        void HotSpotsConsensus(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams);
        bool HotSpotsConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> team);
        TasksSet ConstructMeasurementTasks(std::shared_ptr<AutoTeam_t<AutoVehicle>> team);
        void MergeLocalMap(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams);
        bool MapMergeConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams);

        void MergeHSpots(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams);
        void InformationDrivenHSpots(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams);

    };
}






#endif /* AUTOVEHICLE_HPP */

