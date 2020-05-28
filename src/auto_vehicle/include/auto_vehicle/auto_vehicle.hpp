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

        // Required by synchronization algorithm
        std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> assignment_matrix_history_;
        std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> winning_bids_history_;
    }History;

    class AutoVehicle
    {
        public: 
            AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk);
            // IPAS
            AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk,int64_t num_ss);
            // Collaborative 
            AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t itk, int64_t ctk, int64_t num_ss);
            ~AutoVehicle(){};

            int64_t idx_;
            int64_t pos_;
            TaskType vehicle_type_;
            Eigen::Matrix<int,1,Eigen::Dynamic> network_topo_;
            double energy_capacity_;

            int64_t num_vehicles_;
            int64_t num_tasks_;

            int64_t num_independent_tasks_;
            int64_t num_collaborative_tasks_;

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

            /*** Information required by synchronization algorithm ***/
            // Required by synchronization algorithm
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> syn_c_;
            // The set of available dependent tasks
            std::vector<int> h_avai_de_;
            // Assignment matrix
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> assignment_matrix_;
            // Reward matrix 
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> winning_bids_matrix_;
            // Total waiting time required by the vehicle 
            double waiting_t_;
            // Insert position threshold for dependent tasks
            int last_pos_dependent_;



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

            /* Synchronization Algorithm */
            // Initialize syn 
            void InitSyn(int64_t num_tk);
            // Update the syn_c and the optimal insert position 
            void update_reward_dependent(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph);
            // Find the available dependent task based on current task path
            void available_dep_tasks_finder(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph);
            // Find the desired dependent task from the list of available tasks
            int desired_dep_task_finder();
            // Add dependent task into task path
            void bundle_add_dependent(TasksSet task, const std::shared_ptr<Graph_t<SquareCell*>> graph);
            // Update the optimal group for the given task
            void optimal_group_finder(Task task);
            // Find the vehicles based on the bids
            std::vector<int> FindVehicleFrombid(Task task, std::vector<double> winners_bid);
            // Find the winners for the given task
            std::vector<int> winners_finder(Task task);
            std::vector<double> winning_bids_finder(Task task);
            // Find the number of vehicle 
            int winners_count(Task task);
            // Remove outbid dependent task from task path
            void path_remove_dependent(TasksSet tasks);
            // Remove outbid dependent task from task bundle
            void bundle_remove_dependent();
            // Find the winning bids for the given task
            std::vector<double> winning_bids_finder(TasksSet task);
            // Compute the waiting time required by all dependent tasks
            void UpdateWaitingTime(std::shared_ptr<Graph_t<SquareCell*>> graph, TasksSet tasks);



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

    namespace CollaborativeAlgorithm{

        /*** Functions for Synchronization algorithm ***/
        // Insert dependent tasks into task bundle/path and update iteration
        void BundleAdd(TasksSet tasks,std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team);
        // Remove dependent tasks from task path/bundle
        void PathRemove(TasksSet tasks,std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team);
        // Communicate with neighbors
        void communicate_dependent(TasksSet tasks,std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team);
        // Find the K closest reward based on current reward
        std::vector<double> KClosestFinder(std::vector<double> ordered_bids, int x, int k, int n);
        int findCrossOver(std::vector<double> ordered_bids, int low, int high, int x);
        double maximum_bid_finder(std::vector<double> winning_bids);
        double minimum_bid_finder(std::vector<double> winning_bids);
        // Check whether the convergence of synchronization algorithm is achieved
        bool success_checker_dependent(TasksSet tasks,std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team);
        /*** Compute the waiting time required by vehicle ***/
        double WaitingTimeCalculation(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<Agent>& agents, TasksList tasks);
        int FindNumAppearance(std::vector<int> list, int target);
        std::vector<int> FindWinners(std::vector<Agent> agents, int task_idx);
        // Find the longest path for all dependent tasks
        double MaximumRewardCalculation(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, TasksList tasks);
        // Compute certain length of path by considerring the waiting time
        double PathLengthCalculationWithWaiting(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, TasksList tasks);

        void SynchronizationAlgorithm(TasksSet tasks, std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team);

    };
}






#endif /* AUTOVEHICLE_HPP */

