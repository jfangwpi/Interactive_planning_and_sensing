#ifndef AGENT_HPP
#define AGENT_HPP

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>


namespace librav{
    enum class TaskType{
        MEASURE,
        RESCUE
    };
    
    template <typename AgentType>
    class Vehicle_t
    {
        using VehicleType = Vehicle_t<AgentType>;
        public:
            template <class T = VehicleType>
            Vehicle_t(T vehicle);

            ~Vehicle_t(){};

            // General attributes
            AgentType vehicle_;

            int64_t vehicle_idx_;
            int64_t vehicle_pos_;

            double vehicle_energy_capacity_;

            TaskType vehicle_type_;
            Eigen::Matrix<int,1,Eigen::Dynamic> network_topo_;
            std::vector<int64_t> neighbors_;

            // Find neighbors
            std::vector<int64_t> GetNeighborsIDs();

            std::map<int64_t,double> GetVehicleReward();
    };
}


#endif /* AGENT_HPP */