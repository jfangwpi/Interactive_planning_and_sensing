#ifndef AUTO_TEAM_HPP
#define AUTO_TEAM_HPP

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "vehicle/agent.hpp"

namespace librav{

    template <typename AgentType>
    class Vehicle_t;
   
    template <typename AgentType>
    class AutoTeam_t
    {
        public:
            template <class T = AgentType>
            AutoTeam_t(){};
            ~AutoTeam_t(){};

            typedef Vehicle_t<AgentType> VehicleType;

            // General attributes
            std::vector<VehicleType *> auto_team_;
            int64_t num_vehicles_;
            int64_t num_tasks_;

            std::vector<VehicleType *> rescue_team_;
            std::vector<VehicleType *> measure_team_;

            // Return the vehicle with same id
            VehicleType* GetVehicleFromID(int64_t id);
    };
}


#endif /* AUTO_TEAM_HPP */