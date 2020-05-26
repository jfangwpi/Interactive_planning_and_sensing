#ifndef CBBA_HPP
#define CBBA_HPP


// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "auto_vehicle/auto_vehicle.hpp"
#include "vehicle/agent_impl.hpp"
#include "vehicle/auto_team.hpp"
#include "vehicle/auto_team_impl.hpp"


namespace librav{
    namespace CBBA
    {   

        template<typename VehicleType>
        void BundleConstruction(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team,TasksSet tasks);

        template<typename VehicleType>
        void Consensus(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team);
        
        template<typename VehicleType>
        bool CheckConvergence(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team);

        template<typename VehicleType>
        void ConsensusBasedBundleAlgorithm(std::shared_ptr<AutoTeam_t<VehicleType>> vehicle_team, TasksSet tasks);
    };
}


#endif /* CBBA_HPP */