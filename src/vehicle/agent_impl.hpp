#ifndef AGENT_IMPL_HPP
#define AGENT_IMPL_HPP

#include <cstdint>
#include <algorithm>
#include "vehicle/agent.hpp"

namespace librav{

    template <typename AgentType>
    template <class T>
    Vehicle_t<AgentType>::Vehicle_t(T vehicle): 
        vehicle_(vehicle),vehicle_idx_(vehicle.idx_),vehicle_pos_(vehicle.pos_), 
        network_topo_(vehicle.network_topo_),vehicle_energy_capacity_(vehicle.energy_capacity_)
        {   
            for(int64_t ii = 0; ii < network_topo_.size(); ii++){
                if (ii != vehicle_idx_ && network_topo_(ii) == 1){
                    neighbors_.push_back(ii);
                }
            }
        };


    template<typename AgentType>
    std::vector<int64_t> Vehicle_t<AgentType>::GetNeighborsIDs(){
        std::vector<int64_t> neighbors_ = {};
        for(int64_t ii = 0; ii < network_topo_.size(); ii++){
            if(ii != vehicle_idx_ && network_topo_(ii) == 1){
                neighbors_.push_back(ii);
            }
        }
        return neighbors_;
    };

    template<typename AgentType>
    std::map<int64_t,double> Vehicle_t<AgentType>::GetVehicleReward(){
        std::cout << "Reward is: " << std::endl;
        for(auto& tt: vehicle_.reward_){
            std::cout << tt.first << ": " << tt.second << std::endl;
        }
        return vehicle_.reward_;
    }
}



#endif /* AGENT_IMPL_HPP */