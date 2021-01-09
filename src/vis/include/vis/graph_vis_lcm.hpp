#ifndef GRAPH_VIS_LCM_HPP
#define GRAPH_VIS_LCM_HPP

#include <vector>
#include <cmath>
#include <stdlib.h>

#include "map/square_grid.hpp"
#include "graph/graph.hpp"
#include "graph/vertex.hpp"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/uncertain_map_lcm_msgs.hpp"


namespace librav {

namespace GraphVisLCM
{
	void TransferInfoLCM(std::shared_ptr<Graph_t<SquareCell *>> graph, std::map<int64_t, Path_t<SquareCell*>> paths, std::vector<int64_t> sensors_pos,std::vector<int64_t> local_hspts={});
	void TransferDataTrendLCM(std::vector<double> entropy, std::vector<int64_t> range, std::map<int64_t, std::vector<double>> entropy_paths);
	void TransferGraphLCM(std::shared_ptr<Graph_t<SquareCell *>> graph,std::vector<int64_t> training_data);
	void TransferBayesianInfoLCM(std::shared_ptr<Graph_t<SquareCell *>> graph, std::vector<int64_t> ROIs, std::vector<int64_t> nz_ig,std::vector<int64_t> tdata);

	
	void TransferRandomInfoLCM(std::shared_ptr<Graph_t<SquareCell *>> graph, std::map<int64_t, Path_t<SquareCell*>> paths, std::vector<int64_t> sensors_pos, TasksList tasks, std::vector<Agent> agents);
};


}

#endif /* GRAPH_VIS_LCM_HPP */
