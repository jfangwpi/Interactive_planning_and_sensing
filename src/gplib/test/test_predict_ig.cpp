#include <cmath>
#include <iostream>
#include <gtest/gtest.h>
#include <vector>

// user
#include "map/square_grid.hpp"
#include "vis/graph_vis_lcm.hpp"
#include "vis/graph_vis.hpp"

#include "cbba/cbba_agent.hpp"
#include "cbba/cbba_task.hpp"
#include "gplib/rbf_kernel.hpp"
#include "gplib/log_likelihood.hpp"

using namespace librav;
using namespace cv;

int main(int argc, char** argv)
{
    // Define the parameters required by the Gaussian process
    const size_t kDimension = 2;
    const size_t NumTrainingPoints = 6;
    const size_t NumTests = 20;
    const double kEpsilon = 0.01;
    const double kMaxError = 1e-8;
    const double kNoiseVariance = 0.0;
    const double alpha = 0.001;
    
    // Generate the map
    // Dimension of grid 
	int64_t num_row = 10;
    int64_t num_col = 10;
    std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(num_row, num_col,1);

	for (int i = 0; i < num_row * num_col; i++){
		grid->SetCellProbability(i,0.5);
	}
    
    grid->SetCellProbability(1,0.059);
    grid->SetCellProbability(2,0.0);
    grid->SetCellProbability(3,0.059);
    grid->SetCellProbability(8,0.2);
    grid->SetCellProbability(10,0.2);
    grid->SetCellProbability(11,0.0);
    grid->SetCellProbability(12,0.015);
    grid->SetCellProbability(13,0.0);
    grid->SetCellProbability(14,0.2);
    grid->SetCellProbability(15,0.8);
    grid->SetCellProbability(17,0.059);
    grid->SetCellProbability(18,0.0);
    grid->SetCellProbability(19,0.2);
    grid->SetCellProbability(20,0.941);
    grid->SetCellProbability(21,1.0);
    grid->SetCellProbability(22,0.059);
    grid->SetCellProbability(23,0.2);
    grid->SetCellProbability(24,0.059);
    grid->SetCellProbability(25,1.0);
    grid->SetCellProbability(26,0.941);
    grid->SetCellProbability(27,0.0);
    grid->SetCellProbability(28,0.015);
    grid->SetCellProbability(30,1.0);
    grid->SetCellProbability(31,1.0);
    grid->SetCellProbability(32,0.0);
    grid->SetCellProbability(33,0.015);
    grid->SetCellProbability(34,0.0);
    grid->SetCellProbability(35,0.941);
    grid->SetCellProbability(37,0.015);
    grid->SetCellProbability(38,0.0);
    grid->SetCellProbability(39,0.2);
    grid->SetCellProbability(40,0.941);
    grid->SetCellProbability(41,1.0);
    grid->SetCellProbability(42,0.015);
    grid->SetCellProbability(43,0.0);
    grid->SetCellProbability(44,0.059);
    grid->SetCellProbability(45,0.2);
    grid->SetCellProbability(46,0.2);
    grid->SetCellProbability(47,0.0);
    grid->SetCellProbability(48,0.059);
    grid->SetCellProbability(50,0.0);
    grid->SetCellProbability(51,0.059);
    grid->SetCellProbability(53,0.059);
    grid->SetCellProbability(54,0.2);
    grid->SetCellProbability(55,0.0);
    grid->SetCellProbability(56,0.059);
    grid->SetCellProbability(57,0.2);
    grid->SetCellProbability(60,0.8);
    grid->SetCellProbability(61,1.0);
    grid->SetCellProbability(62,0.941);
    grid->SetCellProbability(63,1.0);
    grid->SetCellProbability(64,0.8);
    grid->SetCellProbability(65,0.015);
    grid->SetCellProbability(66,1.0);
    grid->SetCellProbability(67,0.8);
    grid->SetCellProbability(70,0.8);
    grid->SetCellProbability(71,0.941);
    grid->SetCellProbability(73,0.8);
    grid->SetCellProbability(74,0.941);
    grid->SetCellProbability(75,0.0);
    grid->SetCellProbability(76,0.941);
    grid->SetCellProbability(80,0.0);
    grid->SetCellProbability(81,0.0);
    grid->SetCellProbability(82,0.2);
    grid->SetCellProbability(83,0.2);
    grid->SetCellProbability(84,0.0);
    grid->SetCellProbability(85,0.059);
    grid->SetCellProbability(90,0.2);
    grid->SetCellProbability(91,0.2);
    grid->SetCellProbability(94,0.2);


    // Define the tasks
    LTLFormula Global_LTL;
	LTLDecomposition::GlobalLTLDecomposition(Global_LTL);
	TasksList tasks(Global_LTL);
    tasks.GetAllTasks();
    // Set Task for grid
    for (int i = 0; i < Global_LTL.task_info.size();i++){
		grid->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);	
		grid->SetCellOccupancy(Global_LTL.task_info[i].pos_, OccupancyType::INTERESTED);
	}
    
    // Agent Information
    int agentID = 0;
    int agentPos = 0;
    int numInTask = 3;
    int numDeTask = 0;
    int numAgent = 1;
    Agent agent(agentID,agentPos,numInTask,numDeTask,numAgent);
    agent.cbba_path_ = {0,1,2};
    // agent.cbba_path_ = {0};

    // Set Agent for grid
    grid->SetCellOccupancy(agent.init_pos_, OccupancyType::FREE);
    // Construct the graph
    std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, false, true);

    // Compute the path based on the cbba path
    std::map<int64_t,Path_t<SquareCell*>> paths;
    Path_t<SquareCell*> path = GridGraph::PathComputationAStar(tasks, graph, agent.cbba_path_, agent.init_pos_);
	paths[agent.idx_] = path;
	// GraphVisLCM::TransferInfoLCM(graph, paths, {});

    std::vector<Agent> agents = {agent};
    std::vector<Path_t<SquareCell*>> path_collections = TaskAssignment::PathsDecomposition(tasks,agents,paths);

    // Determine the subregion with given paths
    std::vector<int64_t> sub_domain = {};
    std::vector<int64_t> domain = GridGraph::SubRegionFromPaths(path_collections, grid, graph);
    for(auto& cell: domain){
        std::vector<int64_t>::iterator it_v = std::find(sub_domain.begin(), sub_domain.end(), cell);
        if(it_v == sub_domain.end()){
            sub_domain.push_back(cell);
        }
    }

    std::cout << "The sub region is " << std::endl;
    for (auto &mm: sub_domain){
        std::cout << mm << ", ";
    }
    std::cout << std::endl;

    // Determine the set of vertices which has non zero information gain
    std::vector<int64_t> vertex_ig;
    for(auto& c_v: sub_domain){
        Vertex_t<SquareCell*>* vert = graph->GetVertexFromID(c_v);
        std::vector<int64_t>::iterator it1 = std::find(vertex_ig.begin(), vertex_ig.end(), vert->state_->id_);
        if(it1 == vertex_ig.end()){
            vertex_ig.push_back(vert->state_->id_);
        }
        std::vector<SquareCell *> neig_verts= grid->GetNeighbors(vert->state_->id_,false);
        std::vector<int64_t> neighbors;
        for(auto& vv: neig_verts){
            neighbors.push_back(vv->id_);
        }
        // std::vector<int64_t> neighbors = vert->GetNeighbourIDs();
        for(auto& neighb_id: neighbors){
            std::vector<int64_t>::iterator it = std::find(vertex_ig.begin(), vertex_ig.end(), neighb_id);
            if(it == vertex_ig.end()){
                vertex_ig.push_back(neighb_id);
            }
        }
    }
    
    // Update the information gain
    std::cout << "The cell with none zero ig is "<< std::endl;
    for(int cell = 0; cell < grid->num_row_ * grid->num_col_; cell++){
        std::vector<int64_t>::iterator it = std::find(vertex_ig.begin(), vertex_ig.end(), cell);
        Vertex_t<SquareCell*>* current_v = graph->GetVertexFromID(cell);
        if(it == vertex_ig.end()){
            current_v->state_->ig_ = 0.0;
        }
        else{
            std::cout << current_v->state_->id_ << ", ";
            current_v->state_->ComputeIG(grid, graph, sub_domain);
        }	
    }
    std::cout << std::endl;

    GraphVisLCM::TransferInfoLCM(graph, paths, {});
    return 0;
}