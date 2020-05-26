// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <cmath>
#include <tuple>
#include <string>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "map/square_grid.hpp"
#include "vis/graph_vis_lcm.hpp"
#include "auto_vehicle/auto_vehicle.hpp"
#include "vehicle/auto_team.hpp"
#include "vehicle/agent.hpp"
#include "task_assignment/cbba_impl.hpp"
using namespace librav;

class BayesianFlag
{
    public: 
        bool flag_;
        BayesianFlag(){flag_=false;};
        ~BayesianFlag(){};

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const graph_data::prediction_data* msg)
                {
                    flag_ = msg->bayesian_opt_flag;
                    std::cout << "Received the data from python" <<std::endl;
                    std::cout << "The flag data is " << flag_ << std::endl;
                }

};

int main(int argc, char** argv )
{
	// Dimension of grid 
	// int64_t num_row = 20;
    // int64_t num_col = 20;
    int64_t num_row = 10;
    int64_t num_col = 10;

	srand(time(NULL));
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    BayesianFlag bayesian_data_;
    lcm.subscribe("BayesianChannel",&BayesianFlag::handleMessage,&bayesian_data_);
    /*Construct auto vehicle team */
    Eigen::MatrixXi comm = Eigen::MatrixXi::Ones(1,1);
    AutoVehicle drone = AutoVehicle(0, 90, 1, comm, TaskType::RESCUE,1);
    std::vector<AutoVehicle> drones = {drone};
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team_ = IPASMeasurement::ConstructAutoTeam(drones);

    /*Define a Task*/
    Task task = Task(0,1,{9},TaskType::RESCUE,1);
    std::vector<Task> tasks_set = {task};
    TasksSet tasks = TasksSet(tasks_set);
    
	/*Construct Grid*/
	std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(num_row, num_col, 1);
    
    std::vector<double> p_case = {0.2,0.2,0.3,0.3,0.4,0.5,0.5,0.4,0.8,0.8};
	for (int i = 0; i < num_row * num_col; i++){
        int itg = rand() % 10;
        grid->SetCellProbability(i,p_case[itg]);
        // grid->SetCellProbability(i,0.5);
	}
    // 10*10 map
    // grid->SetCellProbability(5,0.8);
    // grid->SetCellProbability(15,0.0);
    // grid->SetCellProbability(14,0.2);
    // grid->SetCellProbability(16,0.8);
    // grid->SetCellProbability(25,0.2);
    // grid->SetCellProbability(34,0.2);
    // grid->SetCellProbability(44,0.0);
    // grid->SetCellProbability(43,0.8);
    // grid->SetCellProbability(45,0.2);
    // grid->SetCellProbability(54,0.2);

    // 20*20 map
    // grid->SetCellProbability(10,0.945742);
    // grid->SetCellProbability(11,0.945742);
    // grid->SetCellProbability(30,0.945742);
    // grid->SetCellProbability(31,0.945742);

    // grid->SetCellProbability(50,0.0);
    // grid->SetCellProbability(51,0.0);
    // grid->SetCellProbability(70,0.0);
    // grid->SetCellProbability(71,0.0);

    // grid->SetCellProbability(48,0.668741);
    // grid->SetCellProbability(49,0.668741);
    // grid->SetCellProbability(68,0.668741);
    // grid->SetCellProbability(69,0.668741);

    // grid->SetCellProbability(52,0.945742);
    // grid->SetCellProbability(53,0.945742);
    // grid->SetCellProbability(72,0.945742);
    // grid->SetCellProbability(73,0.945742);

    // grid->SetCellProbability(90,0.668741);
    // grid->SetCellProbability(91,0.668741);
    // grid->SetCellProbability(110,0.668741);
    // grid->SetCellProbability(111,0.668741);

    // grid->SetCellProbability(128,0.668741);
    // grid->SetCellProbability(129,0.668741);
    // grid->SetCellProbability(148,0.668741);
    // grid->SetCellProbability(149,0.668741);

    // grid->SetCellProbability(168,0.0);
    // grid->SetCellProbability(169,0.0);
    // grid->SetCellProbability(188,0.0);
    // grid->SetCellProbability(189,0.0);

    // grid->SetCellProbability(166,0.945742);
    // grid->SetCellProbability(167,0.945742);
    // grid->SetCellProbability(186,0.945742);
    // grid->SetCellProbability(187,0.945742);

    // grid->SetCellProbability(170,0.668741);
    // grid->SetCellProbability(171,0.668741);
    // grid->SetCellProbability(190,0.668741);
    // grid->SetCellProbability(191,0.668741);

    // grid->SetCellProbability(208,0.668741);
    // grid->SetCellProbability(209,0.668741);
    // grid->SetCellProbability(228,0.668741);
    // grid->SetCellProbability(229,0.668741);

    // Set Grid for agent pos and task pos
    grid->SetCellProbability(task.pos_[0],0.0);
    grid->SetCellProbability(vehicle_team_->auto_team_[0]->vehicle_.pos_,0.0);
    
	std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, false, true);
    /*Compute the path */
    vehicle_team_->auto_team_[0]->vehicle_.task_path_={0};
    vehicle_team_->auto_team_[0]->vehicle_.task_bundle_={0};
    Path_t<SquareCell*> path = vehicle_team_->auto_team_[0]->vehicle_.PathComputation(tasks,vehicle_team_->auto_team_[0]->vehicle_.local_graph_,vehicle_team_->auto_team_[0]->vehicle_.task_path_,vehicle_team_->auto_team_[0]->vehicle_.pos_);
    // std::vector<int64_t> sub_domain = GridGraph::SubRegionFromPaths({path},grid,graph);
    std::vector<int64_t> sub_domain;
    // std::vector<std::vector<int>> boundary = {{8,20},{28,40},{46,60},{66,80},{86,92},{106,112},
    //     {128,132},{148,152},{168,176},{188,196},{204,216},{224,236},{244,252},{264,272},{280,292},
    //     {300,312},{320,328},{340,348},{360,368},{380,388}};
    std::vector<std::vector<int>> boundary = {{4,10},{13,20},{23,26},{34,36},{44,48},{52,58},{62,66},{70,76},{80,84},{90,94}};
    for(auto seg: boundary){
        for(int i = seg[0]; i < seg[1]; i++){
            sub_domain.push_back(i);
        }
    }
    /*Find non zero ig and update the ig*/
    std::vector<int64_t> vt_none_zero_ig_ = sub_domain;
    for(auto&vt: sub_domain){
        Vertex_t<SquareCell*>* vert_ = graph->GetVertexFromID(vt);
        std::vector<SquareCell*> neighbors = grid->GetNeighbors(vt,false);
        for(auto& negb_vt: neighbors){
            std::vector<int64_t>::iterator it_v = std::find(vt_none_zero_ig_.begin(),vt_none_zero_ig_.end(),negb_vt->id_);
            if(it_v == vt_none_zero_ig_.end()){
                vt_none_zero_ig_.push_back(negb_vt->id_);
            }
        }
    }
    // std::cout << "Dimension of sub domain is " << sub_domain.size() << std::endl;
    // std::cout << "Dimension of nz ig region is " << vt_none_zero_ig_.size() << std::endl;
    // std::sort(vt_none_zero_ig_.begin(),vt_none_zero_ig_.end());
    // for(auto vt: vt_none_zero_ig_){
    //     std::cout << vt << ", ";
    // }
    // std::cout << std::endl;
    /*Update the ig*/
    for(size_t ii = 0; ii < grid->num_col_* grid->num_row_; ii++){
        std::vector<int64_t>::iterator it_ig = std::find(vt_none_zero_ig_.begin(),vt_none_zero_ig_.end(),ii);
        Vertex_t<SquareCell*>* vt_ig = graph->GetVertexFromID(ii);
        if(it_ig == vt_none_zero_ig_.end()){
            vt_ig->state_->ig_ = 0.0;
        }
        else{
            vt_ig->state_->ComputeIG(grid, graph, sub_domain);
            // std::cout << "Vertex " << vt_ig->state_->id_ << ": " << vt_ig->state_->ig_ <<std::endl;
        }
    }
    
    std::map<int64_t,Path_t<SquareCell*>> m_paths;
    m_paths[0] = path;
    GraphVisLCM::TransferInfoLCM(graph,m_paths,{});
    std::cout << "Send the graph info. " <<std::endl;



    while(0 == lcm.handle());

	return 0;
}