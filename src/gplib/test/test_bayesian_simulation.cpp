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
        bool returnFlag(){return flag_;}

};

int main(int argc, char** argv )
{
	// Dimension of grid 
	// int64_t num_row = 20;
    // int64_t num_col = 20;
    int64_t num_row = 30;
    int64_t num_col = 30;

	srand(time(NULL));
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    BayesianFlag bayesian_data_;
    lcm.subscribe("BayesianChannel",&BayesianFlag::handleMessage,&bayesian_data_);
    /*Construct auto vehicle team */
    Eigen::MatrixXi comm = Eigen::MatrixXi::Ones(1,1);
    AutoVehicle drone = AutoVehicle(0, num_col*(num_row-1), 1, comm, TaskType::RESCUE,1);
    std::vector<AutoVehicle> drones = {drone};
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team_ = IPASMeasurement::ConstructAutoTeam(drones);

    /*Define a Task*/
    Task task = Task(0,1,{num_col-1},TaskType::RESCUE,1);
    std::vector<Task> tasks_set = {task};
    TasksSet tasks = TasksSet(tasks_set);
    std::vector<double> p_case = {0.2,0.2,0.3,0.3,0.4,0.5,0.5,0.4,0.8,0.8};
    int num_cases = 100;
    int case_idx = 0;
    bool bflag = false;
    while(case_idx<num_cases){
        bflag=false;
        case_idx++;
        /*Construct Grid*/
        std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(num_row, num_col, 1);
        
        for (int i = 0; i < num_row * num_col; i++){
            int itg = rand() % 10;
            grid->SetCellProbability(i,p_case[itg]);
        }
        
        // Set Grid for agent pos and task pos
        grid->SetCellProbability(task.pos_[0],0.0);
        grid->SetCellProbability(vehicle_team_->auto_team_[0]->vehicle_.pos_,0.0);
        
        std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, false, true);
        /*Compute the path */
        vehicle_team_->auto_team_[0]->vehicle_.task_path_={0};
        vehicle_team_->auto_team_[0]->vehicle_.task_bundle_={0};

        // Intialize the local graph for the vehicle
        vehicle_team_->auto_team_[0]->vehicle_.SetLocalMap(grid);
        Path_t<SquareCell*> path = vehicle_team_->auto_team_[0]->vehicle_.PathComputation(tasks,vehicle_team_->auto_team_[0]->vehicle_.local_graph_,vehicle_team_->auto_team_[0]->vehicle_.task_path_,vehicle_team_->auto_team_[0]->vehicle_.pos_);
    
        // std::vector<int64_t> sub_domain = GridGraph::SubRegionFromPaths({path},grid,graph);
        std::vector<int64_t> sub_domain;
        for(int i=0;i <num_row*num_col;i++){
            sub_domain.push_back(i);
        }
  
        /*Find non zero ig and update the ig*/
        std::vector<int64_t> vt_none_zero_ig_ = sub_domain;
       
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
      

        while(bflag==false && 0 == lcm.handle()){bflag=bayesian_data_.returnFlag();}
        std::cout << "JUMP OUT OF WHILE LOOP"<<std::endl;
    }

	return 0;
}