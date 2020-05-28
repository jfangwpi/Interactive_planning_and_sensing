#include "auto_vehicle/auto_vehicle.hpp"
#include "config_reader/config_reader.hpp"

using namespace librav;

AutoVehicle::AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk):
    idx_(idx),pos_(pos),num_vehicles_(num_v),vehicle_type_(ve_cap),num_tasks_(num_tk)
{   
    network_topo_ = Eigen::MatrixXi::Zero(1,num_vehicles_);
    for(size_t ii = 0; ii < num_vehicles_; ii++){
        network_topo_(ii) = network_topo(ii);
    }
    InitCBBA(num_tk);
}

AutoVehicle::AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t num_tk,int64_t num_ss):
    idx_(idx),pos_(pos),num_vehicles_(num_v),vehicle_type_(ve_cap),num_tasks_(num_tk),num_sensors_(num_ss)
{
    network_topo_ = Eigen::MatrixXi::Ones(1,num_vehicles_);
    for(size_t ii = 0; ii < num_vehicles_; ii++){
        network_topo_(ii) = network_topo(ii);
    }
    InitCBBA(num_tasks_);
    local_grid_ = nullptr;
    local_graph_ = nullptr;
    hotspots_ = {};
    nz_ig_zone_ = {};
    history_hspots_ = {};
}

AutoVehicle::AutoVehicle(int64_t idx,int64_t pos,int64_t num_v,Eigen::MatrixXi network_topo,TaskType ve_cap,int64_t itk, int64_t ctk, int64_t num_ss):
    idx_(idx),pos_(pos),num_vehicles_(num_v),vehicle_type_(ve_cap),num_tasks_(itk),num_collaborative_tasks_(ctk),num_sensors_(num_ss)
{
    network_topo_ = Eigen::MatrixXi::Ones(1,num_vehicles_);
    for(size_t ii = 0; ii < num_vehicles_; ii++){
        network_topo_(ii) = network_topo(ii);
    }
    InitSyn(num_tasks_);
    local_grid_ = nullptr;
    local_graph_ = nullptr;
    hotspots_ = {};
    nz_ig_zone_ = {};
    history_hspots_ = {};
}

void AutoVehicle::SetLocalMap(std::shared_ptr<SquareGrid> grid){
    local_grid_ = grid->DuplicateSquareGrid();
    local_graph_ = GridGraph::BuildGraphFromSquareGrid(local_grid_,false,true);
}

void AutoVehicle::InitCBBA(int64_t num_tk){
    reward_ = {};
    num_tasks_ = num_tk;

    task_bundle_ = {};
    task_path_ = {};

    opt_pos_ = -1 * Eigen::MatrixXi::Ones(1,num_tk);
    h_avai_ = {};

    cbba_y_ = Eigen::MatrixXd::Zero(1,num_tk);
    cbba_z_ = -1 * Eigen::MatrixXi::Ones(1,num_tk);   

    iteration_neighb_ = -1 * Eigen::MatrixXi::Ones(1,num_vehicles_);

    cbba_history_.y_history_ = {cbba_y_};
    cbba_history_.z_history_ = {cbba_z_};
    cbba_history_.iteration_neighb_history_ = {iteration_neighb_};

    cbba_iter_ = 0;
}

void AutoVehicle::InitSyn(int64_t num_tk){
    InitCBBA(num_tk);
    num_independent_tasks_ = num_tasks_ - num_collaborative_tasks_;
    syn_c_ = -1 * Eigen::MatrixXd::Ones(1, num_tasks_);
	
	// Initialize dependent tasks
	assignment_matrix_ = Eigen::MatrixXd::Zero(num_tasks_, num_vehicles_);
	winning_bids_matrix_ = -1 * Eigen::MatrixXd::Ones(num_tasks_, num_vehicles_);
    cbba_history_.assignment_matrix_history_ = {assignment_matrix_};
    cbba_history_.winning_bids_history_ = {winning_bids_matrix_};
}


void AutoVehicle::UpdateReward(TasksSet tasks){
    // Find the task which have not in the task bundle
    std::vector<int64_t> tk_NotInBunlde = {};
    for(auto& tk_: tasks.tasks_){
        std::vector<int64_t>::iterator it_tk = std::find(task_bundle_.begin(),task_bundle_.end(),tk_.idx_);
        if(it_tk == task_bundle_.end() && vehicle_type_ == tk_.task_type_){
            tk_NotInBunlde.push_back(tk_.idx_);
        }
    }

    for(int64_t jj = 0; jj < tk_NotInBunlde.size(); jj++){
        // Find all possible insert positions along current task path
        std::vector<std::vector<int64_t>> dup_path_(task_path_.size()+1,task_path_);
        std::vector<double> poss_reward_ = {};
        for(int64_t mm = 0; mm < dup_path_.size(); mm++){
            std::vector<int64_t>::iterator it_insert = dup_path_[mm].begin();
            dup_path_[mm].insert(it_insert+mm,tk_NotInBunlde[jj]);

            // Compute the path cost corresponding to the duplicated task path
            double path_cost = PathCostComputation(tasks,local_graph_,dup_path_[mm],pos_);
            poss_reward_.push_back(path_cost);
        }
        double rw_min = 1e10;
        for(int64_t kk = 0; kk < poss_reward_.size(); kk++){
            if(poss_reward_[kk] < rw_min){
                rw_min = poss_reward_[kk];
                opt_pos_(tk_NotInBunlde[jj]) = kk;
            }
        }
        if (vehicle_type_ == TaskType::MEASURE){
            reward_[tk_NotInBunlde[jj]] = RW_BENEFIT_ - rw_min;
            // reward_[tk_NotInBunlde[jj]] = RW_BENEFIT_ - rw_min -  MEASUREMENT_COST_ * (opt_pos_(tk_NotInBunlde[jj]) + 1);
            // reward_[tk_NotInBunlde[jj]] = RW_BENEFIT_ - rw_min -  MEASUREMENT_COST_ * task_path_.size();
        }
        else{
            reward_[tk_NotInBunlde[jj]] = RW_BENEFIT_ - rw_min;
        }
    }
}

double AutoVehicle::PathCostComputation(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> tk_path, int64_t init_pos){
    double path_cost_ = 0.0;
    if (tk_path.empty()){
        return 0.0;
    }
    else{
        Task final_tk = tasks.GetTaskFromID(tk_path.back());
        Path_t<SquareCell*> path;

        int64_t start_idx = init_pos;
        for(auto& tk_idx: tk_path){
            Task task = tasks.GetTaskFromID(tk_idx);
            // Whether vehicle is able to acomplish the task
            if(vehicle_type_ == task.task_type_){
                for(auto& pos: task.pos_){
                    // Vertex_t<SquareCell*>* vert = graph->GetVertexFromID(pos);
                    Vertex_t<SquareCell*>* vert = local_graph_->GetVertexFromID(pos);
                    int64_t target_idx = vert->state_->id_;
                    Path_t<SquareCell*> path_seg = {};
                    // Path_t<SquareCell*> path_seg = AStar::Search(graph,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristicUncertain));
                    if(vehicle_type_ == TaskType::RESCUE){
                        path_seg = AStar::Search(local_graph_,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristicUncertain));
                    }
                    else if (vehicle_type_ == TaskType::MEASURE){
                        path_seg = AStar::SearchIgnoreEdgeCost(local_graph_,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristic));
                    }
                    if(target_idx == final_tk.pos_.back()){
                        path.insert(path.end(), path_seg.begin(), path_seg.end());
                    }
                    else{
                        path.insert(path.end(), path_seg.begin(), path_seg.end()-1);
                    }
                    start_idx = target_idx;
                }
            }
            else{
                return NOT_CAPABILITY_COST_;
            }
        }
        
        
        // path_cost_ = double(path.size());
        if(vehicle_type_ == TaskType::RESCUE){
            for(int64_t kk = 0; kk < path.size()-1; kk++){
                path_cost_ += GridGraph::CalcHeuristicUncertain(path[kk],path[kk+1]);
            }
        }
        else if (vehicle_type_ == TaskType::MEASURE){
            path_cost_ = double(path.size());
        }
        return path_cost_;
    }
}

void AutoVehicle::BundleRemove(){
    bool flag_outbid = 0;
    if(!task_bundle_.empty()){
        for(int64_t jj = 0; jj < task_bundle_.size(); jj++){
            if(cbba_z_(task_bundle_[jj]) != idx_){
                flag_outbid = 1;
            }
            if (flag_outbid == 1){
                if(cbba_z_(task_bundle_[jj]) == idx_){
                    cbba_z_(task_bundle_[jj]) = -1;
                    cbba_y_(task_bundle_[jj]) = -1;
                }
                task_bundle_[jj] = -1;
            }
        }
        task_bundle_.erase(std::remove(task_bundle_.begin(),task_bundle_.end(),-1),task_bundle_.end());
    }
    PathRemove();
}

void AutoVehicle::PathRemove(){
    if(!task_path_.empty()){
        std::vector<int64_t> duplicated_path_ = task_path_;
        for(auto& tk: duplicated_path_){
            std::vector<int64_t>::iterator it_remove = std::find(task_bundle_.begin(),task_bundle_.end(),tk);
            if(it_remove == task_bundle_.end()){
                task_path_.erase(std::remove(task_path_.begin(),task_path_.end(),tk),task_path_.end());
            }
        }
    }
}
    
void AutoVehicle::BundleAdd(TasksSet tasks){
    while(task_bundle_.size() <= MAX_TASKS){
        // Update the reward 
        UpdateReward(tasks);
        int64_t opt_task_ = FindOptIndeTask();
        if(opt_task_ != -1){
            cbba_z_(opt_task_) = idx_;
            cbba_y_(opt_task_) = reward_[opt_task_];

            std::vector<int64_t>::iterator it_path = task_path_.begin();
            task_path_.insert(it_path+opt_pos_(opt_task_),opt_task_);
            task_bundle_.push_back(opt_task_);
        }
        else{
            break;
        }
    }
    cbba_history_.z_history_.push_back(cbba_z_);
    cbba_history_.y_history_.push_back(cbba_y_);  
}


int64_t AutoVehicle::FindOptIndeTask(){
    int64_t opt_tk = -1;
    // Find available tasks
    AvailableTasks();

    double max_rw = 0.0;
    if(!h_avai_.empty()){
        for(auto&tk_idx: h_avai_){
            if(reward_[tk_idx] > max_rw){
                max_rw = reward_[tk_idx];
                opt_tk = tk_idx;
            }
        }
    }
    return opt_tk;
}

void AutoVehicle::AvailableTasks(){
    // Initialize h_avai
    h_avai_.clear();
    std::map<int64_t,double>::iterator it_tk;
    for(it_tk = reward_.begin(); it_tk != reward_.end(); it_tk++){
        if(it_tk->second - cbba_y_(it_tk->first) > EPS_){
            h_avai_.push_back(it_tk->first);
        }
        else if(std::fabs(it_tk->second - cbba_y_(it_tk->first))<= EPS_){
            if(idx_ < cbba_z_(it_tk->first)){
                h_avai_.push_back(it_tk->first);
            }
        }
        else{
            continue;
        }
    }
}

std::shared_ptr<AutoTeam_t<AutoVehicle>> IPASMeasurement::ConstructAutoTeam(){
    // Read from config file 
    ConfigReader config_reader("../../src/config/agent.ini");
    if (config_reader.CheckError()){
        std::cout << "Reading config file failed." << std::endl;
    }

    int64_t num_agents = config_reader.GetReal("num_agents", 0);
    int64_t num_tasks = config_reader.GetReal("num_tasks",0);
    int64_t num_sensors = config_reader.GetReal("num_sensors",0);

	std::vector<AutoVehicle> auto_vehicles;
	for (int ii= 0; ii < num_agents; ii++){
		std::string agent_name = "agent"+ std::to_string(ii);
		std::string agent_idx = agent_name + "_idx";
		std::string agent_init_pos = agent_name + "_init";
		std::string agent_neighbors = agent_name + "_neighbors";
        std::string agent_type = agent_name + "_type";

		// Read config from agent.ini
		int idx = ii;
		int init_pos = config_reader.GetReal(agent_init_pos, 0);
		Eigen::MatrixXi comm_net = config_reader.GetVectorBinary(agent_neighbors, Eigen::MatrixXi::Ones(1,num_agents));
		std::string auto_type = config_reader.GetString(agent_type,"");
        TaskType type;
        if(auto_type == "RESCUE"){
            type = TaskType::RESCUE;
        }
        else if (auto_type == "MEASURE"){
            type = TaskType::MEASURE;
        }else {
            std::cout << "Unknown task type." << std::endl;
        }

        AutoVehicle auV = AutoVehicle(idx,init_pos,num_agents,comm_net,type,num_tasks,num_sensors);
        auto_vehicles.push_back(auV);
	}
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_group = ConstructAutoTeam(auto_vehicles);
    std::cout << "Configuration reading is done. " << std::endl;
    return vehicle_group;
}

std::shared_ptr<AutoTeam_t<AutoVehicle>> IPASMeasurement::ConstructAutoTeam(std::vector<AutoVehicle>& team){
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_group = std::make_shared<AutoTeam_t<AutoVehicle>>();
    for(auto& ve_:team){
        Vehicle_t<AutoVehicle> *vehicle_ = new Vehicle_t<AutoVehicle>(ve_);
        vehicle_group->auto_team_.push_back(vehicle_);
    }
    // Organize the team
    vehicle_group->measure_team_ = {};
    vehicle_group->rescue_team_ = {};
    for(auto&agent: vehicle_group->auto_team_){
        if(agent->vehicle_type_ == TaskType::MEASURE){
            vehicle_group->measure_team_.push_back(agent);
        }
        else{
            vehicle_group->rescue_team_.push_back(agent);
        }
    }

    vehicle_group->num_vehicles_ = vehicle_group->auto_team_.size();
    // std::cout << "The number of vehicle is " << vehicle_group->auto_team_.size() << std::endl;
    vehicle_group->num_tasks_ = vehicle_group->auto_team_.front()->vehicle_.num_tasks_;
    return vehicle_group;
}

TasksSet IPASMeasurement::ConstructLTLTasks(){
    ConfigReader config_reader("../../src/config/task.ini");
    if (config_reader.CheckError()){
        std::cout << "Reading config file failed." << std::endl;
    }
    int64_t num_AP = config_reader.GetReal("num_AP", 0);
    std::vector<Task> tasks;
    for(int jj = 0; jj < num_AP; jj++){
        std::string AP_idx = "AP"+ std::to_string(jj);
		std::string AP_pos = AP_idx + "_pos";
		std::string AP_agents = AP_idx + "_num_agents";
        std::string AP_type = AP_idx + "_type";

        int64_t idx = jj;
        int64_t t_AP = config_reader.GetReal(AP_idx,0);
        int64_t t_pos = config_reader.GetReal(AP_pos,0);
        std::string t_type = config_reader.GetString(AP_type,"");
        std::size_t rescue_found = t_type.find("RESCUE");
        std::size_t measurement_found = t_type.find("MEASURE");
        TaskType tt;
        if(rescue_found != std::string::npos){
            tt = TaskType::RESCUE;
        }
        else if (measurement_found != std::string::npos){
            tt = TaskType::MEASURE;
        }else {
            std::cout << "Unknown task type." << std::endl;
        }
        int64_t t_agents = config_reader.GetReal(AP_agents,1);

        Task tks(jj,t_AP,{t_pos},tt,t_agents);
        tasks.push_back(tks);
    }
    TasksSet tkSet(tasks);
    return tkSet;
}



void AutoVehicle::ComputeLocalHotspots(TasksSet tasks){
    hotspots_ = {};
    nz_ig_zone_ = {};
    rois_ = {};
    // Decompose the path to each smaller segments
	std::vector<Path_t<SquareCell*>> paths_;
	if(!task_path_.empty()){
		int64_t start_id = pos_;
		for(auto&tt: task_path_){
			Task tk = tasks.GetTaskFromID(tt);
			std::vector<int64_t> target_ids = tk.pos_;
			for(auto&kk: target_ids){
				int64_t tg_id = kk;
				Path_t<SquareCell*> path_seg = AStar::Search(local_graph_,start_id,tg_id,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristicUncertain));
				paths_.push_back(path_seg);
				start_id = tg_id;
			}
		}

        // Find the sub-region from the local graph and paths
        std::vector<int64_t> sub_domain = GridGraph::SubRegionFromPaths(paths_,local_grid_,local_graph_);
        rois_ = sub_domain;
        // std::cout << "===========================" <<std::endl;
        // std::cout << "===========================" <<std::endl;
        // std::cout << "The sub_domain for vehicle " << idx_ << " is:";
        // for(auto kk: sub_domain){
        //     std::cout << kk << ", ";
        // }
        // std::cout << std::endl;

        // Find the HotSpots
        // Find the set of vertices with none-zero ig
        std::vector<int64_t> vt_none_zero_ig_ = sub_domain;
        for(auto&vt: sub_domain){
            Vertex_t<SquareCell*>* vert_ = local_graph_->GetVertexFromID(vt);
            std::vector<SquareCell*> neighbors = local_grid_->GetNeighbors(vt,false);
            for(auto& negb_vt: neighbors){
                std::vector<int64_t>::iterator it_v = std::find(vt_none_zero_ig_.begin(),vt_none_zero_ig_.end(),negb_vt->id_);
                if(it_v == vt_none_zero_ig_.end()){
                    vt_none_zero_ig_.push_back(negb_vt->id_);
                }
            }
        }
        


        // Compute the ig for local graph
        for(size_t ii = 0; ii < local_grid_->num_col_* local_grid_->num_row_; ii++){
            std::vector<int64_t>::iterator it_ig = std::find(vt_none_zero_ig_.begin(),vt_none_zero_ig_.end(),ii);
            Vertex_t<SquareCell*>* vt_ig = local_graph_->GetVertexFromID(ii);
            if(it_ig == vt_none_zero_ig_.end()){
                vt_ig->state_->ig_ = 0.0;
            }
            else{
                vt_ig->state_->ComputeIG(local_grid_, local_graph_, sub_domain);
                nz_ig_zone_[vt_ig->state_->id_] = vt_ig->state_->ig_;
            }
        }
        // Find the sensors pos and hotspots
        std::vector<int64_t> sensor_pos = GridGraph::SelectSensorsPos(num_sensors_, local_graph_);
        for(auto& ss: sensor_pos){
            Vertex_t<SquareCell*>* vert = local_graph_->GetVertexFromID(ss);
            std::pair<int,double> hspot = std::make_pair(ss,vert->state_->ig_);
            hotspots_.push_back(hspot); 
        }
	}
    else {
        for(size_t ii = 0; ii < local_grid_->num_col_* local_grid_->num_row_; ii++){
            Vertex_t<SquareCell*>* vt_ig = local_graph_->GetVertexFromID(ii);
            vt_ig->state_->ig_ = 0.0;
        }
    }
    history_hspots_.push_back(hotspots_);
    std::cout << "The hotspots of vehicle " << idx_ << " is :" << std::endl;
    for(auto&spot:hotspots_){
        std::cout << spot.first << " (" << spot.second << ")" << std::endl;
    }
}

std::vector<int64_t> AutoVehicle::ComputeLocalROIs(TasksSet tasks){
    // Decompose the path to each smaller segments
	std::vector<Path_t<SquareCell*>> paths_;
	if(!task_path_.empty()){
		int64_t start_id = pos_;
		for(auto&tt: task_path_){
			Task tk = tasks.GetTaskFromID(tt);
			std::vector<int64_t> target_ids = tk.pos_;
			for(auto&kk: target_ids){
				int64_t tg_id = kk;
				Path_t<SquareCell*> path_seg = AStar::Search(local_graph_,start_id,tg_id,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristicUncertain));
				paths_.push_back(path_seg);
				start_id = tg_id;
			}
		}

        // Find the sub-region from the local graph and paths
        std::vector<int64_t> sub_domain = GridGraph::SubRegionFromPaths(paths_,local_grid_,local_graph_);
        rois_ = sub_domain;

        // Find the HotSpots
        // Find the set of vertices with none-zero ig
        std::vector<int64_t> vt_none_zero_ig_ = sub_domain;
        for(auto&vt: sub_domain){
            Vertex_t<SquareCell*>* vert_ = local_graph_->GetVertexFromID(vt);
            std::vector<SquareCell*> neighbors = local_grid_->GetNeighbors(vt,false);
            for(auto& negb_vt: neighbors){
                std::vector<int64_t>::iterator it_v = std::find(vt_none_zero_ig_.begin(),vt_none_zero_ig_.end(),negb_vt->id_);
                if(it_v == vt_none_zero_ig_.end()){
                    vt_none_zero_ig_.push_back(vt);
                }
            }
        }
    }
}

// void IPASMeasurement::TransferGraphInfoBayesian(std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<int64_t> ROIs,std::vector<int64_t> nzig,std::vector<int64_t> samples){
//     srand(time(NULL));
//     lcm::LCM lcm;

//     // Collect all vertex
//     std::vector<Vertex_t<SquareCell*>*> all_vertex = graph->GetAllVertex();
//     graph_data::map_data map_data_;
//     map_data_.num_v_ = all_vertex.size();
//     for(auto & cell: all_vertex){
//         graph_data::vertex_data vertex_;
//         vertex_.idx_ = cell->state_->id_;

//         std::vector<int64_t>::iterator it_roi = std::find(ROIs.begin(),ROIs.end(),cell->state_->id_);
//         if(it_roi!=ROIs.end()){vertex_.is_ROI_=true;}
//         else{vertex_.is_ROI_=false;}

//         std::vector<int64_t>::iterator it_nzig = std::find(nzig.begin(),nzig.end(),cell->state_->id_);
//         if(it_nzig!=nzig.end()){vertex_.is_nzig_=true;}
//         else{vertex_.is_nzig_=false;}

//         std::vector<int64_t>::iterator it_sample = std::find(samples.begin(),samples.end(),cell->state_->id_);
//         if(it_sample!=samples.end()){vertex_.is_sample_=true;vertex_.ig_=cell->state_->ig_;}
//         else{vertex_.is_sample_=false;vertex_.ig_=0.0;}

//         // vertex_.ig_ = cell->state_->ig_;
//         vertex_.p_ = cell->state_->p_;
//         map_data_.vertex_.push_back(vertex_);
//     }

//     std::string Graph_data = std::to_string(fig_idx_) + "GraphDataBaysian";
//     lcm.publish(Graph_data, &map_data_);
//     fig_idx_++;
// }

Path_t<SquareCell*> AutoVehicle::PathComputation(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> tk_path, int64_t init_pos){
    Path_t<SquareCell*> path;
    if (tk_path.empty()){
        return path;
    }
    else{
        Task final_tk = tasks.GetTaskFromID(tk_path.back());
        int64_t start_idx = init_pos;
        for(auto& tk_idx: tk_path){
            Task task = tasks.GetTaskFromID(tk_idx);
            // Whether vehicle is able to acomplish the task
            if(vehicle_type_ == task.task_type_){
                for(auto& pos: task.pos_){
                    Vertex_t<SquareCell*>* vert = graph->GetVertexFromID(pos);
                    int64_t target_idx = vert->state_->id_;
                    Path_t<SquareCell*> path_seg;
                    if(vehicle_type_ == TaskType::RESCUE){
                        path_seg = AStar::Search(graph,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristicUncertain));
                    }
                    else if (vehicle_type_ == TaskType::MEASURE){
                        path_seg = AStar::SearchIgnoreEdgeCost(graph,start_idx,target_idx,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristic));
                    }
                    
                    if(target_idx == final_tk.pos_.back()){
                        path.insert(path.end(), path_seg.begin(), path_seg.end());
                    }
                    else{
                        path.insert(path.end(), path_seg.begin(), path_seg.end()-1);
                    }
                    start_idx = target_idx;
                }
            }
        }
    }
    return path;
}

std::map<int64_t,Path_t<SquareCell*>> IPASMeasurement::GeneratePaths(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team,TasksSet tasks,TaskType task_type){
    std::map<int64_t,Path_t<SquareCell*>> paths_map_;
    for(auto agent: vehicle_team->auto_team_){
        if (agent->vehicle_.vehicle_type_ == task_type){
            if(!agent->vehicle_.task_path_.empty()){
                paths_map_[agent->vehicle_.idx_] = agent->vehicle_.PathComputation(tasks,agent->vehicle_.local_graph_,agent->vehicle_.task_path_,agent->vehicle_.pos_);
            }
            else{
                Path_t<SquareCell*> empty_path_ = {};
                paths_map_[agent->vehicle_.idx_] = empty_path_;
            }
        }
        else{
            Path_t<SquareCell*> empty_path_ = {};
            paths_map_[agent->vehicle_.idx_] = empty_path_;
        }    
    }
    return paths_map_;
}

int64_t IPASMeasurement::GenerateTruePathsCost(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team,TasksSet tasks,std::shared_ptr<Graph_t<SquareCell*>> true_graph){
    int64_t path_cost = 0;
    for(auto agent: vehicle_team->auto_team_){
        if (agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
            if(!agent->vehicle_.task_path_.empty()){
                path_cost += agent->vehicle_.PathComputation(tasks,true_graph,agent->vehicle_.task_path_,agent->vehicle_.pos_).size();
            }
        } 
    }
    return path_cost;
}


bool IPASMeasurement::IPASConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> team,std::map<int64_t,Path_t<SquareCell*>> paths_map){
    for(auto vehicle: team->auto_team_){
        if(paths_map[vehicle->vehicle_.idx_].empty()){return true;}
        double en_thre = paths_map[vehicle->vehicle_.idx_].size() * ENTROPY_THRED_;
        double entropy_path = GridGraph::EntropyPath(paths_map[vehicle->vehicle_.idx_],vehicle->vehicle_.local_graph_);
        if(entropy_path > en_thre){return false;}
    }
    return true;
}

void AutoVehicle::UpdateLocalMap(std::shared_ptr<Graph_t<SquareCell*>> true_graph,TasksSet tasks){
    if(!task_path_.empty()){
        for(auto&tk_id: task_path_){
            // Find the center of sensor
            Task task = tasks.GetTaskFromID(tk_id);
            Vertex_t<SquareCell*>* true_cs= true_graph->GetVertexFromID(task.pos_.front());
            Vertex_t<SquareCell*>* uncertain_cs = local_graph_->GetVertexFromID(task.pos_.front());
            uncertain_cs->state_->occupancy_ = true_cs->state_->occupancy_;
            if(uncertain_cs->state_->occupancy_ == OccupancyType::FREE || uncertain_cs->state_->occupancy_ == OccupancyType::INTERESTED){
                uncertain_cs->state_->p_ = 0.0;
            }
            else if(uncertain_cs->state_->occupancy_ == OccupancyType::OCCUPIED){
                uncertain_cs->state_->p_ = 1.0;
            }

            //Update the vertices inside of the sensing range
            std::vector<Vertex_t<SquareCell*>*> neighbors = uncertain_cs->GetNeighbours();
            for(auto& neigb: neighbors){
                Vertex_t<SquareCell*>* true_neigb = true_graph->GetVertexFromID(neigb->state_->id_);
                if(neigb->state_->occupancy_ == OccupancyType::UNKNOWN){
                    if(true_neigb->state_->occupancy_ == OccupancyType::FREE || true_neigb->state_->occupancy_ == OccupancyType::INTERESTED){
                        double belif_p = GridGraph::UpdateBelProbability(neigb->state_->p_,0);
                        neigb->state_->p_ = belif_p;
                        local_grid_->SetCellProbability(neigb->state_->id_,belif_p);
                        if(neigb->state_->p_ <= cell_accurancy_){
                            neigb->state_->occupancy_ = OccupancyType::FREE;
                            neigb->state_->p_ = 0.0;
                        }
                        if (neigb->state_->p_ > 1.0 - cell_accurancy_){
                            neigb->state_->occupancy_ = OccupancyType::OCCUPIED;
                            neigb->state_->p_ = 1.0;
                        }
                    }
                    else if (true_neigb->state_->occupancy_ == OccupancyType::OCCUPIED){
                        double belif_p = GridGraph::UpdateBelProbability(neigb->state_->p_,1);
                        neigb->state_->p_ = belif_p;
                        if(neigb->state_->p_ <= cell_accurancy_){
                            neigb->state_->occupancy_ = OccupancyType::FREE;
                            neigb->state_->p_ = 0.0;
                        }
                        if (neigb->state_->p_ > 1.0 - cell_accurancy_){
                            neigb->state_->occupancy_ = OccupancyType::OCCUPIED;
                            neigb->state_->p_ = 1.0;
                        }
                    }
                }
            }
        }

        // Update the cost of local graph
        if (vehicle_type_ == TaskType::RESCUE){
            auto edges = local_graph_->GetAllEdges();
            std::vector<int64_t> rm_dst;
            for (auto& e: edges){
                Vertex_t<SquareCell*>* dst_v = e->dst_;
                double uncertain_cost = 1.0 * (1.0 - dst_v->state_->p_) + PENALTY_*dst_v->state_->p_;
                e->cost_ = uncertain_cost;
                // remove the edge points to obstacle
                if(dst_v->state_->p_ == 1.0){
                    rm_dst.push_back(dst_v->state_->id_);
                }
            }
        }
    }
}

void IPASMeasurement::ComputeHotSpots(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team, TasksSet tasks){
    for(auto&agent: vehicle_team->auto_team_){
        agent->vehicle_.ComputeLocalHotspots(tasks);
    }
}

void IPASMeasurement::HotSpotsConsensus(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team){
    std::map<int64_t,double> mm;
    for(auto agent: vehicle_team->auto_team_){
        for(auto hspt: agent->vehicle_.hotspots_){
            if(mm.count(hspt.first)==0){mm[hspt.first] = hspt.second;}
        }
    }
    int64_t cont = 0;
    std::vector<std::pair<int64_t,double>> consensus_hspts_={};
    while(consensus_hspts_.size() < vehicle_team->auto_team_[0]->vehicle_.num_sensors_){
        double max_hspt = 1e-6;
        int64_t max_hspt_idx = -1;
        for(auto cc: mm){
            if(cc.second - max_hspt > 1e-6){
                max_hspt = cc.second;
                max_hspt_idx = cc.first;
            }
            else if (fabs(cc.second - max_hspt) <= 1e-6){
                if(cc.first < max_hspt_idx){
                    max_hspt_idx = cc.first;
                }
            }
            else{continue;}
        }
        
        std::pair<int64_t,double> ig_pair = std::make_pair(max_hspt_idx,max_hspt);
        consensus_hspts_.push_back(ig_pair);
        mm.erase(max_hspt_idx);
    }

    for(auto &agent: vehicle_team->auto_team_){
        agent->vehicle_.hotspots_ = consensus_hspts_;
        agent->vehicle_.history_hspots_.push_back(agent->vehicle_.hotspots_);
    }
}

std::pair<int64_t,double> AutoVehicle::MinHotSpot(){
    std::pair<int64_t,double> min_spot_ = std::make_pair(-1,1e2);
    for(auto&hspt: hotspots_){
        if(min_spot_.second - hspt.second > 1e-3){
            min_spot_.first = hspt.first;
            min_spot_.second = hspt.second;
        }
        else if (fabs(min_spot_.second - hspt.second) <= 1e-3){
            if(min_spot_.first < hspt.first){
                min_spot_.first = hspt.first;
                min_spot_.second = hspt.second;
            }
        }
    }
    return min_spot_;
}

bool IPASMeasurement::HotSpotsConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> team){
    bool flag_ = true;
    for(auto&agent: team->auto_team_){
        std::vector<int64_t> neigb_idx = agent->GetNeighborsIDs();
        for(auto&nb_id: neigb_idx){
            Vehicle_t<AutoVehicle>* neighb = team->GetVehicleFromID(nb_id);
            for(auto& ag_pair: agent->vehicle_.hotspots_){
                std::vector<std::pair<int64_t,double>>::iterator it_hspot = std::find(neighb->vehicle_.hotspots_.begin(),neighb->vehicle_.hotspots_.end(),ag_pair);
                if(it_hspot == neighb->vehicle_.hotspots_.end()){
                    flag_ = false;
                    return flag_;
                }
            }
        }
    }
    return true;
}

TasksSet IPASMeasurement::ConstructMeasurementTasks(std::shared_ptr<AutoTeam_t<AutoVehicle>> team){
    // Create the measurement tasks
    std::vector<Task> tasks;
    for(int64_t kk = 0; kk < team->auto_team_[0]->vehicle_.hotspots_.size(); kk++){
        std::pair<int64_t,double> hspot = team->auto_team_[0]->vehicle_.hotspots_[kk];
        std::cout << "The hspt is " << hspot.first << std::endl;
        Task task(kk,0,{hspot.first},TaskType::MEASURE,1);
        tasks.push_back(task);
    }
    TasksSet tasks_(tasks);
    return tasks_;
}

void IPASMeasurement::MergeHSpots(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams){
    bool flag_ = false;
    for(auto&agent: teams->auto_team_){
        agent->vehicle_.iteration_neighb_ = Eigen::MatrixXi::Zero(1,teams->auto_team_.size());
        agent->vehicle_.iteration_neighb_(agent->vehicle_.idx_) = 1;
    }
    while (flag_ != true){
        for(auto &agent: teams->auto_team_){
            std::vector<int64_t> neighbors = agent->GetNeighborsIDs();
            for(auto nb_id: neighbors){
                Vehicle_t<AutoVehicle>* neighbor = teams->GetVehicleFromID(nb_id);
                // Update ROIs_
                for (auto& r: neighbor->vehicle_.rois_){
                    std::vector<int64_t>::iterator itr = std::find(agent->vehicle_.rois_.begin(),agent->vehicle_.rois_.end(),r);
                    if (itr == agent->vehicle_.rois_.end()){
                        agent->vehicle_.rois_.push_back(r);
                    }
                }
                for(auto& nzvert: neighbor->vehicle_.nz_ig_zone_){
                    Vertex_t<SquareCell*>* local_vt = agent->vehicle_.local_graph_->GetVertexFromID(nzvert.first);
                    if(nzvert.second > local_vt->state_->ig_){
                        local_vt->state_->ig_ = nzvert.second;
                        agent->vehicle_.nz_ig_zone_[nzvert.first] = nzvert.second;
                    }
                }

                for(int64_t ii = 0; ii < teams->auto_team_.size(); ii++){
                    if(neighbor->vehicle_.iteration_neighb_(ii) > agent->vehicle_.iteration_neighb_(ii)){
                        agent->vehicle_.iteration_neighb_(ii) = neighbor->vehicle_.iteration_neighb_(ii);
                    }
                } 
            }
        }
        flag_ = MapMergeConvergence(teams);
        if (flag_ == true){break;}
    }
}

void IPASMeasurement::MergeLocalMap(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams){
    bool flag_ = false;
    for(auto&agent: teams->auto_team_){
        agent->vehicle_.iteration_neighb_ = Eigen::MatrixXi::Zero(1,teams->auto_team_.size());
        agent->vehicle_.iteration_neighb_(agent->vehicle_.idx_) = 1;
    }
    while (flag_ != true){
        for(auto&agent: teams->auto_team_){
            std::vector<int64_t> neighbors = agent->GetNeighborsIDs();
            for(auto&nb_id: neighbors){
                Vehicle_t<AutoVehicle>* neighbor = teams->GetVehicleFromID(nb_id);
                for(auto& hspot: agent->vehicle_.hotspots_){
                    // Find the set of vertices which are required to update
                    std::vector<Vertex_t<SquareCell*>*> vert_to_upadte = {};
                    Vertex_t<SquareCell*>* vt = agent->vehicle_.local_graph_->GetVertexFromID(hspot.first);
                    if(vt->state_->occupancy_ == OccupancyType::UNKNOWN){
                        vert_to_upadte.push_back(vt);
                    }
                    std::vector<Vertex_t<SquareCell*>*> vt_neigbs = vt->GetNeighbours();
                    for(auto&vt_nb:vt_neigbs){
                        if(vt_nb->state_->occupancy_ == OccupancyType::UNKNOWN){
                            vert_to_upadte.push_back(vt_nb);
                        }
                    }
                    // std::cout << "The vertices need to be updated for vehicle " << agent->vehicle_.idx_ << std::endl;
                    for(auto&vv: vert_to_upadte){
                        // std::cout << vv->state_->id_ << ", ";
                        Vertex_t<SquareCell*>* neigb_vt = neighbor->vehicle_.local_graph_->GetVertexFromID(vv->state_->id_);
                        if(neigb_vt->state_->p_ == 0.0 || neigb_vt->state_->p_ == 1.0){
                            vv->state_->p_ = neigb_vt->state_->p_;
                            vv->state_->occupancy_ = neigb_vt->state_->occupancy_;
                        }
                        else{
                            double prob_p_ = vv->state_->p_;
                            double prob_q_ = 1.0 - prob_p_;
                            double current_ep_ = -prob_p_ * log10(prob_p_) - prob_q_ * log10(prob_q_);

                            double neigb_p_ = neigb_vt->state_->p_;
                            double neigb_q_ = 1.0 - neigb_vt->state_->p_;
                            double neigb_ep_ = -neigb_p_ * log10(neigb_p_) - neigb_q_ * log10(neigb_q_);

                            if(neigb_ep_ < current_ep_){
                                vv->state_->p_ = neigb_vt->state_->p_;
                                vv->state_->occupancy_ = neigb_vt->state_->occupancy_;
                            }
                        }
                    }
                }
                for(int64_t ii = 0; ii < teams->auto_team_.size(); ii++){
                    if(neighbor->vehicle_.iteration_neighb_(ii) > agent->vehicle_.iteration_neighb_(ii)){
                        agent->vehicle_.iteration_neighb_(ii) = neighbor->vehicle_.iteration_neighb_(ii);
                    }
                }      
            } 
        }
        flag_ = MapMergeConvergence(teams);
        if(flag_ == true){break;}
    }
    for(auto&agent: teams->auto_team_){
        // Update the cost of local graph
        if(agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
            auto edges = agent->vehicle_.local_graph_->GetAllEdges();
            std::vector<int64_t> dst_vertices;
            for (auto& e: edges){
                double uncertain_cost = 1.0 * (1.0 - e->dst_->state_->p_) + PENALTY_*e->dst_->state_->p_;
                e->cost_ = uncertain_cost;
                if(e->dst_->state_->p_ == 1.0){
                    dst_vertices.push_back(e->dst_->state_->id_);
                }
            }
            for(auto cc: dst_vertices){
                Vertex_t<SquareCell*>* vert = agent->vehicle_.local_graph_->GetVertexFromID(cc);
                std::vector<Vertex_t<SquareCell*>*> neighbs = vert->GetNeighbours();
                for(auto nn: neighbs){
                    agent->vehicle_.local_graph_->RemoveEdge(vert->state_,nn->state_);
                }
            }
        }
    } 
}

bool IPASMeasurement::MapMergeConvergence(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams){
    bool flag_ = true;
    for(auto& agent: teams->auto_team_){
        for(int64_t ii = 0; ii < teams->auto_team_.size(); ii++){
            if(agent->vehicle_.iteration_neighb_(ii) != 1){
                return false;
            }
        }
    }
    return flag_;
}


void IPASMeasurement::InformationDrivenHSpots(std::shared_ptr<AutoTeam_t<AutoVehicle>> teams){
    int64_t num_r = teams->auto_team_[0]->vehicle_.local_grid_->num_row_;
    int64_t num_c = teams->auto_team_[0]->vehicle_.local_grid_->num_col_;
    std::vector<int64_t> whole_map_;
    for(int ii = 0; ii < num_r * num_c; ii++){
        whole_map_.push_back(ii);
    }
    for (int kk = 0; kk < num_r * num_c; kk++){
        teams->auto_team_[0]->vehicle_.hotspots_ = {};
        Vertex_t<SquareCell*>* vert = teams->auto_team_[0]->vehicle_.local_graph_->GetVertexFromID(kk);
        vert->state_->ComputeIG(teams->auto_team_[0]->vehicle_.local_grid_,teams->auto_team_[0]->vehicle_.local_graph_, whole_map_);
        std::vector<int64_t> sensor_pos = GridGraph::SelectSensorsPos(teams->auto_team_[0]->vehicle_.num_sensors_, teams->auto_team_[0]->vehicle_.local_graph_);
        for(auto& ss: sensor_pos){
            Vertex_t<SquareCell*>* vert = teams->auto_team_[0]->vehicle_.local_graph_->GetVertexFromID(ss);
            std::pair<int,double> hspot = std::make_pair(ss,vert->state_->ig_);
            teams->auto_team_[0]->vehicle_.hotspots_.push_back(hspot); 
        }
    }
    teams->auto_team_[0]->vehicle_.history_hspots_.push_back(teams->auto_team_[0]->vehicle_.hotspots_);
    for(int idx = 1; idx < teams->auto_team_.size(); idx++){
        for(int kk = 0; kk < num_r * num_c; kk++){
            Vertex_t<SquareCell*>* vert_s = teams->auto_team_[0]->vehicle_.local_graph_->GetVertexFromID(kk);
            Vertex_t<SquareCell*>* vert = teams->auto_team_[idx]->vehicle_.local_graph_->GetVertexFromID(kk);
            vert->state_->ig_ = vert_s->state_->ig_;
        }
        teams->auto_team_[idx]->vehicle_.hotspots_ = teams->auto_team_[0]->vehicle_.hotspots_;
        teams->auto_team_[idx]->vehicle_.history_hspots_.push_back(teams->auto_team_[0]->vehicle_.hotspots_);
    }
}




// /* Synchronization Task Assignment */
void AutoVehicle::bundle_add_dependent(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph){
	bool bundleFull = 0;
	if(task_bundle_.size() > num_tasks_){
		bundleFull = 1;
	}

	last_pos_dependent_ = -1;
	if (!task_path_.empty()){
		for (int i = task_path_.size()-1; i >= 0; i--){
			if(task_path_[i] >= num_independent_tasks_){
				last_pos_dependent_ = i + 1;
				break;
			}
		}	
	}
	while (bundleFull == 0){
		available_dep_tasks_finder(tasks, graph);
		int desired_idx = desired_dep_task_finder();
		if(desired_idx == -1){
			break;
		}

		task_bundle_.push_back(desired_idx);
		std::vector<int64_t>::iterator it = task_path_.begin();
		task_path_.insert(it+opt_pos_(0, desired_idx), desired_idx);
		
		if(task_path_.size() > num_tasks_){
			bundleFull = 1;
		}
	}
	
	/* ========================================= Debug =====================================*/
	// std::cout << "After inserting all dependent tasks, the optimal sequence is " << std::endl;
	// for (auto &b: cbba_path_){
	// 	std::cout << b << ", ";
	// }
	// std::cout << std::endl;
	// std::cout << "Current winning bids matrix is " << std::endl;
	// std::cout << winning_bids_matrix_ << std::endl;
	// std::cout << "The assignment matrix is " << std::endl;
	// std::cout << assignment_matrix_ << std::endl;

	std::vector<int64_t> current_p = task_path_;
	for (int t_idx = 0; t_idx < current_p.size(); t_idx++){
		Task task = tasks.GetTaskFromID(current_p[t_idx]);
		if(task.num_vehicles_ > 1){
			std::vector<int64_t> path_copy = task_path_;
			std::vector<int64_t>::iterator it = std::find(task_path_.begin(), task_path_.end(), task.idx_);
			int task_length = it - task_path_.begin();
			path_copy.resize(task_length+1);	

			double path_part = RW_BENEFIT_ - PathCostComputation(tasks,local_graph_,path_copy,pos_);
			reward_[task.idx_] = path_part;
			winning_bids_matrix_(task.idx_, idx_) = path_part;

			optimal_group_finder(task);

			if (assignment_matrix_(task.idx_,idx_) == 0){
				task_path_.erase(std::remove(task_path_.begin(), task_path_.end(), task.idx_), task_path_.end());
				task_bundle_.erase(std::remove(task_bundle_.begin(), task_bundle_.end(), task.idx_), task_bundle_.end());
			}
			
			/*** ======================================== DEBUG =============================== ***/
			//std::cout << "============================= DEBUG =============================== " << std::endl;
			// std::cout << "The Result of converging optimal group of vehicle about task " << task.idx_ << " is: " << std::endl;
			// for (auto &b: cbba_path_){
			// 	std::cout << b << ", ";
			// }
			// std::cout << std::endl;
			// std::cout << "Now winning bids matrix becomes " << std::endl;
			// std::cout << winning_bids_matrix_ << std::endl;
			// std::cout << "The assignment matrix becomes " << std::endl;
			// std::cout << assignment_matrix_ << std::endl;
		}
	}

	cbba_history_.assignment_matrix_history_.push_back(assignment_matrix_);
	cbba_history_.winning_bids_history_.push_back(winning_bids_matrix_);
}

/*** Find the available dependent tasks ***/
void AutoVehicle::available_dep_tasks_finder(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph){
	update_reward_dependent(tasks, graph);
	h_avai_de_ = {};
	for (auto &task: tasks.tasks_){
		if (task.num_vehicles_ > 1){
			int de_pos_ = -1;
	
			int num_winners = winners_count(task);
			std::vector<int64_t>::iterator it = std::find(task_bundle_.begin(), task_bundle_.end(), task.idx_);
			if (last_pos_dependent_ == -1){
				de_pos_ = opt_pos_(0, task.idx_);
			}
			else{
				de_pos_ = last_pos_dependent_;
			}

			if(it == task_bundle_.end()){
				if (num_winners < task.num_vehicles_ && opt_pos_(0, task.idx_) >= de_pos_){
					h_avai_de_.push_back(task.idx_);
				}
				else if(num_winners < task.num_vehicles_ && opt_pos_(0, task.idx_) < de_pos_){
					opt_pos_(0, task.idx_) = last_pos_dependent_;
					h_avai_de_.push_back(task.idx_);
				}
				else if(num_winners == task.num_vehicles_ && opt_pos_(0, task.idx_) >= de_pos_){
					h_avai_de_.push_back(task.idx_);
				}
			}
		}
	}	
}

/*** Update syn_c for dependent tasks ***/
void AutoVehicle::update_reward_dependent(TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph){
    // Find dependent task which is not in the curret task path
    std::vector<int64_t> tk_NotInBunlde = {};
    for(auto& tk_: tasks.tasks_){
        std::vector<int64_t>::iterator it_tk = std::find(task_bundle_.begin(),task_bundle_.end(),tk_.idx_);
        if(it_tk == task_bundle_.end() && vehicle_type_ == tk_.task_type_ && tk_.num_vehicles_ > 1){
            tk_NotInBunlde.push_back(tk_.idx_);
        }
    }

	for (int jj = 0; jj < tk_NotInBunlde.size(); jj++){
        // Find all possible insert positions along current task path
        std::vector<std::vector<int64_t>> dup_path_(task_path_.size()+1,task_path_);
        std::vector<double> poss_reward_ = {};
        for(int64_t mm = 0; mm < dup_path_.size(); mm++){
            std::vector<int64_t>::iterator it_insert = dup_path_[mm].begin();
            dup_path_[mm].insert(it_insert+mm,tk_NotInBunlde[jj]);

            // Compute the path cost corresponding to the duplicated task path
            double path_cost = PathCostComputation(tasks,local_graph_,dup_path_[mm],pos_);
            poss_reward_.push_back(path_cost);
        }
        double min_reward = RW_BENEFIT_;
        double best_pos = -1;
        for (int i = 0; i < poss_reward_.size(); i++){
            if(poss_reward_[i] < min_reward){
                min_reward = poss_reward_[i];
                best_pos = i;
            }
        }
        syn_c_(0, tk_NotInBunlde[jj]) = RW_BENEFIT_ - min_reward;
        opt_pos_(0, tk_NotInBunlde[jj]) = best_pos;
	}
}

/*** Find the winners of given dependent task ***/
std::vector<int> AutoVehicle::winners_finder(Task task){
	std::vector<int> winners_ = {};
	for (int i =0; i < num_vehicles_; i++){
		if(assignment_matrix_(task.idx_, i) == 1 && winning_bids_matrix_(task.idx_, i) > 0){
			winners_.push_back(i);
		}
	}
	return winners_;
}

int AutoVehicle::winners_count(Task task){
	std::vector<int> winners = winners_finder(task);
	return winners.size();
}


/*** Find the desired dependent task from the list of available dependent tasks ***/
int AutoVehicle::desired_dep_task_finder(){
	int desired_idx = -1;
	if (!h_avai_de_.empty()){
		double max_reward = 0.0;
		for (auto &t_idx: h_avai_de_){
			if(syn_c_(0, t_idx) > max_reward){
				max_reward = syn_c_(0, t_idx);
				desired_idx = t_idx;
			}
		}
	}
	return desired_idx;
}


/*** Update the optimal group based on current knowledge of reward ***/
void AutoVehicle::optimal_group_finder(Task task){
	// Find the ordered map
	std::multimap<double, int> winning_bids_collect_ = {};
	for (int i = 0; i < num_vehicles_; i++){
		if(winning_bids_matrix_(task.idx_, i) > 0){
			winning_bids_collect_.insert(std::pair<double,int>(winning_bids_matrix_(task.idx_,i),i));
		}
	}

	int n = winning_bids_collect_.size();
	int k = task.num_vehicles_;

	std::multimap<double, int>::iterator it;
	std::vector<double> ordered_bids;
	for (auto it = winning_bids_collect_.begin(); it != winning_bids_collect_.end(); it++){
		ordered_bids.push_back((*it).first);
	}
	std::cout << std::endl;

	std::multimap<double, int>::iterator it1;
	double waiting_time_min = RW_BENEFIT_;
	std::vector<double> winners_bids;
	for (auto it1 = winning_bids_collect_.begin(); it1 != winning_bids_collect_.end(); it1++){
		double x = (*it1).first;
		std::vector<double> winning_bids = CollaborativeAlgorithm::KClosestFinder(ordered_bids, x, k, n);
		double max_bid = CollaborativeAlgorithm::maximum_bid_finder(winning_bids);
		double min_bid = CollaborativeAlgorithm::minimum_bid_finder(winning_bids);
		double waiting_time = max_bid - min_bid;
		if(waiting_time <= waiting_time_min){
			waiting_time_min = waiting_time;
			winners_bids = winning_bids;
		}
	}

	std::vector<int> winners = FindVehicleFrombid(task, winners_bids);
	for (int i = 0; i < num_vehicles_;i++){
		assignment_matrix_(task.idx_, i) = 0;
	}
	for (auto &e: winners){
		assignment_matrix_(task.idx_, e) = 1;
	}
	if (assignment_matrix_(task.idx_, idx_) == 0){
		winning_bids_matrix_(task.idx_, idx_) = 0.0;
	}
}

/*** Find the K closest reward ***/
std::vector<double> CollaborativeAlgorithm::KClosestFinder(std::vector<double> ordered_bids, int x, int k, int n){
	std::vector<double> cloest_set;
	int l = CollaborativeAlgorithm::findCrossOver(ordered_bids, 0, n-1, x);
	int r = l + 1;
	int count = 0;

	if(ordered_bids[l] == x){
		cloest_set.push_back(ordered_bids[l]);
		count = count + 1;
		l--;
	}

	while (l >= 0 && r < n && count < k){
		if(x - ordered_bids[l] < ordered_bids[r] - x){
			cloest_set.push_back(ordered_bids[l--]);
		}
		else{
			cloest_set.push_back(ordered_bids[r++]);
		}
		count ++;
	}

	while (count < k && l >= 0){
		cloest_set.push_back(ordered_bids[l--]);
		count ++;
	}

	while (count < k && r < n){
		cloest_set.push_back(ordered_bids[r++]);
		count ++;
	}
	return cloest_set;
}

/*** Implement to find the K closest reward ***/
int CollaborativeAlgorithm::findCrossOver(std::vector<double> ordered_bids, int low, int high, int x){

	if (ordered_bids[high] <= x){
		return high;
	}
	if (ordered_bids[low] > x){
		return low;
	}

	int mid = (low + high)/2;

	if(ordered_bids[mid] <= x && ordered_bids[mid+1] > x){
		return mid;
	}

	if(ordered_bids[mid] < x){
		return CollaborativeAlgorithm::findCrossOver(ordered_bids, mid+1, high, x);
	}

	return CollaborativeAlgorithm::findCrossOver(ordered_bids, low, mid-1, x);
}

/*** Find the maximum bid with given list of bids ***/
double CollaborativeAlgorithm::maximum_bid_finder(std::vector<double> winning_bids){
	double max_bid = 0.0;
	for (auto &e:winning_bids){
		if(e > max_bid){
			max_bid = e;
		}
	}
	return max_bid;
}

/*** Find the minimum bid with given list of bids ***/
double CollaborativeAlgorithm::minimum_bid_finder(std::vector<double> winning_bids){
	double min_bid = RW_BENEFIT_;
	for (auto &e: winning_bids){
		if(e < min_bid){
			min_bid = e;
		}
	}
	return min_bid;
}

/*** Find the lists of index of vehicles with given list of bids ***/
std::vector<int> AutoVehicle::FindVehicleFrombid(Task task, std::vector<double> winners_bid){
	std::vector<int> winners;
	for (auto &bid: winners_bid){
		for (int i = 0; i < num_vehicles_; i++){
			std::vector<int>::iterator exist = std::find(winners.begin(), winners.end(), i);
			if(bid == winning_bids_matrix_(task.idx_, i) && winning_bids_matrix_(task.idx_, i) != 0.0 && exist == winners.end()){
				winners.push_back(i);
				break;
			}
		}
	}
	return winners;
}