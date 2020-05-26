#include "map/square_grid.hpp"
#include "cbba/cbba_agent.hpp"
#include "cbba/cbba_task.hpp"
#include "config_reader/config_reader.hpp"
#include "vis/graph_vis.hpp"
#include "gplib/log_likelihood_classification.hpp"

#include "auto_vehicle/auto_vehicle.hpp"
#include "auto_vehicle/tasks.hpp"

using namespace cv;
using namespace librav;

std::string SquareCell::GetCellLabels(){
    return cell_labels_.GetCellLabels();
}

int32_t SquareCell::GetCellBitMap(){
    return cell_labels_.GetCellBitMap();
}


SquareGrid::SquareGrid(int32_t row_num, int32_t col_num, double cell_size, int32_t pixel_per_meter, int32_t default_label):
                num_row_(row_num),
                num_col_(col_num),
                cell_size_(cell_size),
                pixel_per_meter_(pixel_per_meter){
    assert((row_num > 0 && col_num > 0));
    grid_cells_.resize(col_num);
    for (auto &grid_col : grid_cells_)
        grid_col.resize(row_num);
    for (int32_t y = 0; y < row_num; y++){
        for (int32_t x = 0; x < col_num; x++){
            int64_t new_id = y*col_num + x;
            SquareCell *new_cell = new SquareCell(new_id, x, y, OccupancyType::FREE);
            grid_cells_[x][y] = new_cell;
            grid_cells_[x][y]->UpdateCellInfo(num_row_, num_col_, cell_size_, pixel_per_meter_);
           
            grid_cells_[x][y]->cell_labels_.SetDefaultRegionLabel(default_label);
        }
    }
    obstacle_label_ = 1;
}

SquareGrid::~SquareGrid(){
    // Remove all Square cells
    for (auto &grid_col: grid_cells_){
        for (auto &cell: grid_col)
            delete cell;
    }
}

Position2i SquareGrid::GetCoordinateFromID(int64_t id){
    int32_t y = id/num_col_;
    int32_t x = id%num_col_;
    return Position2i(x,y);
}

void SquareGrid::SetCellOccupancy(int32_t x_col, int32_t y_row, OccupancyType occ){
    grid_cells_[x_col][y_row]->occupancy_ = occ;
    if(occ == OccupancyType::FREE){
        grid_cells_[x_col][y_row]->p_ = 0.0;
    }
    else if (occ == OccupancyType::OCCUPIED){
        grid_cells_[x_col][y_row]->p_ = 1.0;
    }
    else if (occ == OccupancyType::INTERESTED){
        grid_cells_[x_col][y_row]->p_ = 0.0;
    }
}

void SquareGrid::SetCellOccupancy(int64_t id, OccupancyType occ){
    Position2i coord = GetCoordinateFromID(id);
    SetCellOccupancy(coord.x, coord.y, occ);
}

int64_t SquareGrid::GetIDFromCoordinate(int32_t x_col, int32_t y_row)
{
    return y_row * num_col_ + x_col;
}

SquareCell* SquareGrid::GetCellFromID(int64_t id){
    Position2i coord = GetCoordinateFromID(id);
    return grid_cells_[coord.x][coord.y];
}

std::vector<SquareCell *> SquareGrid::GetNeighbors(int64_t id, bool allow_diag){
    auto cell = GetCellFromID(id);
    return GetNeighbors(cell->coordinate_.x, cell->coordinate_.y, allow_diag);
}


std::vector<SquareCell *> SquareGrid::GetNeighbors(int32_t x_col, int32_t y_row, bool allow_diag){
    std::vector<SquareCell *> neighbors;
    // Consider diagonal cells
    if (allow_diag){
        for (int32_t x = x_col - 1; x <= x_col + 1; ++x){
            for (int32_t y = y_row - 1; y <= y_row + 1; ++y){
                if (x == x_col && y == y_row)
                    continue;
                if (x >= 0 && x < num_col_ && y >= 0 && y < num_row_)
                    neighbors.push_back(grid_cells_[x][y]);
            }
        }
    }
    else{
        // Not consider diagonal cells
        Position2i pos[4];
        pos[0].x = x_col;
        pos[0].y = y_row + 1;
        pos[1].x = x_col;
        pos[1].y = y_row - 1;
        pos[2].x = x_col + 1;
        pos[2].y = y_row;
        pos[3].x = x_col - 1;
        pos[3].y = y_row;

        for (int i = 0; i < 4; i++){
            if (pos[i].x >= 0 && pos[i].x < num_col_ &&
                pos[i].y >= 0 && pos[i].y < num_row_){
                    neighbors.push_back(grid_cells_[pos[i].x][pos[i].y]);
            }
        }
    }
    return neighbors;
}

void SquareGrid::SetObstacleRegionLabel(int64_t id, int8_t label){
    this->SetCellOccupancy(id, OccupancyType::OCCUPIED);
    SquareCell* current_cell = this->GetCellFromID(id);
    current_cell->cell_labels_.RemoveRegionLabel(current_cell->cell_labels_.GetDefaultRegionLabel());
    current_cell->cell_labels_.AssignRegionLabel(label);
}

void SquareGrid::SetInterestedRegionLabel(int64_t id, int8_t label){
    this->SetCellOccupancy(id, OccupancyType::INTERESTED);
    SquareCell* current_cell = this->GetCellFromID(id);
    current_cell->cell_labels_.AssignRegionLabel(label);
}

std::shared_ptr<SquareGrid> GridGraph::CreateSquareGrid(int32_t row_size, int32_t col_size, double cell_size){
    return std::make_shared<SquareGrid>(row_size,col_size,cell_size);
}

std::shared_ptr<SquareGrid> GridGraph::CreateSquareGrid(int64_t row_size, int64_t col_size, double cell_size, std::shared_ptr<AutoTeam_t<AutoVehicle>> teams, TasksSet tasks){
    std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(row_size,col_size,cell_size);
    for (int i = 0; i < row_size * col_size; i++){
		grid->SetCellProbability(i,0.5);
	}
    for(auto&agent: teams->auto_team_){
        grid->SetCellOccupancy(agent->vehicle_.pos_, OccupancyType::FREE);
    }
    for(auto&tk: tasks.tasks_){
        for(auto& pp: tk.pos_){
            grid->SetInterestedRegionLabel(pp,tk.AP_label_);	
		    grid->SetCellOccupancy(pp,OccupancyType::INTERESTED);
        }
    }
    return grid;
}

std::shared_ptr<SquareGrid> GridGraph::CreateSquareGrid(std::string filename){
    std::string path_to_file = "../../src/config/" + filename + ".ini";
    ConfigReader config_reader(path_to_file);
    if (config_reader.CheckError()){
        std::cout << "Reading config file failed." << std::endl;
    }

    int32_t num_row = config_reader.GetReal("grid_row", 0);
    int32_t num_col = config_reader.GetReal("grid_column", 0);
    double cell_size = config_reader.GetReal("cell_size", 0.1);
    int32_t pixel_per_meter = config_reader.GetReal("pixel_per_meter", 100);
    int32_t default_label = config_reader.GetReal("default_label", 0);
    int32_t obstacle_label = config_reader.GetReal("obstacle_label",1);

    std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(num_row, num_col, cell_size, pixel_per_meter, default_label);
    grid->default_label_ = default_label;
    grid->obstacle_label_ = obstacle_label;

    int32_t num_obs_block = config_reader.GetReal("obs_block_num", 0);
    for (int i = 0; i < num_obs_block; i++){
        std::string obs_block_start = "obs"+std::to_string(i)+"_start";
        std::string obs_block_end = "obs"+std::to_string(i)+"_end";
        int32_t obs_start = config_reader.GetReal(obs_block_start, 0);
        int32_t obs_end = config_reader.GetReal(obs_block_end, 0);
        for (int j = obs_start; j < obs_end; j++){
            grid->SetObstacleRegionLabel(j,obstacle_label);
        }
    }
    int32_t num_obs_cell = config_reader.GetReal("obs_cell_num", 0);
    for(int i = 0; i < num_obs_cell; i++){
        std::string obs_cell_str = "obs"+std::to_string(i);
        int obs_cell = config_reader.GetReal(obs_cell_str, 0);
        grid->SetObstacleRegionLabel(obs_cell,obstacle_label);
    }
    return  grid;
}

std::shared_ptr<SquareGrid> GridGraph::CreateSquareGrid(){
    std::string filename = "true_map";
    return GridGraph::CreateSquareGrid(filename);
}

std::shared_ptr<Graph_t<SquareCell *>> GridGraph::BuildGraphFromSquareGrid(std::shared_ptr<SquareGrid> grid, bool allow_diag_move, bool ignore_obs){
    std::shared_ptr<Graph_t<SquareCell *>> graph = std::make_shared<Graph_t<SquareCell *>>();

    // DO consider obstacle
    if (ignore_obs == false){
        for (auto &cell_col : grid->grid_cells_){
		    for (auto &cell : cell_col){
			    if(cell->occupancy_ != OccupancyType::OCCUPIED){
                    int64_t current_nodeid = cell->id_;
			        std::vector<SquareCell *> neighbour_list = grid->GetNeighbors(current_nodeid, allow_diag_move);
                    graph->AddVertex(cell);
                    for (auto &neighbour : neighbour_list){
                        if(neighbour->occupancy_ != OccupancyType::OCCUPIED){
                            double cost = 0;
                            cost = 1.0 * (1.0 - neighbour->p_) + PENALTY_ * neighbour->p_;
                            graph->AddEdge(cell, neighbour, cost);
                            // std::cout << "Cell occupancy is " << cell->occupancy_ << std::endl;
                            // std::cout << "Neighbor occupancy is " << neighbour->occupancy_ << std::endl;
                            // std::cout << "Edge from cell " << cell->id_ << " to cell " << neighbour->id_ << " is added. " << std::endl;
                        }
                        // else{
                        //     graph->AddVertex(neighbour);
                        // }
                    }
                    
                }
                else{
                    graph->AddVertex(cell);
                }
		    }
        }
    }
    else{
        // DO NOT consider obstacle
        for (auto &cell_col : grid->grid_cells_){
		    for (auto &cell : cell_col){
                int64_t current_nodeid = cell->id_;
			    std::vector<SquareCell *> neighbour_list = grid->GetNeighbors(current_nodeid, allow_diag_move);
			    for (auto &neighbour : neighbour_list){
                    double cost = 0;
				    cost = 1.0 * (1.0 - neighbour->p_) + PENALTY_ * neighbour->p_;

				    graph->AddEdge(cell, neighbour, cost);
			    }		
		    }
        }
    }
	return graph;
}

void SquareGrid::SetCellProbability(int64_t id, double p){
    Position2i vertex_coordinate = GetCoordinateFromID(id);
    grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->p_ = p;
   
    if (grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->p_ == 0.0){
        grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->occupancy_ = OccupancyType::FREE;
    }
    else if(grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->p_ == 1.0){
        SetObstacleRegionLabel(id, obstacle_label_);
    }
    else{
        grid_cells_[vertex_coordinate.x][vertex_coordinate.y]->occupancy_ = OccupancyType::UNKNOWN;
    }
}

void SquareCell::ComputeIG(std::shared_ptr<SquareGrid> grid, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> subRegion){
    Vertex_t<SquareCell*>* vertex_ = graph->GetVertexFromID(id_);
    std::vector<Vertex_t<SquareCell*>*> neighbors = vertex_->GetNeighbours();
    // Determine the list of vertices which are unknown from current vertex_'s neighbors
    std::vector<Vertex_t<SquareCell*>*> N_v_;
    for(auto& neigb: neighbors){
        if(neigb->state_->occupancy_ == OccupancyType::UNKNOWN){
            N_v_.push_back(neigb);
        }
    }
    if(vertex_->state_->occupancy_ == OccupancyType::UNKNOWN){N_v_.push_back(vertex_);};

    // Compute the map of possible configuration for the N_v_
    std::vector<std::map<int64_t, int>> binary_configs_ = GridGraph::PotentialConfiguration(N_v_);

    double new_entropy = 0.0;
    for(auto &single_config_: binary_configs_){
        double entropy_potential = GridGraph::EntropyPotentialConfig(subRegion,grid,single_config_,id_,false);
        double config_probability = GridGraph::MeasurementProbability(graph, single_config_);
        new_entropy += entropy_potential * config_probability;
    }

    // Compute the origin entropy from origin graph
    double original_entropy = GridGraph::EntropyCalculation(subRegion, graph);

    // Compute the ig 
    ig_ = original_entropy - new_entropy;
}

// Given a list of unknown vertices, compute the list of possible configuration
std::vector<std::map<int64_t, int>> GridGraph::PotentialConfiguration(std::vector<Vertex_t<SquareCell*>*> N_v_){
    int num_confg = std::pow(2, N_v_.size());
    std::vector<std::map<int64_t,int>> binary_config_;
    for (int i = 0; i < num_confg; i++){
        int single_case_ = i;
        std::vector<int> config_ = {};
        while(single_case_ > 0){
            config_.insert(config_.begin(), single_case_%2);
            single_case_ = single_case_/2;
        }
        while (config_.size() < N_v_.size()){
            config_.insert(config_.begin(),0);
        }
        std::map<int64_t, int> single_config_ = {};
        for(int j = 0; j < N_v_.size(); j++){
            single_config_[N_v_[j]->state_->id_] = config_[j];
        }
        binary_config_.push_back(single_config_);
    } 
    return binary_config_;
}

// Compute the entropy with given potential configuration
double GridGraph::EntropyPotentialConfig(std::vector<int64_t> subRegion, std::shared_ptr<SquareGrid> grid, std::map<int64_t, int> config_, int64_t sensor_pos_, bool allow_diag_move){
    // Create a duplicate grid 
    std::shared_ptr<SquareGrid> duplicated_grid = std::make_shared<SquareGrid>(grid->num_row_,grid->num_col_,grid->cell_size_);
    for (int row = 0; row < grid->grid_cells_.size(); row++){
        for(int col = 0; col < grid->grid_cells_[row].size(); col++){
            if(grid->grid_cells_[row][col]->occupancy_ == OccupancyType::INTERESTED){
                std::vector<int32_t> ROIs_labels = grid->grid_cells_[row][col]->cell_labels_.GetCellLabelID();
                for(auto &roi_lb: ROIs_labels){
                    duplicated_grid->SetInterestedRegionLabel(grid->grid_cells_[row][col]->id_, roi_lb);
                }
            }
            else if (grid->grid_cells_[row][col]->occupancy_ == OccupancyType::OCCUPIED){
                duplicated_grid->SetObstacleRegionLabel(grid->grid_cells_[row][col]->id_, grid->obstacle_label_);
            }
            else if (grid->grid_cells_[row][col]->occupancy_ == OccupancyType::UNKNOWN){
                duplicated_grid->SetCellOccupancy(grid->grid_cells_[row][col]->id_, OccupancyType::UNKNOWN);
                duplicated_grid->SetCellProbability(grid->grid_cells_[row][col]->id_, grid->grid_cells_[row][col]->p_);
            }
        }
    }

    for(auto& un_cell: config_){
        double previous_bel = duplicated_grid->GetCellFromID(un_cell.first)->p_;
        // If measurement is obstacle
        if (un_cell.second == 1){
            if (un_cell.first != sensor_pos_){
                double bel_obs = GridGraph::UpdateBelProbability(previous_bel,1);
                duplicated_grid->SetCellProbability(un_cell.first, bel_obs);
            }
            else {
                duplicated_grid->SetObstacleRegionLabel(un_cell.first, grid->obstacle_label_);
                duplicated_grid->SetCellProbability(un_cell.first, 1.0);
            }
        }
        else{
            // Q: Should we update the edge here??
            // If the measurement is free
            if(un_cell.first != sensor_pos_){
                double bel_free = GridGraph::UpdateBelProbability(previous_bel,0);
                duplicated_grid->SetCellProbability(un_cell.first, bel_free);
            }
            else{
                duplicated_grid->SetCellOccupancy(un_cell.first, OccupancyType::FREE);
                duplicated_grid->SetCellProbability(un_cell.first, 0.0);
            }
        }
    }

 

    // Build the graph corresponding to the duplicated grid
    std::shared_ptr<Graph_t<SquareCell *>> duplicated_graph = GridGraph::BuildGraphFromSquareGrid(duplicated_grid, allow_diag_move, true);
    double entropy_ = GridGraph::EntropyCalculation(subRegion, duplicated_graph);
    return entropy_;
}

double GridGraph::UpdateBelProbability(double p, int status){
    // Default bel(m=obs)
    double bel;

    double eta_reciprocal = 1.0;
    // If the actual status is obstacle
    if(status == 1){
        eta_reciprocal = sensor_accuracy * p + (1.0 - sensor_accuracy) * (1.0 - p);
    }
    else if (status == 0){
        eta_reciprocal = (1.0 - sensor_accuracy) * p + sensor_accuracy * (1.0 - p);
    }
    else{
        std::cout << "Unknown configuration case. " << std::endl;
    }

    if(status == 1){
        bel = sensor_accuracy * p / eta_reciprocal;
    }
    else if (status == 0){
        bel = (1.0 - sensor_accuracy) * p / eta_reciprocal;
    }

    return bel;
}

double GridGraph::MeasurementProbability(std::shared_ptr<Graph_t<SquareCell*>> graph, std::map<int64_t, int> config){
    double measurement_p = 1.0;
    for(auto &un_cell: config){
        Vertex_t<SquareCell*>* un_v = graph->GetVertexFromID(un_cell.first);
        double un_p = 0.0;
        // If the measurement is obstacle
        if(un_cell.second == 1){
            un_p = sensor_accuracy * un_v->state_->p_ + (1.0 - sensor_accuracy) * (1.0 - un_v->state_->p_);
        }
        else if(un_cell.second == 0){
            un_p = (1.0 - sensor_accuracy) * un_v->state_->p_ + sensor_accuracy * (1.0 - un_v->state_->p_);
        }
        else{
            std::cout << "Unknown measurement configuration. " << std::endl;
        }
        measurement_p = measurement_p * un_p;
    }
    return measurement_p;
}

// Compuet entropy
double GridGraph::EntropyCalculation(std::vector<int64_t> subRegion, std::shared_ptr<Graph_t<SquareCell *>> graph){
    double entropy_ = 0.0;
    for (auto& cell_: subRegion){
        Vertex_t<SquareCell *>* vertex_ = graph->GetVertexFromID(cell_);
        double p = vertex_->state_->p_;
        double q = 1.0 - p;
        if (p == 0.0 || q == 0.0){
            entropy_ += 0.0;
        }
        else{
            entropy_ += -p * log10(p) - q * log10(q);
        }
    }
    return entropy_;
}

// Compute the entropy of whole graph
double GridGraph::EntropyMap(std::shared_ptr<Graph_t<SquareCell*>> graph){
    int64_t num_vertex = graph->GetGraphVertexNumber();
    std::vector<int64_t> id_coll;
    for(int i = 0; i < num_vertex; i++){
        id_coll.push_back(i);
    }
    double entropy_ = GridGraph::EntropyCalculation(id_coll, graph);
    return entropy_;
}

double GridGraph::EntropyMap(std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<int64_t> measurements){
    int64_t num_vertex = graph->GetGraphVertexNumber();
    double entropy_ = GridGraph::EntropyCalculation(measurements,graph);
    int64_t num_vertex_pred = num_vertex - measurements.size();
    for(int ii = 0; ii < num_vertex_pred; ii++){
        entropy_ += -0.5 * log10(0.5) - 0.5 * log10(0.5);
    }
    return entropy_;
}
// Compute the entropy of path
double GridGraph::EntropyPath(Path_t<SquareCell*> path, std::shared_ptr<Graph_t<SquareCell*>> graph){
    std::vector<int64_t> id_coll;
    for(auto& cell: path){
        id_coll.push_back(cell->id_);
    }
    double entropy_ = GridGraph::EntropyCalculation(id_coll, graph);
    return entropy_;
}

std::vector<int64_t> GridGraph::SubRegionFromPaths(std::vector<Path_t<SquareCell*>> paths, std::shared_ptr<SquareGrid> grid, std::shared_ptr<Graph_t<SquareCell*>> graph){
    std::vector<int64_t> subRegion;
    for(auto& segment: paths){
        // Add the cells along current routes into sub-region
        std::vector<int64_t> segment_id = {};
  
        for(auto& cell: segment){
            std::vector<int64_t>::iterator it = std::find(subRegion.begin(), subRegion.end(), cell->id_);
            if(it == subRegion.end()){
                subRegion.push_back(cell->id_);
            }
            segment_id.push_back(cell->id_);
        }
        
        // Looking for cells along potential routes
        for(auto & cell: segment){
            // Find a set of vertices 
            Vertex_t<SquareCell*>* vert_ = graph->GetVertexFromID(cell->id_);
            std::vector<Vertex_t<SquareCell*>*> neighbors = vert_->GetNeighbours();
            std::vector<Vertex_t<SquareCell*>*> unknown_verts;
            for(auto& neigb: neighbors){
                if(neigb->state_->occupancy_ == OccupancyType::UNKNOWN){
                    unknown_verts.push_back(neigb);
                }
            }
            if (vert_->state_->occupancy_ == OccupancyType::UNKNOWN){
                unknown_verts.push_back(vert_);
            }

            // Find the index of unknown verts along the current segment
            std::map<int64_t,Vertex_t<SquareCell*>*> un_v_segment;
            if(!unknown_verts.empty()){
                for(auto& un_v:unknown_verts){
                    std::vector<int64_t>::iterator it_segm = std::find(segment_id.begin(), segment_id.end(), un_v->state_->id_);
                    if(it_segm != segment_id.end()){
                        int64_t v_idx = it_segm - segment_id.begin();
                        un_v_segment[v_idx] = un_v;
                    }
                }
            
                // Compute the src and dst vertex along the segment
                if (!un_v_segment.empty()){
                    int64_t init_idx = un_v_segment.begin()->first;
                    int64_t start_idx = 0;
                    if(init_idx == 0){
                        start_idx = 0;
                    }
                    else{
                        start_idx = init_idx - 1;
                    }
                    int64_t start_id = segment_id[start_idx];

                    int64_t end_idx = un_v_segment.rbegin()->first;
                    int64_t finish_idx = segment_id.size() - 1;
                    if(end_idx == finish_idx){
                        continue;
                    }
                    else{
                        finish_idx = end_idx + 1;
                    }
                    int64_t finish_id = segment_id[finish_idx];
                   

                    // Compute the repaired route from src and dst
                    std::vector<std::map<int64_t, int>> binary_config = GridGraph::PotentialConfiguration(unknown_verts);
                    for(auto& potential_binary: binary_config){
                        std::vector<int64_t> region_ = GridGraph::SubRegionComputation(grid,potential_binary,vert_->state_->id_,start_id,finish_id,false);
                        for(auto& cell_: region_){
                            std::vector<int64_t>::iterator it_reg = std::find(subRegion.begin(), subRegion.end(), cell_);
                            if(it_reg == subRegion.end()){
                                subRegion.push_back(cell_);
                            }
                        }
                    }
                }
                else{
                    break;
                }
            }
        }
    }
    return subRegion;
}

std::shared_ptr<SquareGrid> SquareGrid::DuplicateSquareGrid(){
    std::shared_ptr<SquareGrid> duplicated_grid = std::make_shared<SquareGrid>(num_row_,num_col_,cell_size_);
    for(size_t ii = 0; ii < grid_cells_.size(); ii++){
        for(size_t jj = 0; jj < grid_cells_.size(); jj++){
            SquareCell* vert = duplicated_grid->grid_cells_[ii][jj];
            vert->p_ = grid_cells_[ii][jj]->p_;
            vert->ig_ = grid_cells_[ii][jj]->ig_;
            vert->occupancy_ = grid_cells_[ii][jj]->occupancy_;
            vert->lifted_vertices_id_ = grid_cells_[ii][jj]->lifted_vertices_id_;
            vert->cell_labels_ = grid_cells_[ii][jj]->cell_labels_;
        }
    }
    return duplicated_grid;
}

std::vector<int64_t> GridGraph::SubRegionComputation(std::shared_ptr<SquareGrid> grid, std::map<int64_t, int> config, int64_t sensor_pos, int64_t start_id, int64_t end_id, bool allow_diag_move){
     // Create a duplicate grid 
    std::shared_ptr<SquareGrid> duplicated_grid = grid->DuplicateSquareGrid();

    for(auto& un_cell: config){
        double previous_bel = duplicated_grid->GetCellFromID(un_cell.first)->p_;
        // If measurement is obstacle
        if (un_cell.second == 1){
            if (un_cell.first != sensor_pos){
                double bel_obs = GridGraph::UpdateBelProbability(previous_bel,1);
                duplicated_grid->SetCellProbability(un_cell.first, bel_obs);
            }
            else {
                duplicated_grid->SetObstacleRegionLabel(un_cell.first, grid->obstacle_label_);
                duplicated_grid->SetCellProbability(un_cell.first, 1.0);
            }
        }
        else{
            // Q: Should we update the edge here??
            // If the measurement is free
            if(un_cell.first != sensor_pos){
                double bel_free = GridGraph::UpdateBelProbability(previous_bel,0);
                duplicated_grid->SetCellProbability(un_cell.first, bel_free);
            }
            else{
                duplicated_grid->SetCellOccupancy(un_cell.first, OccupancyType::FREE);
                duplicated_grid->SetCellProbability(un_cell.first, 0.0);
            }
        }
    }
    // Build the graph corresponding to the duplicated grid
    std::shared_ptr<Graph_t<SquareCell *>> duplicated_graph = GridGraph::BuildGraphFromSquareGrid(duplicated_grid, allow_diag_move, false);
    Path_t<SquareCell*> path_ = AStar::Search(duplicated_graph,start_id,end_id,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristic));
    std::vector<int64_t> region_ = {};
    if(!path_.empty()){
        for(auto& cell:path_){
            region_.push_back(cell->id_);
        }
    }
    return region_;
}


double GridGraph::CalcHeuristic(SquareCell *node1, SquareCell *node2){
    int64_t dist_row = node1->coordinate_.x - node2->coordinate_.x;
    int64_t dist_col = node1->coordinate_.y - node2->coordinate_.y;
    return std::sqrt(dist_row*dist_row + dist_col*dist_col);
}

double GridGraph::CalcHeuristicUncertain(SquareCell *node1, SquareCell *node2){
    return 1.0*(1.0 - node2->p_) + PENALTY_ * node2->p_;
}

Path_t<SquareCell *> GridGraph::PathComputationAStar(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> task_path_, int64_t init_pos_){
    Vertex_t<SquareCell*> * start_node_;
    Vertex_t<SquareCell*> * end_node_;
    int64_t final_base_;
    Path_t<SquareCell *> path_;

    // Determine the list of targets vehicle needs to visit
    std::vector<int64_t> targets_sequence_;
    for(auto& target_: task_path_){
        cbba_Task task = tasks.FindTaskFromID(target_);
        if(task.type_ == TasksType::VISIT){
            if(task.pos_.size() == 1){
                targets_sequence_.push_back(task.pos_.front());
            }
            else{
                for(auto& sub_target_: task.pos_){
                    targets_sequence_.push_back(sub_target_);
                }
            }
        }
        else if (task.type_ == TasksType::SEARCH){
            if(task.pos_.size() != task.buchi_regions_.size()){
                targets_sequence_.push_back(task.pos_.front());
            }
            else{
                final_base_ = task.pos_.back();
                targets_sequence_.push_back(task.pos_.front());
            }
        }
        else{
            std::cout << "Unknown task type. " << std::endl;
        }
    }

    start_node_ = graph->GetVertexFromID(init_pos_);
    for(int i = 0; i < targets_sequence_.size(); i++){
        end_node_ = graph->GetVertexFromID(targets_sequence_[i]);
        Path_t<SquareCell *> path_part_ = AStar::Search(graph,start_node_->state_->id_,end_node_->state_->id_,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristic));
        if(i == targets_sequence_.size() - 1){
            path_.insert(path_.end(), path_part_.begin(), path_part_.end());
        }
        else{
            path_.insert(path_.end(), path_part_.begin(), path_part_.end()-1);
        }
        start_node_ = end_node_;
    }
    return path_;
}


std::pair<bool, std::vector<double>> GridGraph::ConvergenceCheck(std::map<int64_t, Path_t<SquareCell*>> paths, std::shared_ptr<Graph_t<SquareCell*>> graph, double threshold){
    bool flag = true;
    std::pair<bool,std::vector<double>> flag_entropy_pair;
    std::vector<bool> flag_paths_;
    std::vector<double> entropy_paths_;
    // std::cout << "The entropy threshold is " << threshold << std::endl;
    for(std::map<int64_t, Path_t<SquareCell*>>::iterator it = paths.begin(); it != paths.end(); it++){
        if(it->second.empty()){
            flag_paths_.push_back(true);
            entropy_paths_.push_back(0.0);
        }
        else{
            double en_thre = it->second.size() * threshold;
            double entropy_path = GridGraph::EntropyPath(it->second, graph);
            // std::cout << "en_thre is " << en_thre << " , entropy of path is " << entropy_path << std::endl;
            if(entropy_path < en_thre){
                flag_paths_.push_back(true);
            }
            else{
                flag_paths_.push_back(false);
            }
            entropy_paths_.push_back(entropy_path);
        }
    }

    for (auto fs = flag_paths_.begin(); fs != flag_paths_.end(); fs++){
        if (*fs == false){
            flag = false;
            break;
        }
    }
    flag_entropy_pair = std::make_pair(flag, entropy_paths_);
    return flag_entropy_pair;
}

std::vector<int64_t> GridGraph::SelectSensorsPos(int64_t num_sensors, std::shared_ptr<Graph_t<SquareCell *>> graph){
    std::vector<int64_t> sensors_pos_ = {};
    // Get all vertices
    std::vector<Vertex_t<SquareCell*>*> total_vs = graph->GetAllVertex();
    // Set the multimap <ig, id> for all vertices in the map
    std::multimap<double, int64_t> ig_map;
    for(auto& v: total_vs){
        ig_map.insert(std::pair<double,int64_t>(v->state_->ig_, v->state_->id_));
    }
    
    double max_ig = ig_map.rbegin()->first;
    std::vector<int64_t> list_max_ig = {};

    std::multimap<double,int64_t>::reverse_iterator rit;
    for(rit = ig_map.rbegin(); rit != ig_map.rend(); rit++){
        if(std::fabs(rit->first - max_ig) <= 0.0001){
            list_max_ig.push_back(rit->second);
        }
        else{
            // Right now all vertices with maximum ig have been collected
            if(list_max_ig.size() >= num_sensors){
                break;
            }
            else{
                for(auto& cell: list_max_ig){
                    std::vector<int64_t>::iterator it = std::find(sensors_pos_.begin(), sensors_pos_.end(), cell);
                    if(it == sensors_pos_.end()){
                        sensors_pos_.push_back(cell);
                    }
                }
                max_ig = rit->first;
                list_max_ig.push_back(rit->second);
            }
        }
    }
    // MIN_MIN MISS probability
    if (sensors_pos_.empty()){
        sensors_pos_.push_back(list_max_ig.front());
    }
    while(sensors_pos_.size() < num_sensors){
        int64_t sensor_id = GridGraph::SensorPosFromMissProbability(sensors_pos_,list_max_ig, graph);
        sensors_pos_.push_back(sensor_id);    
    }
    return sensors_pos_;
}

std::vector<int64_t> GridGraph::SelectSensorsPos(int64_t num_sensors,std::vector<int64_t> sensors,std::shared_ptr<Graph_t<SquareCell *>> graph){
    std::vector<int64_t> sensors_pos_ = {};
    // Get all vertices
    std::vector<Vertex_t<SquareCell*>*> total_vs = graph->GetAllVertex();
    // Set the multimap <ig, id> for all vertices in the map
    std::multimap<double, int64_t> ig_map;
    for(auto& v: total_vs){
        if (!sensors.empty()){
            std::vector<int64_t>::iterator it_v = std::find(sensors.begin(), sensors.end(), v->state_->id_);
            if(it_v == sensors.end()){
                ig_map.insert(std::pair<double,int64_t>(v->state_->ig_, v->state_->id_));
            }
        }
        else{
            ig_map.insert(std::pair<double,int64_t>(v->state_->ig_, v->state_->id_));
        }
    }
    
    double max_ig = ig_map.rbegin()->first;
    std::vector<int64_t> list_max_ig = {};

    std::multimap<double,int64_t>::reverse_iterator rit;
    for(rit = ig_map.rbegin(); rit != ig_map.rend(); rit++){
        if(std::fabs(rit->first - max_ig) <= 0.0001){
            list_max_ig.push_back(rit->second);
        }
        else{
            // Right now all vertices with maximum ig have been collected
            if(list_max_ig.size() >= num_sensors){
                break;
            }
            else{
                for(auto& cell: list_max_ig){
                    std::vector<int64_t>::iterator it = std::find(sensors_pos_.begin(), sensors_pos_.end(), cell);
                    if(it == sensors_pos_.end()){
                        sensors_pos_.push_back(cell);
                    }
                }
                max_ig = rit->first;
                list_max_ig.push_back(rit->second);
            }
        }
    }
    // MIN_MIN MISS probability
    if (sensors_pos_.empty()){
        sensors_pos_.push_back(list_max_ig.front());
    }
    while(sensors_pos_.size() < num_sensors){
        int64_t sensor_id = GridGraph::SensorPosFromMissProbability(sensors_pos_,list_max_ig, graph);
        sensors_pos_.push_back(sensor_id);    
    }
    return sensors_pos_;
}


int64_t GridGraph::SensorPosFromMissProbability(std::vector<int64_t> sensors_pos_,std::vector<int64_t> list_max_ig, std::shared_ptr<Graph_t<SquareCell*>> graph){
    int64_t size_cells = list_max_ig.size();
    std::map<int64_t, double> cell_m_tilde_;

    // Update current miss probability matrix
    std::map<int64_t, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> miss_matrix_sensors;
    for(auto& cell: sensors_pos_){
        Vertex_t<SquareCell *>* v_ = graph->GetVertexFromID(cell);
        std::vector<int64_t> neighbors_id = v_->GetNeighbourIDs();

        Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> miss_p_ = Eigen::MatrixXd::Ones(1, size_cells);
        std::vector<int64_t>::iterator it1 = std::find(list_max_ig.begin(), list_max_ig.end(), cell);
        if(it1 != list_max_ig.end()){
            int64_t c_idx = it1 - list_max_ig.begin();
            miss_p_(0,c_idx) = 0.0;
            for(auto& n_idx: neighbors_id){
                std::vector<int64_t>::iterator it2 = std::find(list_max_ig.begin(), list_max_ig.end(), n_idx);
                if(it2 == list_max_ig.end()){
                    continue;
                }
                else{
                    int64_t neighbor_idx = it2 - list_max_ig.begin();
                    miss_p_(0, neighbor_idx) = 1.0 - sensor_accuracy;
                }
            }
        }
        miss_matrix_sensors[cell] = miss_p_;
    }

    for(int idx1 = 0; idx1 < list_max_ig.size(); idx1++){
        std::vector<int64_t>::iterator it3 = std::find(sensors_pos_.begin(), sensors_pos_.end(), list_max_ig[idx1]);
        if(it3 != sensors_pos_.end()){
            cell_m_tilde_[list_max_ig[idx1]] = 1000.0;
        }
        else{
            Vertex_t<SquareCell*>* current_v_ = graph->GetVertexFromID(list_max_ig[idx1]);
            std::vector<int64_t> neighbors = current_v_->GetNeighbourIDs();
            // Define the miss probability matrix
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> miss_matrix_cell = Eigen::MatrixXd::Ones(1, size_cells);
            miss_matrix_cell(0, idx1) = 0.0;
            for(auto& idx2 : neighbors){
                std::vector<int64_t>::iterator it4 = std::find(list_max_ig.begin(), list_max_ig.end(),idx2);
                if(it4 == list_max_ig.end()){
                    continue;
                }
                else{
                    int64_t n_idx1 = it4 - list_max_ig.begin();
                    miss_matrix_cell(0,n_idx1) = 1.0 - sensor_accuracy;
                }
            }
            // Compute the collective miss probability for all vertices
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> collective_miss_matrix_ = Eigen::MatrixXd::Ones(1, size_cells);

            for(int c_idx = 0; c_idx < list_max_ig.size(); c_idx++){
                double collective_p = 1.0;
                for(auto& sensor: miss_matrix_sensors){
                    collective_p = collective_p * sensor.second(0,c_idx);
                }
                collective_p = collective_p * miss_matrix_cell(0, c_idx);
                collective_miss_matrix_(0, c_idx) = collective_p;
            }

            double m_tilde = 0.0;
            for(int cell_idx = 0; cell_idx < list_max_ig.size(); cell_idx++){
                m_tilde = m_tilde + collective_miss_matrix_(0, cell_idx) * miss_matrix_cell(0, cell_idx);
            }

            cell_m_tilde_[list_max_ig[idx1]] = m_tilde;
        }
    }

    // Find the cell in list of cells with minium m_tilde
    double min_m_tilde = 1000.0;
    int64_t selected_sensor = 0;
    for(auto& sensor_candidate: cell_m_tilde_){
        if(sensor_candidate.second <= min_m_tilde){
            min_m_tilde = sensor_candidate.second;
            selected_sensor = sensor_candidate.first;
        }
    }

    return selected_sensor;
}

void GridGraph::UpdateUncertainMap(std::vector<int64_t> sensor_pos, std::shared_ptr<Graph_t<SquareCell *>> graph, std::shared_ptr<Graph_t<SquareCell *>> true_graph){
    for (auto& sensor: sensor_pos){
        Vertex_t<SquareCell*>* true_v = true_graph->GetVertexFromID(sensor);
        Vertex_t<SquareCell*>* uncertain_v = graph->GetVertexFromID(sensor);
        // Update the occupancy
        uncertain_v->state_->occupancy_ = true_v->state_->occupancy_;
        if(true_v->state_->occupancy_ == OccupancyType::FREE || true_v->state_->occupancy_ == OccupancyType::INTERESTED){
            uncertain_v->state_->p_ = 0.0;
        }
        else if(true_v->state_ ->occupancy_ == OccupancyType::OCCUPIED){
            uncertain_v->state_->p_ = 1.0;
        }

        std::vector<Vertex_t<SquareCell*>*> neighbors = uncertain_v->GetNeighbours();
        for(auto& neigb: neighbors){
            Vertex_t<SquareCell*>* true_neigb = true_graph->GetVertexFromID(neigb->state_->id_);
            if(neigb->state_->occupancy_ == OccupancyType::UNKNOWN){
                if(true_neigb->state_->occupancy_ == OccupancyType::FREE || true_neigb->state_->occupancy_ == OccupancyType::INTERESTED){
                    double belif_p = GridGraph::UpdateBelProbability(neigb->state_->p_,0);
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
    GridGraph::UpdateUncertainGraphEdgeCost(graph);
}

void GridGraph::UpdateUncertainGraphEdgeCost(std::shared_ptr<Graph_t<SquareCell*>> graph){
    auto edges = graph->GetAllEdges();
    for (auto& e: edges){
        Vertex_t<SquareCell*>* dst_v = e->dst_;
        double uncertain_cost = 1.0 * (1.0 - dst_v->state_->p_) + PENALTY_*dst_v->state_->p_;
        e->cost_ = uncertain_cost;
        // remove the edge points to obstacle
        if(dst_v->state_->p_ == 1.0){
            graph->RemoveEdge(e->src_->state_,e->dst_->state_);
        }
    }
}

// Ignore the prob of the occupancy for GP map
void GridGraph::UpdateUncertainGPGraphEdgeCost(std::shared_ptr<Graph_t<SquareCell*>> graph){
    auto edges = graph->GetAllEdges();
    for (auto& e: edges){
        Vertex_t<SquareCell*>* dst_v = e->dst_;
        double uncertain_cost = 1.0;
        e->cost_ = uncertain_cost;
        // remove the edge points to obstacle
        if(dst_v->state_->p_ > 0.5){
            e->cost_ = 1000.0;
        }
    }
}

//====================================================================================================//
//====================================================================================================//
//====================================================================================================//
// Generate random occupancy grid map
void SquareGrid::SetRandomOccupanyMap(double obstacle_percentage){
    srand(time(NULL));

    int num_obstacle = (int)round(num_col_ * num_row_ * obstacle_percentage);
    
    // Generate the obstacles
    std::vector<int64_t> obstacles_ = {};
    while(obstacles_.size() < num_obstacle){
        int64_t idx = rand()%(num_col_ * num_row_);
        std::vector<int64_t>::iterator it = std::find(obstacles_.begin(), obstacles_.end(), idx);
        if(it == obstacles_.end()){
            obstacles_.push_back(idx);
        }
    }
    
    for(auto& obs_idx:obstacles_){
        // std::cout << "Set vertex " << obs_idx << " be occupied."<< std::endl;
        SetObstacleRegionLabel(obs_idx, obstacle_label_);
        SetCellProbability(obs_idx, 1.0);
        // auto cell = GetCellFromID(obs_idx);
        // if(cell->occupancy_ == OccupancyType::OCCUPIED){
        //     // std::cout << "And Yes. vertex " << cell->id_ << " is set to be occupied" << std::endl;
        // }
        // else{
        //     // std::cout << "No. Vertex " << cell->id_ << "has not been set to be occupied. " << std::endl;
        // }
    }
    
}

// Generate the initial position of agents randomly
std::vector<Agent> SquareGrid::SetRandomAgents(int num_Agents, int num_Tasks_inde, int num_Tasks_de){
    srand(time(NULL));

    std::vector<int64_t> agents_cell = {};
    while(agents_cell.size() < num_Agents){
        int64_t idx = rand()%(num_col_ * num_row_);
        SquareCell* vertex_agent = GetCellFromID(idx);
        if(vertex_agent->occupancy_ == OccupancyType::FREE){
            agents_cell.push_back(idx);
        }
    }
    
    std::vector<Agent> agents_ = {};
    for(int idx=0; idx < num_Agents; idx++){
        Agent agent = Agent(idx,agents_cell[idx],num_Tasks_inde,num_Tasks_de,num_Agents);
        agent.comm_topo_ = Eigen::MatrixXd::Ones(1, num_Agents);
        agents_.push_back(agent);
    }

    return agents_;
}

// Generate the task position randomly
TasksList SquareGrid::SetRandomTasks(int num_Tasks_inde, int num_Tasks_de){
    srand(time(NULL));
    TasksList missions = TasksList();
    missions.tasks_ = {};

    std::vector<int> tasks_cell = {};
    while(tasks_cell.size() < num_Tasks_inde){
        int64_t t_idx = rand()%(num_col_ * num_row_);
        SquareCell* task_cell = GetCellFromID(t_idx);
        std::vector<int>::iterator it_t = std::find(tasks_cell.begin(), tasks_cell.end(), t_idx);
        if(task_cell->p_ != 1.0 && it_t == tasks_cell.end()){
            tasks_cell.push_back(t_idx);
        } 
    }
 


    for(int task_idx = 0; task_idx < num_Tasks_inde; task_idx++){
        std::vector<int64_t> pos = {tasks_cell[task_idx]};
        TasksType type = TasksType::VISIT;
        int num_agents = 1;
        std::string liveness = "(<> p" + std::to_string(task_idx+2) + ")";
        std::string safety = "([] p0) && ([] !p1)";
        std::vector<std::string> buchi_regions = {liveness};

        cbba_Task task = cbba_Task(task_idx,pos,type,num_agents,liveness,safety,buchi_regions);
        SetInterestedRegionLabel(pos.front(),task_idx+2);
        missions.tasks_.push_back(task);
    
    }
    missions.specification_safety_ = "([] p0) && ([] !p1)";
    return missions;
}


bool SquareGrid::OccupancyGridMapValidity(std::vector<Agent> agents, TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph){
    bool flag = true;

    for(auto task: tasks.tasks_){
        for(auto& agent: agents){
            int64_t start_id = agent.init_pos_;
            int64_t end_id = task.pos_.front();
            Path_t<SquareCell*> path = AStar::Search(graph,start_id,end_id,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristic));
            
            if(path.empty()){
                // std::cout << "Doesn't find any path from vertex " << start_id << " to vertex " << end_id << std::endl; 
                flag = false;
                break;
            }
            // else{
            //     std::cout << "The path length from vertex " << path[0]->id_ << " to vertex " << path.back()->id_ << " is " << path.size() << std::endl;
            //     for(auto&kk: path){
            //         std::cout << kk->id_ << "-> ";
            //     }
            //     std::cout << std::endl;
            // }
        }
        if(flag == false){
            break;
        }
    }

    return flag;
}

bool SquareGrid::OccupancyGridMapValidity(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team, TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph){
    bool flag = true;
    for(auto tk: tasks.tasks_){
        for(auto agent: vehicle_team->auto_team_){
            if(agent->vehicle_.vehicle_type_ == TaskType::RESCUE){
                int64_t start_id = agent->vehicle_.pos_;
                int64_t end_id = tk.pos_.front();
                Path_t<SquareCell*> path = AStar::Search(graph,start_id,end_id,CalcHeuristicFunc_t<SquareCell *>(GridGraph::CalcHeuristic));
                if (path.empty()){
                    flag = false;
                    return flag;
                }
            }
        }
    }
    return flag;
}


IGdata GridGraph::IPAS(std::vector<Agent> agents, TasksList tasks, int num_sensors, std::shared_ptr<SquareGrid> grid, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<SquareCell*>> true_graph,double threshold){
    IGdata ig_data_;

    /**************************************************************************/
	/********************************* IPAS ***********************************/
	/**************************************************************************/
    // The flag of task assignment
	bool Flag_ts = 0;
	// The flag of ipas
	bool Flag_ipas = 0;
	
	int T = 0;
    // Compute the initial entropy
    ig_data_.entropy_map_init_ = GridGraph::EntropyMap(graph);

	clock_t cbba_time;
	cbba_time = clock();
	std::vector<double> entropy_trend;
	std::vector<int64_t> range_trend;
	std::map<int64_t, std::vector<double>> entropy_trend_paths;
    // while(T < 1){
	while(Flag_ipas == false){
		T++;
		/***************************************************************************************/
		/********************************** Task Assignment ************************************/
		/***************************************************************************************/
		TaskAssignment::ResetTaskAssignmentInfo(agents);
		Flag_ts = 0;
	
		while (Flag_ts != 1){
			for (int i = 0; i < agents[0].num_agents_; i++)
				agents[i].iter_ = 0;

			/*** 6. Communication among neighbors ***/
			TaskAssignment::communicate(agents);
			//Update the history of iteration neighbors
			for (int i = 0; i < agents[0].num_agents_; i++)
				agents[i].history_.iter_neighbors_his.push_back(agents[i].iteration_neighbors_);
			
			/*** 7. Bundle Operations ***/
			/*** 7.1 Remove the out-bid task from the bundle ***/
			TaskAssignment::bundle_remove(agents);
			/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
			TaskAssignment::bundle_add(tasks, graph, agents);
			/*** 7.3 Check whether the assignment converge or not ***/
			Flag_ts = TaskAssignment::success_checker(agents);
			/*** 7.4 Update the number of interation ***/
			// Increase the iteration
			for (int i = 0; i < agents[0].num_agents_; i++)
				agents[i].iter_++;
		}
	
		// Compute the paths for each vehicle while statisfying the local specification
		std::map<int64_t, Path_t<SquareCell*>> paths = {};
		for(auto& agent: agents){
			Path_t<SquareCell*> path = GridGraph::PathComputationAStar(tasks, graph, agent.cbba_path_, agent.init_pos_);
			paths[agent.idx_] = path;
		}
        
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
	
		// Check the convergence condition
		std::pair<bool, std::vector<double>> flag_entropy_pair = GridGraph::ConvergenceCheck(paths, graph, threshold);
		Flag_ipas = flag_entropy_pair.first;
		if(Flag_ipas == true){
			for(int agent_idx = 0; agent_idx < flag_entropy_pair.second.size(); agent_idx++){
				entropy_trend_paths[agent_idx].push_back(flag_entropy_pair.second[agent_idx]);
			}
			// Print out the final cost of path after convergence
            double rewards_total = 0.0;
			for(auto& path: paths){
                double rewards = 0.0;
				for(int idx_c = 1; idx_c < path.second.size(); idx_c++){
					rewards = rewards + PENALTY_ * path.second[idx_c]->p_ + (1.0 - path.second[idx_c]->p_);
                }
                ig_data_.cost_path_[path.first] = rewards;
                rewards_total = rewards_total + rewards;
			}
            // Store important output
            ig_data_.num_iteration_ = T;
            ig_data_.entropy_map_ = GridGraph::EntropyMap(graph);
            ig_data_.cost_paths_ = rewards_total;
			break;
		}

		// Store the entropy of current paths
		for(int agent_idx = 0; agent_idx < flag_entropy_pair.second.size(); agent_idx++){
			entropy_trend_paths[agent_idx].push_back(flag_entropy_pair.second[agent_idx]);
		}

		// Determine the set of vertices which has non zero information gain
		std::vector<int64_t> vertex_ig;
		for(auto& c_v: sub_domain){
			Vertex_t<SquareCell*>* vert = graph->GetVertexFromID(c_v);
			std::vector<int64_t>::iterator it1 = std::find(vertex_ig.begin(), vertex_ig.end(), vert->state_->id_);
			if(it1 == vertex_ig.end()){
				vertex_ig.push_back(vert->state_->id_);
			}
			
			std::vector<int64_t> neighbors = vert->GetNeighbourIDs();
			for(auto& neighb_id: neighbors){
				std::vector<int64_t>::iterator it = std::find(vertex_ig.begin(), vertex_ig.end(), neighb_id);
				if(it == vertex_ig.end()){
					vertex_ig.push_back(neighb_id);
				}
			}
		}
		
		
		// Update the information gain
		for(int cell = 0; cell < grid->num_row_ * grid->num_col_; cell++){
			std::vector<int64_t>::iterator it = std::find(vertex_ig.begin(), vertex_ig.end(), cell);
			Vertex_t<SquareCell*>* current_v = graph->GetVertexFromID(cell);
			if(it == vertex_ig.end()){
				current_v->state_->ig_ = 0.0;
			}
			else{
				current_v->state_->ComputeIG(grid, graph, sub_domain);
			}	
		}

		// Select the sensors position  
		std::vector<int64_t> sensor_pos = GridGraph::SelectSensorsPos(num_sensors, graph);
   	
		// Update the uncertain map correspondingly
		GridGraph::UpdateUncertainMap(sensor_pos, graph, true_graph);
	}

    ig_data_.entropy_reduction_ = (ig_data_.entropy_map_init_ - ig_data_.entropy_map_)/ig_data_.entropy_map_init_;
    // Task assignment
    std::map<int, std::vector<int>> agents_tasks_;
    for(auto& agent: agents){
        agents_tasks_[agent.idx_] = agent.cbba_path_;
    }
    ig_data_.agents_tasks = agents_tasks_;
    // Tasks
    std::map<int, int64_t> tasks_info;
    for(auto& task: tasks.tasks_){
        tasks_info[task.idx_] = task.pos_.front();
    }
    ig_data_.tasks_ = tasks_info;

    return ig_data_;
}


IGdata GridGraph::StandardIG(std::vector<Agent> agents, TasksList tasks, int num_sensors, std::shared_ptr<SquareGrid> grid, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<SquareCell*>> true_graph,double threshold){
    IGdata ig_data_;

    /**************************************************************************/
	/*********************** Standard Information Gain ************************/
	/**************************************************************************/
    // The flag of task assignment
	bool Flag_ts = 0;
	// The flag of ipas
	bool Flag_ipas = 0;
	
	int T = 0;
    // Compute the initial entropy
    ig_data_.entropy_map_init_ = GridGraph::EntropyMap(graph);

	clock_t cbba_time;
	cbba_time = clock();
    // while(T < 1){
	while(Flag_ipas == false){
		T++;
		/***************************************************************************************/
		/********************************** Task Assignment ************************************/
		/***************************************************************************************/
		TaskAssignment::ResetTaskAssignmentInfo(agents);
		Flag_ts = 0;
	
		while (Flag_ts != 1){
			for (int i = 0; i < agents[0].num_agents_; i++)
				agents[i].iter_ = 0;

			/*** 6. Communication among neighbors ***/
			TaskAssignment::communicate(agents);
			//Update the history of iteration neighbors
			for (int i = 0; i < agents[0].num_agents_; i++)
				agents[i].history_.iter_neighbors_his.push_back(agents[i].iteration_neighbors_);
			
			/*** 7. Bundle Operations ***/
			/*** 7.1 Remove the out-bid task from the bundle ***/
			TaskAssignment::bundle_remove(agents);
			/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
			TaskAssignment::bundle_add(tasks, graph, agents);
			/*** 7.3 Check whether the assignment converge or not ***/
			Flag_ts = TaskAssignment::success_checker(agents);
			/*** 7.4 Update the number of interation ***/
			// Increase the iteration
			for (int i = 0; i < agents[0].num_agents_; i++)
				agents[i].iter_++;
		}
	
		// Compute the paths for each vehicle while statisfying the local specification
		std::map<int64_t, Path_t<SquareCell*>> paths = {};
		for(auto& agent: agents){
			Path_t<SquareCell*> path = GridGraph::PathComputationAStar(tasks, graph, agent.cbba_path_, agent.init_pos_);
			paths[agent.idx_] = path;
		}
        
        std::vector<Path_t<SquareCell*>> path_collections = TaskAssignment::PathsDecomposition(tasks,agents,paths);

	
	
		// Check the convergence condition
		std::pair<bool, std::vector<double>> flag_entropy_pair = GridGraph::ConvergenceCheck(paths, graph, threshold);
		Flag_ipas = flag_entropy_pair.first;
		if(Flag_ipas == true){
			// Print out the final cost of path after convergence
            double rewards_total = 0.0;
			for(auto& path: paths){
                double rewards = 0.0;
				for(int idx_c = 1; idx_c < path.second.size(); idx_c++){
					rewards = rewards + PENALTY_ * path.second[idx_c]->p_ + (1.0 - path.second[idx_c]->p_);
                }
                ig_data_.cost_path_[path.first] = rewards;
                rewards_total = rewards_total + rewards;
			}
            // Store important output
            ig_data_.num_iteration_ = T;
            ig_data_.entropy_map_ = GridGraph::EntropyMap(graph);
            ig_data_.cost_paths_ = rewards_total;
			break;
		}

		
		// Update the information gain
        std::vector<int64_t> sub_domain;
        for(int idx = 0; idx < grid->num_row_ * grid->num_col_; idx++){
            sub_domain.push_back(idx);
        }
		for(int cell = 0; cell < grid->num_row_ * grid->num_col_; cell++){
			Vertex_t<SquareCell*>* current_v = graph->GetVertexFromID(cell);
			current_v->state_->ComputeIG(grid, graph, sub_domain);
		}

		// Select the sensors position  
		std::vector<int64_t> sensor_pos = GridGraph::SelectSensorsPos(num_sensors, graph);
   	
		// Update the uncertain map correspondingly
		GridGraph::UpdateUncertainMap(sensor_pos, graph, true_graph);
	}

    ig_data_.entropy_reduction_ = (ig_data_.entropy_map_init_ - ig_data_.entropy_map_)/ig_data_.entropy_map_init_;
    // Task assignment
    std::map<int, std::vector<int>> agents_tasks_;
    for(auto& agent: agents){
        agents_tasks_[agent.idx_] = agent.cbba_path_;
    }
    ig_data_.agents_tasks = agents_tasks_;
    // Tasks
    std::map<int, int64_t> tasks_info;
    for(auto& task: tasks.tasks_){
        tasks_info[task.idx_] = task.pos_.front();
    }
    ig_data_.tasks_ = tasks_info;

    return ig_data_;
}


Json::Value GridGraph::WriteResultToJson(std::string method, int64_t caseT, int num_row, int num_col, int num_agents_, int num_tasks_inde_, int num_obstacles, int num_sensors, IGdata ipas_data){
    Json::Value root;
    root[method]["Case #"] = (int)caseT;
    root[method]["# of row"] = num_row;
    root[method]["# of col"] = num_col;
    root[method]["# of agents"] = num_agents_;
    root[method]["# of tasks"] = num_tasks_inde_;
    root[method]["# of obstacle"] = num_obstacles;
    root[method]["# of sensors"] = num_sensors;
    
    // IGdata ipas_data = GridGraph::IPAS(agents,tasks,num_sensors,grid,graph,true_graph);
    
    std::cout << "========================================================"<< std::endl;
    std::cout << "Case T: " << caseT << std::endl;
    std::cout << "Num of row: " << num_row << ", Num of col: " << num_col << std::endl;
    std::cout << "Num of agents: " << num_agents_ << ", Num of tasks: " << num_tasks_inde_ << std::endl;
    std::cout << "Task information is: " << std::endl; 
    for(auto& ts: ipas_data.tasks_){
        std::cout << "Task " << ts.first << ": " << ts.second << std::endl;
    }
    std::cout << "Task assignment is " << std::endl;
    for(auto& ta: ipas_data.agents_tasks){
        std::cout << "Agent " << ta.first << ": ";
        for(auto& tt: ta.second){
            std::cout << tt << ", ";
        }
        std::cout << std::endl;
    }
    std::cout << "Num of sensors: " << num_sensors << std::endl;
    std::cout << "Num of iteration: " << ipas_data.num_iteration_ << std::endl;
    std::cout << "Cost of paths: " << ipas_data.cost_paths_ << std::endl;
    std::cout << "Initial entropy of map: " << ipas_data.entropy_map_init_ << std::endl;
    std::cout << "Entropy of map: " << ipas_data.entropy_map_ << std::endl;
    std::cout << "Entropy reduction: " << ipas_data.entropy_reduction_ << std::endl; 
    

    root[method]["# of Iteration"] = ipas_data.num_iteration_;
    root[method]["Cost of paths"] = ipas_data.cost_paths_;
    for(auto& ct: ipas_data.cost_path_){
        std::string pathname = "Cost of path for agent " + std::to_string(ct.first);
        root[method][pathname] = ct.second;
    }
    root[method]["Initial entropy"] = ipas_data.entropy_map_init_;
    root[method]["Final Entropy"] = ipas_data.entropy_map_;
    root[method]["Entropy Reduction"] = ipas_data.entropy_reduction_;
    for(auto& ag: ipas_data.agents_tasks){
        std::string ts_agent = "Task assignment " + std::to_string(ag.first);
        for(int ts = 0; ts < ag.second.size(); ts++){
            root[method][ts_agent][ts] = ag.second[ts];
        }
    }
    for(auto& tt: ipas_data.tasks_){
        std::string ts_info = "Task "+ std::to_string(tt.first);
        root[method][ts_info] = (int)tt.second; 
    }

    std::cout << std::endl;
  
    return root;
}


void GridGraph::UpdateUncertainMapGP(std::vector<int64_t> sensor_pos, std::vector<int64_t>& certain_cell, std::vector<Eigen::VectorXd>& points, std::vector<int>& targets, Eigen::VectorXd& pi, Eigen::VectorXd& W_sr, Eigen::MatrixXd& LLT, Eigen::VectorXd& kernel_opt,std::shared_ptr<Graph_t<SquareCell *>> graph, std::shared_ptr<Graph_t<SquareCell *>> true_graph){
    // Insert new sensor positions into training data
    for(auto& ss: sensor_pos){
        Vertex_t<SquareCell*>* vert = graph->GetVertexFromID(ss);
        Eigen::VectorXd new_ss(kDimension);
        new_ss(0) = vert->state_->position_.y;
        new_ss(1) = vert->state_->position_.x;
        // Add into training data if the new sensor position has not been inserted
        std::vector<Eigen::VectorXd>::iterator it_ss = std::find(points.begin(),points.end(),new_ss);
        if(it_ss == points.end()){
            points.push_back(new_ss);
            // Check if the postion is occupied or free
            Vertex_t<SquareCell*>* true_vert = true_graph->GetVertexFromID(vert->state_->id_);
            if (true_vert->state_->occupancy_ == OccupancyType::FREE || true_vert->state_->occupancy_ == OccupancyType::INTERESTED){
                vert->state_->p_ = 0.0;
                vert->state_->occupancy_ = OccupancyType::FREE;
                targets.push_back(0);
            }
            else if (true_vert->state_->occupancy_ == OccupancyType::OCCUPIED){
                vert->state_->p_ = 1.0;
                vert->state_->occupancy_ = OccupancyType::OCCUPIED;
                targets.push_back(1);
            }
            else{
                std::cout << "Unknown cell type." << std::endl;
            }
        }
        else{
            Vertex_t<SquareCell*>* true_vert = true_graph->GetVertexFromID(vert->state_->id_);
            if (true_vert->state_->occupancy_ == OccupancyType::FREE || true_vert->state_->occupancy_ == OccupancyType::INTERESTED){
                vert->state_->occupancy_ = OccupancyType::FREE;
                vert->state_->p_ = 0.0;
            }
            else if (true_vert->state_->occupancy_ == OccupancyType::OCCUPIED){
                vert->state_->occupancy_ = OccupancyType::OCCUPIED;
                vert->state_->p_ = 1.0;
            }
            else{
                std::cout << "Unknown cell type." << std::endl;
            }
        }
 
        std::vector<Vertex_t<SquareCell*>*> neighbors = vert->GetNeighbours();
        for(auto&& neighb: neighbors){
            Eigen::VectorXd new_neigb(kDimension);
            new_neigb(0) = neighb->state_->position_.y;
            new_neigb(1) = neighb->state_->position_.x;
            std::vector<Eigen::VectorXd>::iterator it_nn = std::find(points.begin(),points.end(),new_neigb);
            if(it_nn == points.end()){
                points.push_back(new_neigb);
                // Check the occupancy of the neighbor
                Vertex_t<SquareCell*>* true_neighb = true_graph->GetVertexFromID(neighb->state_->id_);
                if(true_neighb->state_->occupancy_ == OccupancyType::FREE || true_neighb->state_->occupancy_ == OccupancyType::INTERESTED){
                    neighb->state_->p_ = GridGraph::UpdateBelProbability(neighb->state_->p_,0);
                    if (neighb->state_->p_ <= cell_accurancy_){
                        neighb->state_->p_ = 0.0;
                        neighb->state_->occupancy_ = OccupancyType::FREE;
                    }
                    // neighb->state_->occupancy_ = OccupancyType::FREE;
                    targets.push_back(0);
                }
                else if(true_neighb->state_->occupancy_ == OccupancyType::OCCUPIED){
                    neighb->state_->p_ = GridGraph::UpdateBelProbability(neighb->state_->p_,1);
                    if(neighb->state_->p_ >= 1.0 - cell_accurancy_){
                        neighb->state_->p_ = 1.0;
                        neighb->state_->occupancy_ = OccupancyType::OCCUPIED;
                    }
                    // neighb->state_->occupancy_ = OccupancyType::OCCUPIED;
                    targets.push_back(1);
                }
                else{
                    std::cout << "Unknown cell type." << std::endl;
                }
            }
            else{
                Vertex_t<SquareCell*>* true_neighb = true_graph->GetVertexFromID(neighb->state_->id_);
                if(true_neighb->state_->occupancy_ == OccupancyType::FREE || true_neighb->state_->occupancy_ == OccupancyType::INTERESTED){
                    neighb->state_->p_ = GridGraph::UpdateBelProbability(neighb->state_->p_,0);
                    if (neighb->state_->p_ <= cell_accurancy_){
                        neighb->state_->p_ = 0.0;
                        neighb->state_->occupancy_ = OccupancyType::FREE;
                    }
                }
                else if(true_neighb->state_->occupancy_ == OccupancyType::OCCUPIED){
                    neighb->state_->p_ = GridGraph::UpdateBelProbability(neighb->state_->p_,1);
                    if(neighb->state_->p_ >= 1.0 - cell_accurancy_){
                        neighb->state_->p_ = 1.0;
                        neighb->state_->occupancy_ = OccupancyType::OCCUPIED;
                    }
                }
                else{
                    std::cout << "UNKNOWN CELL TYPE" << std::endl;
                }
            }
        }
        
        // certain_cell.push_back(ss);
    }

    Eigen::VectorXd *training_y_ = new Eigen::VectorXd(points.size());
    for(size_t kk = 0; kk < training_y_->size(); kk++){
        (*training_y_)(kk) = targets[kk];
    }

    if (!points.empty()){
        // Create a kernel
        Eigen::VectorXd lengths(kDimension);
        lengths(0) = 0.5;
        lengths(1) = 0.5;
        Kernel::Ptr kernel = RBFKernel::Create(lengths);

        // Eigen::VectorXd* targs_ = targets;
        // Create a trainingloglikelihood
        std::shared_ptr<std::vector<Eigen::VectorXd>> points_ptr(new std::vector<Eigen::VectorXd>(points));
        TrainingLogLikelihoodClassification cost(points_ptr, training_y_, kernel, 0.0);
    
        // Test that the analytic and numerical derivatives match at a bunch of
        // different length vectors.
        double parameters[kDimension];
        double gradient[kDimension];

        // Initialize the parameter 
        parameters[0] = 0.5;
        parameters[1] = 0.5;
                                                                
        double objective;
        // Training the hyper-parameters
        cost.Optimizer(kDimension,parameters,&objective,gradient);
        
        // Update the gpc
        Eigen::VectorXd lengths_star_(kDimension);
        lengths_star_(0) = parameters[0];
        lengths_star_(1) = parameters[1];
        Kernel::Ptr kernel_star_ = RBFKernel::Create(lengths_star_);
        
        GaussianProcessClassification gpc(kernel_star_, 0.0, points_ptr, *training_y_, points.size());
        // std::vector<Vertex_t<SquareCell*>*> verts_tt = graph->GetAllVertex();
        // for(auto& vv: verts_tt){
        //     Eigen::VectorXd targ_(kDimension);
        //     targ_(0) = vv->state_->coordinate_.y;
        //     targ_(1) = vv->state_->coordinate_.x;
        //     std::vector<Eigen::VectorXd>::iterator it_v = std::find(points.begin(),points.end(),targ_);
        //     if (it_v == points.end()){
        //         std::pair<bool,double> pred_p = gpc.Predict(targ_,cost.pi_trained_,cost.W_sr_trained_,cost.LLT_trained_);
        //         vv->state_->p_ = pred_p.second;   
        //     }
        // }


        std::vector<Eigen::VectorXd> vert_range_ = {};
        for(int nn = 0; nn < points.size(); nn++){
            std::vector<Eigen::VectorXd> v_neighbs = GridGraph::GetPhysicalNeighbors(points[nn]);
            for(auto& neigb: v_neighbs){
                std::vector<Eigen::VectorXd>::iterator it_nn = std::find(vert_range_.begin(),vert_range_.end(),neigb);
                std::vector<Eigen::VectorXd>::iterator it_nn_training = std::find(points.begin(),points.end(),neigb);
                if(it_nn == vert_range_.end() && it_nn_training == points.end()){
                    vert_range_.push_back(neigb);
                }
            }
        }
        // std::cout << "The number of training data is " << points.size() << std::endl;
        // std::cout << "The number of vertex needed to be updated is " << vert_range_.size() << std::endl;

        int num_vertex_range_ = vert_range_.size();
        for(int nn = 0; nn < num_vertex_range_; nn++){
            std::vector<Eigen::VectorXd> v_neighbs = GridGraph::GetPhysicalNeighbors(vert_range_[nn]);
            // std::cout << "The number of neighbors is "<< v_neighbs.size() << std::endl;
            for(auto& neigb: v_neighbs){
                std::vector<Eigen::VectorXd>::iterator it_nn = std::find(vert_range_.begin(),vert_range_.end(),neigb);
                std::vector<Eigen::VectorXd>::iterator it_nn_training = std::find(points.begin(),points.end(),neigb);
                if(it_nn == vert_range_.end() && it_nn_training == points.end()){
                    vert_range_.push_back(neigb);
                }
            }
        }

        // std::cout << "The number of vertex needed to be updated is " << vert_range_.size() << std::endl;
        for(auto& vert_coord: vert_range_){
            int v_id = GridGraph::GetIDFromCoordinate(vert_coord);
            // std::cout << v_id << ", ";
            Vertex_t<SquareCell*>* vv = graph->GetVertexFromID(v_id);
            Eigen::VectorXd targ_(kDimension);
            targ_(0) = vv->state_->coordinate_.y;
            targ_(1) = vv->state_->coordinate_.x;
            std::pair<bool,double> pred_p = gpc.Predict(targ_,cost.pi_trained_,cost.W_sr_trained_,cost.LLT_trained_);
            if (pred_p.second > cell_accurancy_ && pred_p.second < 1.0 - cell_accurancy_){
                vv->state_->p_ = pred_p.second; 
            }
            else if(pred_p.second <= cell_accurancy_){
                vv->state_->p_ = 0.0;
                vv->state_->occupancy_ = OccupancyType::FREE;
            }
            else{
                vv->state_->p_ = 1.0;
                vv->state_->occupancy_ = OccupancyType::OCCUPIED; 
            }
              
        }
        std::cout << std::endl;
        


        kernel_opt = lengths_star_;
        pi = *cost.pi_trained_;
        W_sr = *cost.W_sr_trained_;
        LLT = *cost.LLT_trained_;
    }
    else{
        std::cout << "No training data is given. " << std::endl;
    }
    GridGraph::UpdateUncertainGraphEdgeCost(graph);

}

std::vector<Eigen::VectorXd> GridGraph::GetPhysicalNeighbors(Eigen::VectorXd vertex){
    std::vector<Eigen::VectorXd> neighbors = {};

    double row  = vertex(0);
    double col = vertex(1);

    // Up
    double row_up = row - 1.0;
    if(row_up >= 0){
        Eigen::VectorXd v_up(kDimension);
        v_up(0) = row_up;
        v_up(1) = col;
        neighbors.push_back(v_up);
    }

    // Down
    double row_down = row + 1.0;
    if(row_down < 30){
        Eigen::VectorXd v_down(kDimension);
        v_down(0) = row_down;
        v_down(1) = col;
        neighbors.push_back(v_down);
    }

    // Left
    double col_left = col - 1.0;
    if(col_left >= 0){
        Eigen::VectorXd v_left(kDimension);
        v_left(0) = row;
        v_left(1) = col_left;
        neighbors.push_back(v_left);
    }

    double col_right = col + 1.0;
    if (col_right < 30){
        Eigen::VectorXd v_right(kDimension);
        v_right(0) = row;
        v_right(1) = col_right;
        neighbors.push_back(v_right);
    }
    return neighbors;   
}

int64_t GridGraph::GetIDFromCoordinate(Eigen::VectorXd vertex){
    return 30*vertex(0) + vertex(1);
}

TasksSet SquareGrid::ConstructRandomLTLTasks(int64_t num_tasks){
    srand(time(NULL));
    std::vector<int64_t> tk_pos;
    std::vector<Task> tks;
    while(tk_pos.size() < num_tasks){
        int64_t t_idx = rand()%(num_col_ * num_row_);
        SquareCell* task_cell = GetCellFromID(t_idx);
        std::vector<int64_t>::iterator it_t = std::find(tk_pos.begin(), tk_pos.end(), t_idx);
        if(task_cell->p_ != 1.0 && it_t == tk_pos.end()){
            tk_pos.push_back(t_idx);
        } 
    }
    for(int ii = 0; ii < num_tasks; ii++){
        int64_t t_AP = ii;
        int64_t t_pos = tk_pos[ii];
        TaskType tt = TaskType::RESCUE;
        int64_t t_agents = 1;

        Task tk(ii,t_AP,{t_pos},tt,t_agents);
        tks.push_back(tk);
    }
    TasksSet tkSet(tks);
    return tkSet;
}

std::shared_ptr<AutoTeam_t<AutoVehicle>> SquareGrid::ConstructRandomAutoTeam(int64_t num_actors, int64_t num_sensors, int64_t num_tasks){
    std::vector<AutoVehicle> auto_vehicles;
    std::vector<int64_t> actor_pos;
    int64_t num_agents = num_actors + num_sensors;

    while(actor_pos.size() < num_actors){
        int64_t act_idx = rand()%(num_col_ * num_row_);
        SquareCell* actor_cell = GetCellFromID(act_idx);
        std::vector<int64_t>::iterator it_t = std::find(actor_pos.begin(), actor_pos.end(), act_idx);
        if(actor_cell->p_ != 1.0 && it_t == actor_pos.end()){
            actor_pos.push_back(act_idx);
        } 
    }
	for (int ii= 0; ii < num_actors+num_sensors; ii++){
		// Read config from agent.ini
		int idx = ii;
        int init_pos;
        if(ii < num_actors){init_pos = actor_pos[ii];}
        else{init_pos = actor_pos[ii-num_actors];}
		
		Eigen::MatrixXi comm_net = Eigen::MatrixXi::Ones(1,num_actors+num_sensors);
        TaskType type;
        if(ii < num_actors){type = TaskType::RESCUE;}
        else{type = TaskType::MEASURE;}
        AutoVehicle auV = AutoVehicle(idx,init_pos,num_agents,comm_net,type,num_tasks,num_sensors);
        auto_vehicles.push_back(auV);
	}
    std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_group = IPASMeasurement::ConstructAutoTeam(auto_vehicles);
    return vehicle_group;
}