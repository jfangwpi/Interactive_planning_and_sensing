#include "map/square_grid.hpp"
#include "config_reader/config_reader.hpp"
#include "vis/graph_vis.hpp"

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

int64_t GridGraph::GetIDFromCoordinate(Eigen::VectorXd vertex){
    return 30*vertex(0) + vertex(1);
}
