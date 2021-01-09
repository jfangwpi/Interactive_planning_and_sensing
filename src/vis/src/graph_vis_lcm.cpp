#include <iostream>
#include <string>

#include "vis/graph_vis_lcm.hpp"


using namespace librav;

void GraphVisLCM::TransferInfoLCM(std::shared_ptr<Graph_t<SquareCell *>> graph, std::map<int64_t, Path_t<SquareCell*>> paths, std::vector<int64_t> sensors_pos, std::vector<int64_t> local_hspts){
    srand(time(NULL));
    lcm::LCM lcm;

    // Find all vertex 
    std::vector<Vertex_t<SquareCell*>*> all_vertex = graph->GetAllVertex();
    std::cout << "Total number of vertex is " << all_vertex.size() << std::endl;
    graph_data::map_data map_data_;
    map_data_.num_v_ = all_vertex.size();
    for(auto & cell: all_vertex){
        graph_data::vertex_data vertex_;
        vertex_.idx_ = cell->state_->id_;
        vertex_.ig_ = cell->state_->ig_;
        vertex_.p_ = cell->state_->p_;
        map_data_.vertex_.push_back(vertex_);
    }

    // Define path
    graph_data::paths_data paths_data_;
    paths_data_.num_path_ = paths.size();
    for(std::map<int64_t, Path_t<SquareCell*>>::iterator it = paths.begin(); it != paths.end(); it++){
        graph_data::path_data path_data_;
        path_data_.path_size_ = it->second.size();
        for (auto& cell: it->second){
            path_data_.cell_.push_back(cell->id_);
        }
        paths_data_.path_collection_.push_back(path_data_);
    }

    std::string Graph_data = std::to_string(fig_idx_) + "GraphData";
    std::cout << "The chanel is " << Graph_data << std::endl;
    std::string Path_Collection_data = std::to_string(fig_idx_) + "PathCollectionData";

    lcm.publish(Graph_data, &map_data_);
    lcm.publish(Path_Collection_data, &paths_data_);

    // Define sensors position
    graph_data::sensors_data sensors_data_;
    sensors_data_.num_sensor_ = sensors_pos.size();
    for(auto&sensor: sensors_pos){
        sensors_data_.sensor_pos_.push_back(sensor);
    }
    std::string Sensors_data = std::to_string(fig_idx_) + "SensorsData";

    // Define local sensors positions
    graph_data::local_hspts_data local_hspts_data_;
    local_hspts_data_.num_spts_ = local_hspts.size();
    for(auto spt: local_hspts){
        local_hspts_data_.hspts_.push_back(spt);
    }
    std::string Hspts_data = std::to_string(fig_idx_) + "HsptsData";

    lcm.publish(Hspts_data,&local_hspts_data_);
    lcm.publish(Sensors_data, &sensors_data_);
    
    fig_idx_++;
}


void GraphVisLCM::TransferDataTrendLCM(std::vector<double> entropy, std::vector<int64_t> range, std::map<int64_t, std::vector<double>> entropy_paths){
    srand(time(NULL));
    lcm::LCM lcm;

    // Entropy
    graph_data::entropy_trend_data entropy_data_;
    entropy_data_.num_iteration_ = entropy.size();
    for(auto& e: entropy){
        entropy_data_.entropy_.push_back(e);
    }

    // Range
    graph_data::range_checked_data range_data_;
    range_data_.num_iteration_ = range.size();
    for(auto& r: range){
        range_data_.range_.push_back(r);
    }

    // Entropy Paths
    graph_data::entropy_paths_trend_data entropy_paths_trend_data_;
    entropy_paths_trend_data_.num_path_ = entropy_paths.size();
    for(auto& entropy_case: entropy_paths){
        graph_data::entropy_path_trend_data entropy_path_trend_data_;
        entropy_path_trend_data_.agent_idx_ = entropy_case.first;
        entropy_path_trend_data_.num_iteration_ = entropy_case.second.size();
        for (auto& en: entropy_case.second){
            entropy_path_trend_data_.entropy_path_.push_back(en);
        }
        entropy_paths_trend_data_.entropy_paths_.push_back(entropy_path_trend_data_);
    }
    

    std::string Entropy_Trend_data = "EntropyTrendData";
    std::string Range_Trend_data = "RangeTrendData";
    std::string Entropy_Trend_Paths_data = "EntropyTrendPathsData";

    lcm.publish(Entropy_Trend_data, &entropy_data_);
    lcm.publish(Range_Trend_data, &range_data_);
    lcm.publish(Entropy_Trend_Paths_data, &entropy_paths_trend_data_);
}


void GraphVisLCM::TransferGraphLCM(std::shared_ptr<Graph_t<SquareCell *>> graph, std::vector<int64_t> training_data){
    srand(time(NULL));
    lcm::LCM lcm;

    // Find all vertex 
    std::vector<Vertex_t<SquareCell*>*> all_vertex = graph->GetAllVertex();
    std::cout << "Total number of vertex is " << all_vertex.size() << std::endl;
    graph_data::map_data map_data_;
    map_data_.num_v_ = all_vertex.size();
    for(auto & cell: all_vertex){
        graph_data::vertex_data vertex_;
        vertex_.idx_ = cell->state_->id_;
        vertex_.ig_ = cell->state_->ig_;
        vertex_.p_ = cell->state_->p_;
        map_data_.vertex_.push_back(vertex_);
    }

    std::string Graph_data = std::to_string(fig_idx_) + "GraphDataGP";
    lcm.publish(Graph_data, &map_data_);

    // Define training data 
    graph_data::sensors_data training_data_;
    training_data_.num_sensor_ = training_data.size();
    for(auto&sensor: training_data){
        training_data_.sensor_pos_.push_back(sensor);
    }
    std::string Training_data = std::to_string(fig_idx_) + "TrainingData";
    lcm.publish(Training_data, &training_data_);

    fig_idx_++;
}




void GraphVisLCM::TransferBayesianInfoLCM(std::shared_ptr<Graph_t<SquareCell *>> graph, std::vector<int64_t> ROIs, std::vector<int64_t> nz_ig,std::vector<int64_t> tdata){
    srand(time(NULL));
    lcm::LCM lcm;

    // Find all vertices in the graph
    std::vector<Vertex_t<SquareCell*>*> all_vertex = graph->GetAllVertex();
    std::cout << "Total number of vertex is " << all_vertex.size() << std::endl;
    graph_data::map_data map_data_;
    map_data_.num_v_ = all_vertex.size();
    for(auto & cell: all_vertex){
        graph_data::vertex_data vertex_;
        vertex_.idx_ = cell->state_->id_;
        vertex_.p_ = cell->state_->p_;
        vertex_.ig_ = cell->state_->ig_;

        // Is the vertex is ROIs if 
        std::vector<int64_t>::iterator it_nzig = std::find(nz_ig.begin(),nz_ig.end(),cell->state_->id_);
        if (it_nzig != nz_ig.end()){
            vertex_.isNZIG_ = true;
        }
        else {
            vertex_.isNZIG_ = false;
        }

        // Is the vertex is training data
        std::vector<int64_t>::iterator it_trained = std::find(tdata.begin(),tdata.end(),cell->state_->id_);
        if(it_trained != tdata.end()){
            vertex_.isSamples_ = true;
        }
        else{
            vertex_.isSamples_ = false;
        }

        // Is the vertex is ROIs_
        std::vector<int64_t>::iterator it_roi = std::find(ROIs.begin(),ROIs.end(),cell->state_->id_);
        if (it_roi != ROIs.end()){
            vertex_.isROIs_ = true;
        }
        else {
            vertex_.isROIs_ = false;
        }

        map_data_.vertex_.push_back(vertex_);
    }

    std::string Graph_data = std::to_string(fig_idx_) + "BayesianGraphData";
    lcm.publish(Graph_data, &map_data_);
    fig_idx_++;
}