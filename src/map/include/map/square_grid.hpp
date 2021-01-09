#ifndef SQUARE_GRID_HPP
#define SQUARE_GRID_HPP

#include <map>
#include <cmath>
#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include <assert.h>
#include <time.h>
#include <eigen3/Eigen/Core>


#include "map/grid_cell_impl.hpp"
#include "ltl/cell_label.hpp"
#include "graph/graph.hpp"
#include "json/json.h"
#include "vehicle/auto_team.hpp"

namespace librav{
    /*
 * Coordinate System:
 *		
 *	^	origin ------------------> x
 *	^	|
 *		|
 * row	|
 *		|
 *		|
 *	v	|
 *  v y	v 
 *		<<		   column       >>
 */
    const double sensor_accuracy = 0.8;
    static int64_t fig_idx_ = 1;
    const double entropy_threshold = 0.0243;
    const double cell_accurancy_ = 0.01;

    /*** Declear the square grid ***/
    class SquareGrid;

    class AutoVehicle;
    class TasksSet;
    
    /*** Struct for big simulation ***/
    struct IGdata
    {   
        IGdata(): num_iteration_(0), cost_paths_(0.0), entropy_map_init_(0.0), entropy_map_(0.0), entropy_reduction_(0.0){}
        
        // IPAS result
        int num_iteration_;
        double cost_paths_;
        std::map<int64_t,double> cost_path_;
        double entropy_map_init_;
        double entropy_map_;
        double entropy_reduction_;

        // Task Assignment result
        std::map<int, std::vector<int>> agents_tasks;

        // Tasks information
        // (index of task, target position)
        std::map<int, int64_t> tasks_; 
    };

    /*** Square Cell in known envrionment ***/
    class SquareCell: public GridCellBase{
        public: 
            SquareCell(int64_t id, int32_t col, int32_t row, OccupancyType _occu): GridCellBase(id, col, row, _occu){
                ig_ = 0.0; 
                p_ = 0.0;
            };
            ~SquareCell() = default;
        
            // For buchi automaton
            CellLabel cell_labels_;

            // For lifted graph
            std::vector<int64_t> lifted_vertices_id_;

            // For uncertain square cell in unknown environment
            double ig_;
            double p_;

        public:
            /*** Required by buchi automaton ***/
            std::string GetCellLabels();
            int32_t GetCellBitMap();     

            // Compute the information gain for each square cell
            void ComputeIG(std::shared_ptr<SquareGrid> grid, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int64_t> subRegion);
    };

    class SquareGrid{
        public:
            SquareGrid(int32_t row_num, int32_t col_num, double cell_size = 0.1, int32_t pixel_per_meter = 100, int32_t default_label = 0);
            ~SquareGrid();

        public: 
            std::vector<std::vector<SquareCell *>> grid_cells_;

        public: 
            int32_t num_row_;
            int32_t num_col_;
            double cell_size_;
            int32_t pixel_per_meter_;

            int32_t default_label_;
            int32_t obstacle_label_;

        public: 
            Position2i GetCoordinateFromID(int64_t id);
        
            void SetCellOccupancy(int32_t x_col, int32_t y_row, OccupancyType occ);
            void SetCellOccupancy(int64_t id, OccupancyType occ);

            int64_t GetIDFromCoordinate(int32_t x_col, int32_t y_row);

	        SquareCell* GetCellFromID(int64_t id);

	        std::vector<SquareCell*> GetNeighbors(int64_t id, bool allow_diag);
            std::vector<SquareCell*> GetNeighbors(int32_t x_col, int32_t y_row, bool allow_diag);

            /*** Set region label***/
            void SetObstacleRegionLabel(int64_t id, int8_t label);
            void SetInterestedRegionLabel(int64_t id, int8_t label);

            /*** Uncertain Grid Map ***/
            void SetCellProbability(int64_t id, double p);

            /*** Generate random occupancy grid map ***/
            void SetRandomOccupanyMap(double obstacle_percentage);
            bool OccupancyGridMapValidity(std::shared_ptr<AutoTeam_t<AutoVehicle>> vehicle_team, TasksSet tasks, std::shared_ptr<Graph_t<SquareCell*>> graph);
            std::shared_ptr<SquareGrid> DuplicateSquareGrid();
    
    };
    
    namespace GridGraph{
        /*** Required by accurate environment ***/
        // Create gird map with given row and colum num
        std::shared_ptr<SquareGrid> CreateSquareGrid(int32_t row_size, int32_t col_size, double cell_size);
        std::shared_ptr<SquareGrid> CreateSquareGrid(int64_t row_size, int64_t col_size, double cell_size, std::shared_ptr<AutoTeam_t<AutoVehicle>> teams, TasksSet tasks);
        // Create grid map by reading from config file: map.ini
        std::shared_ptr<SquareGrid> CreateSquareGrid();
        std::shared_ptr<SquareGrid> CreateSquareGrid(std::string filename);

        // Create a graph based on the given grid
        std::shared_ptr<Graph_t<SquareCell *>> BuildGraphFromSquareGrid(std::shared_ptr<SquareGrid> grid, bool allow_diag_move, bool ignore_obs);       
        
        /*** Required by unknown environment ***/
        // Compute the entropy of defined subregion with given graph
        double EntropyCalculation(std::vector<int64_t> subRegion, std::shared_ptr<Graph_t<SquareCell *>> graph);
        // Compute the list of potential configurations
        std::vector<std::map<int64_t, int>> PotentialConfiguration(std::vector<Vertex_t<SquareCell*>*> N_v_);
        // Compute the entropy of subRegion with given possible configuration and subregion
        double EntropyPotentialConfig(std::vector<int64_t> subRegion, std::shared_ptr<SquareGrid> grid, std::map<int64_t, int> config_, int64_t sensor_pos_, bool allow_diag_move = false);
        // Update the belief probability: 1 -> obstacle, 0 -> free
        double UpdateBelProbability(double pred_bel, int status);
        // Given certain measurement configuration and current graph, compute the probabiity of the given measurement
        double MeasurementProbability(std::shared_ptr<Graph_t<SquareCell*>> graph, std::map<int64_t, int> config);
        // Compute the entropy with given graph 
        double EntropyMap(std::shared_ptr<Graph_t<SquareCell*>> graph);
        double EntropyMap(std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<int64_t> measurements);
        // Compute the entropy with given path
        double EntropyPath(Path_t<SquareCell*> path,std::shared_ptr<Graph_t<SquareCell*>> graph);
        // Compute the sub-region from the given paths
        std::vector<int64_t> SubRegionFromPaths(std::vector<Path_t<SquareCell*>> paths, std::shared_ptr<SquareGrid> grid, std::shared_ptr<Graph_t<SquareCell*>> graph);
        std::vector<int64_t> SubRegionComputation(std::shared_ptr<SquareGrid> grid, std::map<int64_t, int> config, int64_t sensor_pos, int64_t start_id, int64_t end_id, bool allow_diag_move);
        // Compute the egde cost 
        double CalcHeuristic(SquareCell *node1, SquareCell *node2);    
        double CalcHeuristicUncertain(SquareCell *node1, SquareCell *node2); 
        // Compute the convergence flag
        std::pair<bool, std::vector<double>> ConvergenceCheck(std::map<int64_t, Path_t<SquareCell*>> paths, std::shared_ptr<Graph_t<SquareCell*>> graph, double threshold=entropy_threshold);
        // Select the sensors position
        std::vector<int64_t> SelectSensorsPos(int64_t num_sensors, std::shared_ptr<Graph_t<SquareCell *>> graph);
        std::vector<int64_t> SelectSensorsPos(int64_t num_sensors,std::vector<int64_t> sensors,std::shared_ptr<Graph_t<SquareCell *>> graph);
        // Determin the next sensor position based on miss probability strategy
        int64_t SensorPosFromMissProbability(std::vector<int64_t> sensors_pos_,std::vector<int64_t> list_max_ig, std::shared_ptr<Graph_t<SquareCell*>> graph);
        // Update the uncertain map with given position of sensors
        void UpdateUncertainMap(std::vector<int64_t> sensor_pos, std::shared_ptr<Graph_t<SquareCell *>> graph, std::shared_ptr<Graph_t<SquareCell *>> true_graph);
        // Update the uncertain map based on GP with given new sensor positions
        void UpdateUncertainMapGP(std::vector<int64_t> sensor_pos, std::vector<int64_t>& certain_cell, std::vector<Eigen::VectorXd>& points, std::vector<int>& targets, Eigen::VectorXd& pi, Eigen::VectorXd& W_sr, Eigen::MatrixXd& LLT, Eigen::VectorXd& kernel_opt,std::shared_ptr<Graph_t<SquareCell *>> graph, std::shared_ptr<Graph_t<SquareCell *>> true_graph);
        
        // Update the edge cost with given measurement
        void UpdateUncertainGraphEdgeCost(std::shared_ptr<Graph_t<SquareCell*>> graph);
        void UpdateUncertainGPGraphEdgeCost(std::shared_ptr<Graph_t<SquareCell*>> graph);
        // Write result to json file
        Json::Value WriteResultToJson(std::string method, int64_t caseT, int num_row, int num_col, int num_agents_, int num_tasks_inde_, int num_obstacles, int num_sensors, IGdata ipas_data);
    
        // Find the physical neighbors
        std::vector<Eigen::VectorXd> GetPhysicalNeighbors(Eigen::VectorXd vertex);
        int64_t GetIDFromCoordinate(Eigen::VectorXd vertex);
    };

}


#endif /* SQUARE_GRID_HPP */