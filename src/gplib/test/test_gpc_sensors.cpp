#include <cmath>
#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <utility>
#include "vis/graph_vis_lcm.hpp"
#include "map/square_grid.hpp"
#include "gplib/log_likelihood_classification.hpp"

using namespace librav;
bool ObsChecker(std::vector<std::pair<double,double>> obs, Eigen::VectorXd point){
  bool flag_obs = true;
  std::pair<double,double> check_pt = std::make_pair<double,double>((double)point(0),(double)point(1));
  std::vector<std::pair<double,double>>::iterator it = std::find(obs.begin(),obs.end(),check_pt);
  if(it == obs.end()){
    flag_obs = false;
  }
  else{
    flag_obs = true;
  }
  return flag_obs;
}


int main(int argc, char** argv){
    // Define parameters
    const size_t kDimension = 2;
    const size_t NumParams = 2;
    const size_t NumTrainingPoints = 10;
    const size_t NumTests = 20;
    const double kEpsilon = 0.01;
    const double kMaxError = 1e-8;
    const double kNoiseVariance = 0.0;
    const double alpha = 0.01;
    std::shared_ptr<std::vector<Eigen::VectorXd>> points(new std::vector<Eigen::VectorXd>);
    std::vector<bool> targets_v;

    // Define the obs blocks
    // Block 1
    std::vector<std::pair<double,double>> obs_; 
    for(size_t ii = 2; ii < 5;ii++){
      for(double jj = 0; jj < 2;jj++){
        obs_.push_back(std::make_pair(ii,jj));
      }
    }

    // Block 2
    for(size_t ii = 0; ii < 4;ii++){
      for(size_t jj = 5; jj < 7;jj++){
        obs_.push_back(std::make_pair(ii,jj));
      }
    }

    // Block 3
    for(size_t ii = 6; ii < 8;ii++){
      for(size_t jj = 0; jj < 5;jj++){
        obs_.push_back(std::make_pair(ii,jj));
      }
    }


    // Block 4 
    for(size_t ii = 6; ii < 10;ii++){
      for(size_t jj = 6; jj < 8;jj++){
        obs_.push_back(std::make_pair(ii,jj));
      }
    }

    // std::cout << "The obs is " << std::endl;
    // for(auto& kk: obs_){
    //   std::cout << "( "<< kk.first << ", " << kk.second << " )"<<std::endl;
    // }
    std::cout << "The num of obs is " << obs_.size() << std::endl;

    // Initialize the training data
    std::map<int,std::vector<double>> sensors_;
    sensors_[1] = {5,3};
    sensors_[2] = {5,7};
    sensors_[3] = {7,5};
    sensors_[4] = {2,5};
    

    for(auto& ss: sensors_){
        Eigen::VectorXd point(kDimension);
        point(0) = ss.second[0];
        point(1) = ss.second[1];
        std::vector<Eigen::VectorXd>::iterator it_p = std::find(points->begin(), points->end(), point);
        if(it_p == points->end()){
          points->push_back(point);
          bool flag_obs = ObsChecker(obs_,point);
          if(flag_obs == false){
            targets_v.push_back(0);
          }
          else{
            targets_v.push_back(1);
          }
        }
        // if(flag_obs == false){
        //   std::cout << "point ( " << point(0) <Eigen::VectorXd< ", " << point(1) << " ) is not obs. " << std::endl; 
        // }
        // else{
        //   std::cout << "point ( " << point(0) << ", " << point(1) << " ) is obs. " << std::endl; 
        
        // Find the neighbors
        // Up
        double row_up = ss.second[0];
        double col_up = ss.second[1] + 1.0;
        if(col_up < 10.5){
          Eigen::VectorXd point_up(kDimension);
          point_up(0) = row_up;
          point_up(1) = col_up;
          std::vector<Eigen::VectorXd>::iterator it_p = std::find(points->begin(), points->end(), point_up);
          if(it_p == points->end()){
            points->push_back(point_up);
            bool flag_obs_pt_up = ObsChecker(obs_,point_up);
            if(flag_obs_pt_up == false){
              targets_v.push_back(0);
            }
            else{
              targets_v.push_back(1);
            }
          }
        }
        

        // Down
        double row_down = ss.second[0];
        double col_down = ss.second[1] - 1.0;
        if(col_down >= 0.5){
          Eigen::VectorXd point_down(kDimension);
          point_down(0) = row_down;
          point_down(1) = col_down;
          std::vector<Eigen::VectorXd>::iterator it_p = std::find(points->begin(), points->end(), point_down);
          if(it_p == points->end()){
            points->push_back(point_down);
            bool flag_obs_pt_down = ObsChecker(obs_,point_down);
            if(flag_obs_pt_down == false){
              targets_v.push_back(0);
            }
            else{
              targets_v.push_back(1);
            }
          }
        }

        // Left
        double row_left = ss.second[0] - 1.0;
        double col_left = ss.second[1];
        if(row_left >= 0.5){
          Eigen::VectorXd point_left(kDimension);
          point_left(0) = row_left;
          point_left(1) = col_left;
          std::vector<Eigen::VectorXd>::iterator it_p = std::find(points->begin(), points->end(), point_left);
          if(it_p == points->end()){
            points->push_back(point_left);
            bool flag_obs_pt_left = ObsChecker(obs_,point_left);
            if(flag_obs_pt_left == false){
              targets_v.push_back(0);
            }
            else{
              targets_v.push_back(1);
            }
          }
        }

        // Right
        double row_right = ss.second[0] + 1.0;
        double col_right = ss.second[1];
        if(row_right < 10.5){
          Eigen::VectorXd point_right(kDimension);
          point_right(0) = row_right;
          point_right(1) = col_right;
          std::vector<Eigen::VectorXd>::iterator it_p = std::find(points->begin(), points->end(), point_right);
          if(it_p == points->end()){
            points->push_back(point_right);
            bool flag_obs_pt_right = ObsChecker(obs_,point_right);
            if(flag_obs_pt_right == false){
              targets_v.push_back(0);
            }
            else{
              targets_v.push_back(1);
            }
          }
        }
    }

    Eigen::VectorXd point1(kDimension);
    point1(0) = 0;
    point1(1) = 0;
    points->push_back(point1);
    targets_v.push_back(0);

    Eigen::VectorXd point2(kDimension);
    point2(0) = 0;
    point2(1) = 7;
    points->push_back(point2);
    targets_v.push_back(0);

    Eigen::VectorXd point3(kDimension);
    point3(0) = 5;
    point3(1) = 0;
    points->push_back(point3);
    targets_v.push_back(0);

    Eigen::VectorXd point4(kDimension);
    point4(0) = 9;
    point4(1) = 5;
    points->push_back(point4);
    targets_v.push_back(0);


    // std::cout << "The num of training data is " << targets.size() << std::endl;
    Eigen::VectorXd targets(targets_v.size());
    for(int ii = 0; ii < targets.size(); ii++){
      if(targets_v[ii] == true){
        targets(ii) = 1;
      }
      else{
        targets(ii) = 0;
      }
    }


    std::cout << "=============================" << std::endl;
    std::cout << "Training data is " << std::endl;
    for(size_t kk = 0; kk < points->size(); kk++){
      std::cout << "v: (" << (*points)[kk](0) << ", " << (*points)[kk](1) << "). The target is " << targets(kk) << std::endl;
    }
    std::cout << "=============================" << std::endl;


    

    // Create the graph 
    int64_t num_row = 10;
    int64_t num_col = 10;
	  std::shared_ptr<SquareGrid> grid = GridGraph::CreateSquareGrid(num_row, num_col, 1);
    std::shared_ptr<Graph_t<SquareCell*>> graph = GridGraph::BuildGraphFromSquareGrid(grid, false, true);


    // Create a kernel
    Eigen::VectorXd lengths(kDimension);
    lengths(0) = 0.1;
    lengths(1) = 0.1;
    Kernel::Ptr kernel = RBFKernel::Create(lengths);
  
    // Create a trainingloglikelihood
    std::cout << "The number of points is " << points->size() << std::endl;
    TrainingLogLikelihoodClassification cost(points, &targets, kernel, kNoiseVariance);
   
    // Test that the analytic and numerical derivatives match at a bunch of
    // different length vectors.
    double parameters[kDimension];
    double gradient[kDimension];

    // Initialize the parameter 
    parameters[0] = 0.1;
    parameters[1] = 0.1;
                                                             
    double objective;
    // Training the hyper-parameters
    cost.Optimizer(kDimension,parameters,&objective,gradient);
    
    // Update the gpc
    Eigen::VectorXd lengths_star_(kDimension);
    lengths_star_(0) = parameters[0];
    lengths_star_(1) = parameters[1];
    Kernel::Ptr kernel_star_ = RBFKernel::Create(lengths_star_);

    GaussianProcessClassification gpc(kernel_star_, kNoiseVariance, points, targets, points->size());
    for(size_t ii = 0 ; ii < 10; ii++){
      for(size_t jj = 0; jj < 10; jj++){
        Eigen::VectorXd targ_(kDimension);
        targ_(0) = ii;
        targ_(1) = jj;
        std::pair<bool,double> pred_p = gpc.Predict(targ_,cost.pi_trained_,cost.W_sr_trained_,cost.LLT_trained_);
        // Find the id of the cell
        // double row_jj = ii -0.5;
        // double col_ii = jj -0.5;
        int cellID = num_row * int(ii) + int(jj);
        std::cout << "The actual coordinate is (" << ii << ", " << jj << "). The correponding id is " << cellID << std::endl;
        grid->SetCellProbability(cellID, pred_p.second);
        std::cout << "The probability of occupancy is " << pred_p.second << std::endl;
        std::cout << "=====================" << std::endl;
        // jj = jj + 1.0;
      }
      // ii = ii + 1.0;
    }

    GraphVisLCM::TransferGraphLCM(graph,{});
    
    return 0;
}