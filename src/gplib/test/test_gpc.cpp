#include <cmath>
#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include "gplib/rbf_kernel.hpp"
#include "gplib/log_likelihood_classification.hpp"

using namespace librav;
int main(int argc, char** argv){
    // Define parameters
    const size_t kDimension = 2;
    const size_t NumParams = 2;
    const size_t NumTrainingPoints = 5;
    const size_t NumTests = 20;
    const double kEpsilon = 0.01;
    const double kMaxError = 1e-8;
    const double kNoiseVariance = 0.0;
    const double alpha = 0.01;


    // Initialize the training data
    std::shared_ptr<std::vector<Eigen::VectorXd>> points(new std::vector<Eigen::VectorXd>);
    Eigen::VectorXd targets(NumTrainingPoints);

    // Training Data for points
    Eigen::VectorXd point1(kDimension);
    point1(0) = 0.5;
    point1(1) = 6.5;
    points->push_back(point1);
    targets(0) = 1;

    Eigen::VectorXd point2(kDimension);
    point2(0) = 1.5;
    point2(1) = 7.5;
    points->push_back(point2);
    targets(1) = 1;

    Eigen::VectorXd point3(kDimension);
    point3(0) = 1.5;
    point3(1) = 6.5;
    points->push_back(point3);
    targets(2) = 1;

    Eigen::VectorXd point4(kDimension);
    point4(0) = 1.5;
    point4(1) = 5.5;
    points->push_back(point4);
    targets(3) = 1;

    Eigen::VectorXd point5(kDimension);
    point5(0) = 2.5;
    point5(1) = 6.5;
    points->push_back(point5);
    targets(4) = 0;

    // Eigen::VectorXd point6(kDimension);
    // point6(0) = 2.5;
    // point6(1) = 4.5;
    // points->push_back(point6);
    // targets(5) = 0;

    // Eigen::VectorXd point7(kDimension);
    // point7(0) = 3.5;
    // point7(1) = 5.5;
    // points->push_back(point7);
    // targets(6) = 0;

    // Eigen::VectorXd point8(kDimension);
    // point8(0) = 3.5;
    // point8(1) = 4.5;
    // points->push_back(point8);
    // targets(7) = 0;

    // Eigen::VectorXd point9(kDimension);
    // point9(0) = 3.5;
    // point9(1) = 3.5;
    // points->push_back(point9);
    // targets(8) = 1;

    // Eigen::VectorXd point10(kDimension);
    // point10(0) = 4.5;
    // point10(1) = 4.5;
    // points->push_back(point10);
    // targets(9) = 0;


    // Create a kernel
    Eigen::VectorXd lengths(kDimension);
    lengths(0) = 0.0;
    lengths(1) = 0.0;
    Kernel::Ptr kernel = RBFKernel::Create(lengths);
  
      // Create a trainingloglikelihood
    TrainingLogLikelihoodClassification cost(points, &targets, kernel, kNoiseVariance);

    // Test that the analytic and numerical derivatives match at a bunch of
    // different length vectors.
    double parameters[kDimension];
    double gradient[kDimension];

    // Initialize the parameter 
    parameters[0] = 0.0;
    parameters[1] = 0.0;
                                                             
    double objective;

    int ii = 1;
    bool foundlocalmin[kDimension] = {false,false};
    while(foundlocalmin[ii] != true){
      cost.Evaluate(parameters,&objective,gradient);
      std::cout << "The log likelihood fun is " << objective << std::endl;
      std::cout << "The gradient is ";
      for(auto &kk: gradient){
        std::cout << kk << ", ";
      }
      std::cout << std::endl;  
      double cost_prev = objective;
      parameters[ii] += alpha * gradient[ii];
      std::cout << "Parameter is " << parameters[ii] << std::endl;
      std::cout << "The gradient is " << gradient[ii] << std::endl; 
      cost.Evaluate(parameters,&objective,gradient);
      std::cout << "The log likelihood fun is " << objective << std::endl;
      std::cout << "The gradient is ";
      for(auto &kk: gradient){
        std::cout << kk << ", ";
      }
      std::cout << std::endl;
      std::cout << "====================================" << std::endl;  

      if(std::fabs((cost_prev - objective)/cost_prev) <= kMaxError){
        std::cout << "Found the local minimum. The value of function is " << objective << std::endl;
        std::cout << "The parameter for " << ii << " is " << parameters[ii] << std::endl;
        foundlocalmin[ii] = true;
      } 
    }
    
    

    


    return 0;
}