#include <cmath>
#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include "gplib/rbf_kernel.hpp"
#include "gplib/log_likelihood.hpp"

using namespace librav;
int main(int argc, char** argv){
    // Define parameters
    const size_t kDimension = 2;
    const size_t NumTrainingPoints = 6;
    const size_t NumTests = 20;
    const double kEpsilon = 0.01;
    const double kMaxError = 1e-8;
    const double kNoiseVariance = 0.0;
    const double alpha = 0.001;


    // Initialize the training data
    std::shared_ptr<std::vector<Eigen::VectorXd>> points(new std::vector<Eigen::VectorXd>);
    Eigen::VectorXd targets(NumTrainingPoints);

    // Enter the ig for certain cell
    Eigen::VectorXd point1(kDimension);
    point1(0) = 7;
    point1(1) = 0;
    points->push_back(point1);
    targets(0) = 0.552;

    Eigen::VectorXd point2(kDimension);
    point2(0) = 7;
    point2(1) = 1;
    points->push_back(point2);
    targets(1) = 0.552;

    Eigen::VectorXd point3(kDimension);
    point3(0) = 8;
    point3(1) = 0;
    points->push_back(point3);
    targets(2) = 0.468;

    Eigen::VectorXd point4(kDimension);
    point4(0) = 7;
    point4(1) = 2;
    points->push_back(point4);
    targets(3) = 0.084;

    Eigen::VectorXd point5(kDimension);
    point5(0) = 7;
    point5(1) = 3;
    points->push_back(point5);
    targets(4) = 0.0;

    Eigen::VectorXd point6(kDimension);
    point6(0) = 7;
    point6(1) = 4;
    points->push_back(point6);
    targets(5) = 0.084;

    // Create a kernel
    Eigen::VectorXd lengths(kDimension);
    lengths(0) = 1.0;
    lengths(1) = 1.0;

    Kernel::Ptr kernel = RBFKernel::Create(lengths);
  
      // Create a trainingloglikelihood
    TrainingLogLikelihood cost(points, &targets, kernel, kNoiseVariance);

    // Test that the analytic and numerical derivatives match at a bunch of
    // different length vectors.
    double parameters[kDimension];
    double gradient[kDimension];

    // Initialize the parameter 
    parameters[0] = 1.0;
    parameters[1] = 1.0;                                                                

    double objective;
    cost.Evaluate(parameters,&objective,gradient);

    bool foundlocalmin[kDimension] = {false, false};

    for(size_t ii = 1; ii >=0; ii--){
        while(foundlocalmin[ii] != true){
        // int IterT = 0;
        // while(IterT <= 1){
            // IterT ++;
            // Evaluate the value of objective fun 
            // double objective;
            cost.Evaluate(parameters,&objective,gradient);
            // std::cout << "The gradient is " << *gradient << std::endl;
        
            // Update the parameters
            parameters[ii] -= alpha * gradient[ii];
            
            cost.Evaluate(parameters,&objective,gradient);
            
            // std::cout << "The objective function is " << objective << std::endl;

            // Evaluate numerical gradient with a forward difference
            parameters[ii] -= kEpsilon;
            double forward;
            cost.Evaluate(parameters,&forward,NULL);
            parameters[ii] += kEpsilon;
            // std::cout << "forward value is " << forward << std::endl;

            if(fabs(gradient[ii]) <= kMaxError){
                std::cout << "Found the local minimum. The value of function is " << objective << std::endl;
                std::cout << "The parameter for " << ii << " is " << parameters[ii] << std::endl;
                foundlocalmin[ii] = true;
            } 
        } 
    }

    return 0;
}