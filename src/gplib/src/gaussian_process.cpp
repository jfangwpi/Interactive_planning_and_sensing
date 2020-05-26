#include "gplib/log_likelihood.hpp"

#include <ceres/ceres.h>
#include <random>

using namespace librav;
// Constructor for Gaussian Process
GaussianProcess::GaussianProcess(const Kernel::Ptr& kernel, double noise, const std::shared_ptr<std::vector<Eigen::VectorXd>> points, const Eigen::VectorXd& targets, size_t max_points):
    kernel_(kernel),
    noise_(noise),
    points_(points),
    max_points_(max_points),
    targets_(max_points),
    regressed_(max_points),
    covariance_(max_points,max_points){
        dimension_ = points_->at(0).size();

        // Set "target_: y".
        targets_.head(points_->size()) = targets;

        // Compute covariance matrix
        Covariance();

        // Compute Cholesky decomposition of covariance
        llt_.compute(covariance_.topLeftCorner(points_->size(),points_->size()));
        Eigen::MatrixXd LLT_ = llt_.matrixL();

        // Compute regressed targets.(the target based on the predicted model).
        regressed_.head(points_->size()) = llt_.solve(targets_.head(points_->size()));
    }

// Evaluate the mean and covariance at a point
void GaussianProcess::Evaluate(const Eigen::VectorXd& x, double& mean, double& variance) const{
    Eigen::VectorXd cross(points_->size());
    CrossCovariance(x,cross);

    // Compute mean and variance
    mean = cross.dot(regressed_);
    variance = 1.0 - cross.dot(llt_.solve(cross));
}

// Evaluate at the ii'th training point.
void GaussianProcess::EvaluateTrainingPoint(size_t ii, double& mean, double& variance) const{
    //Extract cross covariance (must subtract off added noise).
    Eigen::VectorXd cross = covariance_.col(ii);
    cross(ii) -= noise_;

    // Compute mean and covariance
    mean = cross.dot(regressed_);
    variance = 1.0 - cross.dot(llt_.solve(cross));
}


// Compute the covariance and cross covariance against the training points.
void  GaussianProcess::Covariance(){
    for(size_t ii=0; ii < points_->size(); ii++){
        covariance_(ii,ii) = kernel_->Evaluate(points_->at(ii),points_->at(ii));
        for (size_t jj=0; jj < ii; jj++){
            covariance_(ii,jj) = kernel_->Evaluate(points_->at(ii),points_->at(jj));
            covariance_(jj,ii) = covariance_(ii,jj);
        }
    }
}

void GaussianProcess::CrossCovariance(const Eigen::VectorXd& x, Eigen::VectorXd& cross) const{
    for(size_t ii = 0; ii < points_->size(); ii++){
        cross(ii) = kernel_->Evaluate(points_->at(ii),x);
    }
}

// Add single point
bool GaussianProcess::Add(const Eigen::VectorXd& x, double target){
    const size_t N = points_->size();

    if(N < max_points_){
        for (size_t ii = 0; ii < N; ii++){
            // Add row and column to the covariance matrix.
            covariance_(ii,N) = kernel_->Evaluate(x,points_->at(ii));
            covariance_(N,ii) = covariance_(ii,N);
        }
        covariance_(N,N) = 1.0 + noise_;

        // Add the new data's target
        targets_(N) = target;
        points_->push_back(x);

        // Recompute Cholesky decomposition and regressed targets
        llt_.compute(covariance_.topLeftCorner(N+1,N+1));
        regressed_.head(N+1) = llt_.solve(targets_.head(N+1));

        return true;
    }
    else{
        return false;
    }
}

bool GaussianProcess::Add(const std::vector<Eigen::VectorXd>& points, const Eigen::VectorXd& targets){
    size_t total_points = points_->size() + points.size();
    bool has_room;
    if(total_points <= max_points_){
        has_room = true;
    }
    else{
        has_room = false;
    }

    for(size_t ii=0; ii < points.size(); ii++){
        const size_t N = points_->size();
        if(N >= max_points_){break;};

        for(size_t jj=0; jj < N; jj++){
            covariance_(jj,N) = kernel_->Evaluate(points[ii],points_->at(jj));
            covariance_(N,jj) = covariance_(jj,N);
        }

        covariance_(N,N) = 1.0 + noise_;

        // Add the new data
        targets_(N) = targets(ii);
        points_->push_back(points[ii]);
    }

    // Recompute Cholesky decomposition and regressed targets
    llt_.compute(covariance_.topLeftCorner(points_->size(),points_->size()));

    regressed_.head(points_->size()) = llt_.solve(targets_.head(points_->size()));

    return has_room;
}


