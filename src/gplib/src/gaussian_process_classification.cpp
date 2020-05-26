#include "gplib/log_likelihood_classification.hpp"

#include <ceres/ceres.h>
#include <random>

using namespace librav;
// Constructor for Gaussian Process Classification
GaussianProcessClassification::GaussianProcessClassification(const Kernel::Ptr& kernel, double noise, const std::shared_ptr<std::vector<Eigen::VectorXd>> points, const Eigen::VectorXd& targets, size_t max_points):
    kernel_(kernel),
    noise_(noise),
    points_(points),
    max_points_(max_points),
    targets_(max_points),
    regressed_(max_points),
    covariance_(max_points,max_points),
    covariance_gradient_(max_points,max_points){
        dimension_ = points_->at(0).size();

        // Set "target_: y".
        targets_.head(points_->size()) = targets;

        // Compute covariance matrix
        Covariance();
        CovarianceGradient();
       

        // Compute Cholesky decomposition of covariance
        llt_.compute(covariance_.topLeftCorner(points_->size(),points_->size()));
        Eigen::MatrixXd LLT_ = llt_.matrixL();

        // Compute regressed targets.(the target based on the predicted model).
        regressed_.head(points_->size()) = llt_.solve(targets_.head(points_->size()));
    }

// Evaluate the mean and covariance at a point
void GaussianProcessClassification::Evaluate(const Eigen::VectorXd& x, double& mean, double& variance) const{
    Eigen::VectorXd cross(points_->size());
    CrossCovariance(x,cross);

    // Compute mean and variance
    mean = cross.dot(regressed_);
    variance = 1.0 - cross.dot(llt_.solve(cross));
}

// Evaluate at the ii'th training point.
void GaussianProcessClassification::EvaluateTrainingPoint(size_t ii, double& mean, double& variance) const{
    //Extract cross covariance (must subtract off added noise).
    Eigen::VectorXd cross = covariance_.col(ii);
    cross(ii) -= noise_;

    // Compute mean and covariance
    mean = cross.dot(regressed_);
    variance = 1.0 - cross.dot(llt_.solve(cross));
}

// Compute the gradient of covariance 
void GaussianProcessClassification::CovarianceGradient(){
    for(size_t ii = 0; ii < points_->size(); ii++){
        covariance_gradient_(ii,ii) = kernel_->EvaluateGradient(points_->at(ii),points_->at(ii),ii,ii);
        for(size_t jj = 0; jj < ii; jj++){
            covariance_gradient_(ii,jj) = kernel_->EvaluateGradient(points_->at(ii),points_->at(jj),ii,jj);
            covariance_gradient_(jj,ii) = covariance_gradient_(ii,jj);
        }
    }
}


// Compute the covariance and cross covariance against the training points.
void  GaussianProcessClassification::Covariance(){
    for(size_t ii=0; ii < points_->size(); ii++){
        covariance_(ii,ii) = kernel_->Evaluate(points_->at(ii),points_->at(ii));
        for (size_t jj=0; jj < ii; jj++){
            covariance_(ii,jj) = kernel_->Evaluate(points_->at(ii),points_->at(jj));
            covariance_(jj,ii) = covariance_(ii,jj);
        }
    }
}

void GaussianProcessClassification::CrossCovariance(const Eigen::VectorXd& x, Eigen::VectorXd& cross) const{
    for(size_t ii = 0; ii < points_->size(); ii++){
        cross(ii) = kernel_->Evaluate(points_->at(ii),x);
    }
}

// Add single point
bool GaussianProcessClassification::Add(const Eigen::VectorXd& x, double target){
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

bool GaussianProcessClassification::Add(const std::vector<Eigen::VectorXd>& points, const Eigen::VectorXd& targets){
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

// Applied for predict: update the covariance matrix
Eigen::MatrixXd GaussianProcessClassification::Covariance(const Eigen::VectorXd& target){
    Eigen::MatrixXd K_star_(points_->size(),1);
    for(size_t ii = 0; ii < points_->size(); ii++){
        K_star_(ii,0) = kernel_->Evaluate(target,points_->at(ii));
    }

    return K_star_;
}

// Predict the data X based on Algorithm 3.2
std::pair<bool,double> GaussianProcessClassification::Predict(const Eigen::VectorXd& x, Eigen::VectorXd* pi_, Eigen::VectorXd *W_sr_, Eigen::MatrixXd *LLT_){
    std::pair<bool,double> pred_p_;
    // Compute the covariance K_star
    Eigen::MatrixXd K_star_ = Covariance(x);

    Eigen::VectorXd diff_ = targets_ - *pi_;
    double f_star_ = 0.0;
    for (size_t ii = 0; ii < points_->size(); ii++){
        f_star_ += K_star_(ii) * diff_(ii);
    }
    // Determine the class type
    bool class_type;
    if (f_star_ > 0){
        class_type = 1;
    }
    else{
        class_type = 0;
    }

    // std::cout << "The LLT is " << std::endl;
    // std::cout << *LLT_ << std::endl;
    Eigen::VectorXd W_sr_Kstar_(points_->size());
    for(size_t ii = 0; ii < points_->size(); ii++){
        W_sr_Kstar_(ii) = (*W_sr_)(ii) * K_star_(ii);
    }
    
    Eigen::VectorXd v_ = LLT_->colPivHouseholderQr().solve(W_sr_Kstar_);
    double v_T_v_ = v_.transpose() * v_;

    double var_f_star_ = 1.0 - v_T_v_;
    double alpha = 1.0/(2.0 * var_f_star_);

    // Parameters
    Eigen::VectorXd LAMBDAS(5);
    LAMBDAS(0) = 0.41;
    LAMBDAS(1) = 0.4;
    LAMBDAS(2) = 0.37;
    LAMBDAS(3) = 0.44;
    LAMBDAS(4) = 0.39;
    
    Eigen::VectorXd COEFS(5);
    COEFS(0) = -1854.8214151;
    COEFS(1) = 3516.89893646;
    COEFS(2) = 221.29346712;
    COEFS(3) = 128.12323805;
    COEFS(4) = -2010.49422654;
    

    Eigen::VectorXd gamma = LAMBDAS * f_star_;
    Eigen::VectorXd integrals_(5);
    for(size_t ii = 0; ii < 5; ii++){
        integrals_(ii) = std::sqrt(3.1415926 / alpha) * (std::erf(gamma(ii) * std::sqrt(alpha / (alpha + LAMBDAS(ii) * LAMBDAS(ii)))) / (2.0 * std::sqrt(var_f_star_ * 2.0 * 3.1415926)));
    }
   
    double pi_star_ = 0.0;
    for(size_t ii = 0; ii < 5; ii++){
        pi_star_ += COEFS(ii) * integrals_(ii) + 0.5 * COEFS(ii);
    }

    pred_p_ = std::make_pair(class_type,pi_star_);
    return pred_p_;
}

