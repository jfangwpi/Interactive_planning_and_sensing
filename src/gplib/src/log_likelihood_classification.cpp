#include "gplib/log_likelihood_classification.hpp"

#include <ceres/ceres.h>
#include <math.h>

using namespace librav;
TrainingLogLikelihoodClassification::TrainingLogLikelihoodClassification(const std::shared_ptr<std::vector<Eigen::VectorXd>>& points,
                                   const Eigen::VectorXd* targets,
                                   const Kernel::Ptr& kernel,
                                   double noise):
                points_(points),
                targets_(targets),
                kernel_(kernel),
                noise_(noise){
                    pi_trained_ = new Eigen::VectorXd(points_->size());
                    W_sr_trained_ = new Eigen::VectorXd(points_->size());
                    LLT_trained_ = new Eigen::MatrixXd(points_->size(),points_->size());};


bool TrainingLogLikelihoodClassification::Evaluate(const double* const parameters, double* cost, double* gradient) const{
    // Update the kernel
    for(size_t ii = 0; ii < NumParameters(); ii++){
        kernel_-> SetParams(ii, parameters[ii]);
    }

    // Create a new GP model and extract computed variables.
    GaussianProcessClassification gpc(kernel_, noise_, points_, *targets_, points_->size());
    // Eigen::SparseMatrixBase<long double> f_;
    Eigen::VectorXd f_ = Eigen::VectorXd::Zero(points_->size());
    long double log_marginal_likelihood_ = -1e8;
    long max_iteration_ = 1e10;
    Eigen::MatrixXd K = gpc.ImmutableCovariance();
    // std::cout << "Covariance is " << std::endl << K << std::endl;

    // for(size_t kk = 0; kk < points_->size(); kk++){
    //     pi_trained_(kk) = 0.0;
    // }
    
    Eigen::VectorXd pi_(points_->size());
    Eigen::VectorXd W_(points_->size());
    Eigen::VectorXd W_sr_(points_->size());
    Eigen::MatrixXd W_sr_K_(points_->size(),points_->size());
    Eigen::MatrixXd W_sr_K_W_sr_(points_->size(),points_->size());
    Eigen::VectorXd b_(points_->size());
    Eigen::VectorXd a_(points_->size());
    Eigen::LLT<Eigen::MatrixXd> L_;
    Eigen::MatrixXd LLT_;
    // Based on Algorithm 3.1
    for (int t = 0; t < max_iteration_; t++){
        // Line 4
        for(size_t ii = 0; ii < points_->size(); ii++){
            pi_(ii) = 1.0/(1.0 + std::exp(-f_(ii)));
            W_(ii) = pi_(ii) * (1.0 - pi_(ii));
            W_sr_(ii) = std::sqrt(W_(ii));
        }
        // std::cout << "pi_ is " << std::endl << pi_ << std::endl;
        // std::cout << "W is " << std::endl << W_ << std::endl;
        // std::cout << "W_sr is " << std::endl << W_sr_ << std::endl;
        // std::cout << "K is " << std::endl << K << std::endl;
        // Line 5
        // Element wise dot product
        for (int ii = 0; ii < W_sr_.size(); ii++){
            W_sr_K_.row(ii) = W_sr_(ii) * K.row(ii);
            W_sr_K_W_sr_.row(ii) = W_sr_(ii) * K.row(ii) * W_sr_(ii);
        }
        // std::cout << "W_sr_K is " << std::endl << W_sr_K_ << std::endl;
        // std::cout << "W_sr_K_sr_ is " << std::endl << W_sr_K_W_sr_ << std::endl;
        Eigen::MatrixXd B_ = Eigen::MatrixXd::Identity(points_->size(),points_->size());
        B_ = B_ + W_sr_K_W_sr_;
        // std::cout << "B_ is " << std::endl << B_ << std::endl;
        L_.compute(B_.topLeftCorner(points_->size(),points_->size()));
        LLT_ = L_.matrixL();
        // std::cout << "LLT is " << std::endl << LLT_ << std::endl;
        // Line 6
        for(int ii = 0; ii < points_->size(); ii++){
            b_(ii) = W_(ii) * f_(ii);
        }
        b_ = b_ + *targets_ - pi_;
        // std::cout << "b is " << std::endl << b_ << std::endl;
        // Line 7
        Eigen::VectorXd D_ = L_.solve(W_sr_K_ * b_);
        // std::cout << "D_ is " << std::endl << D_ << std::endl;
        for(int ii = 0; ii < D_.size(); ii++){
            a_(ii) = b_(ii) - W_sr_(ii) * D_(ii);
        }
        // Line 8
        f_ = K * a_;
        // std::cout << "a is " << std::endl << a_ << std::endl;
        // std::cout << "f_ is " << std::endl << f_ << std::endl;
        // Line 10: Update the log mariginal likelihood 
        double lml = -0.5 * a_.transpose() * f_;
        // std::cout << "1st item of lml is " << std::endl << lml << std::endl;
        double log_term = 0.0;
        for(int ii = 0; ii < points_->size(); ii++){
            log_term += -std::log(1.0 + std::exp(-((*targets_)(ii) * 2.0 - 1.0) * f_(ii)));
        }
        lml += log_term;
        // std::cout << "2nd item of lml is " << std::endl << log_term << std::endl;
        double L_diag_sum = 0.0;
        for(int ii = 0; ii < points_->size(); ii++){
            L_diag_sum += std::log(LLT_(ii,ii));
        }
        lml -= L_diag_sum;
        // std::cout << "3rd item of lml is " << std::endl << L_diag_sum << std::endl;
        // std::cout << "The log likelihood is " << lml << std::endl;
        if(lml - log_marginal_likelihood_ < 1e-10){
            break;
        }
        log_marginal_likelihood_ = lml;
    }
    *cost = log_marginal_likelihood_;
    *pi_trained_ = pi_;
    *W_sr_trained_ = W_sr_;
    *LLT_trained_ = LLT_;
 
    if (gradient){
        // std::cout << "The gradient is " << std::endl << gpc.ImmutableCovarianceGradient() << std::endl;
        // Update gradient based on Algorithm 5.1
        Eigen::MatrixXd R_(points_->size(),points_->size());
        Eigen::MatrixXd R_r_(points_->size(),points_->size());
        Eigen::MatrixXd W_sr_diag_ = W_sr_.asDiagonal();
        R_r_ = L_.solve(W_sr_diag_);
        // std::cout << "W_sr_diag is " << std::endl << W_sr_diag_ << std::endl;
        // std::cout << "L_ is " << std::endl << LLT_ << std::endl;
        // std::cout << "R_r_ is " << std::endl << R_r_ << std::endl;
        for(int ii = 0; ii < points_->size(); ii++){
            R_.row(ii) = W_sr_(ii) * R_r_.row(ii);
        }
        // std::cout << "R is "<< std::endl << R_ << std::endl;
        Eigen::MatrixXd C_(points_->size(), points_->size());
        C_ = LLT_.colPivHouseholderQr().solve(W_sr_K_);
        // C_ = L_.solve(W_sr_K_);
        // std::cout << "W_sr_K is " << std::endl << W_sr_K_ << std::endl;
        // std::cout << "C is " << std::endl << C_ << std::endl;
        
        // Line 9
        Eigen::MatrixXd C_T_C_ = C_.transpose() * C_;
        Eigen::VectorXd s_2_(points_->size());
        for(int ii = 0; ii < points_->size(); ii++){
            s_2_(ii) = -0.5 * (K(ii,ii) - C_T_C_(ii,ii)) * pi_(ii) * (1.0 - pi_(ii)) * (1.0 - 2.0 * pi_(ii));
        }
        // std::cout << "s_2_ is " << std::endl << s_2_ << std::endl;

    
        Eigen::MatrixXd K_grad = gpc.ImmutableCovarianceGradient();
        int num_param = NumParameters() - 1;
        Eigen::MatrixXd d_Z_ = Eigen::MatrixXd::Zero(num_param,1);
        Eigen::VectorXd s_3_(points_->size());
        for (size_t ii = 0; ii < num_param; ii++){
            C_ = K_grad;
            // std::cout << "C is " << std::endl << C_ << std::endl;
            Eigen::MatrixXd R_T_ = R_.transpose();
            int num_element = R_.cols() * R_.rows();
            Eigen::VectorXd R_T_ravel_ = Eigen::VectorXd(num_element);
            int ravel_idx = 0;
            for(size_t rr = 0; rr < R_T_.rows(); rr++){
                for(size_t cc = 0; cc < R_T_.cols(); cc++){
                    R_T_ravel_(ravel_idx) = R_T_(rr,cc);
                    ravel_idx ++;
                }
            }
            int num_ele_C = C_.cols() * C_.rows();
            Eigen::VectorXd C_ravel_ = Eigen::VectorXd(num_ele_C);
            int ravel_C_idx = 0;
            for(size_t rr = 0; rr < C_.rows(); rr++){
                for(size_t cc = 0; cc < C_.cols(); cc++){
                    C_ravel_(ravel_C_idx) = C_(rr,cc);
                    ravel_C_idx ++;
                }
            }

            double s_1_ = 0.5 * a_.transpose() * C_ * a_;
            // std::cout << "1st item of s_1 is " << s_1_ << std::endl;
            for(size_t kk = 0; kk < R_T_ravel_.size(); kk++){
                s_1_ -= 0.5 * R_T_ravel_(kk) * C_ravel_(kk);
            }
            // std::cout << "s_1 is " << std::endl << s_1_ << std::endl;
            
            b_ = C_ * (*targets_ - pi_);
            s_3_ = b_ - K * (R_ * b_);
            // std::cout << "s_3 is " << std::endl << s_3_ << std::endl;
            d_Z_(ii) = s_1_ + s_2_.transpose() * s_3_;
            // std::cout << "s_2_.transpose() * s_3_ " << s_2_.transpose() * s_3_ << std::endl;
            gradient[ii + 1] = d_Z_(ii);
            // std::cout << "d_Z is " << d_Z_(ii) << std::endl; 
        }
    }
    return true;
}

bool TrainingLogLikelihoodClassification::Optimizer(const size_t kDim, double* parameters, double* cost, double* gradient){
    const double kMaxError = 1e-8;
    const double alpha = 0.01;
    int ii = 1;

    bool foundlocalmax[kDim] = {false,false};
    while(foundlocalmax[ii] != true){
        Evaluate(parameters,cost,gradient);
        // std::cout << "The lml value is " << *cost << std::endl;
        // std::cout << "The gradient is " << gradient[ii] << std::endl;

        double cost_prev = *cost;
        parameters[ii] += alpha * gradient[ii];
        // std::cout << "Parameter is " << parameters[ii] << std::endl;
        
        // Update the objective fun and gradient with new parameters
        Evaluate(parameters,cost,gradient);
        // std::cout << "The lml value is " << *cost << std::endl;
        // std::cout << "The gradient is " << gradient[ii] << std::endl;
        // std::cout << "============================================" << std::endl;

        if(std::fabs(cost_prev - *cost) <= kMaxError){
            std::cout << "============================================" << std::endl;
            std::cout << "The local maximum is found: " << *cost << std::endl;
            std::cout << "The trained parameter is " << parameters[ii] << std::endl;
            foundlocalmax[ii] = true;
        }
    }

    return foundlocalmax[ii];
}

