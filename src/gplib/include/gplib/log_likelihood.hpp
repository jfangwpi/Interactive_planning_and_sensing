#ifndef LOGLIKELIHOOD_HPP
#define LOGLIKELIHOOD_HPP

#include "gplib/gaussian_process.hpp"
#include "gplib/rbf_kernel.hpp"

#include <ceres/ceres.h>
#include <math.h>

namespace librav{
    // Compute twice the negative log-likelihood of the training data of a GP
    // and the gradient against all the parameters of the kernel.
    class TrainingLogLikelihood: public ceres::FirstOrderFunction {
        public: 
            // Inputs: training points, training targets, kernel, and noise.
            // Optimization variables: kernel parameters.
            TrainingLogLikelihood(const std::shared_ptr<std::vector<Eigen::VectorXd>>& points,
                                   const Eigen::VectorXd* targets,
                                   const Kernel::Ptr& kernel,
                                   double noise):
                points_(points),
                targets_(targets),
                kernel_(kernel),
                noise_(noise){};

            // Evaluate objective fun and gradient.
            bool Evaluate(const double* const parameters, double* cost, double* gradient) const{
                // Update the kernel
                for(size_t ii = 0; ii < NumParameters(); ii++){
                    kernel_-> SetParams(ii, parameters[ii]);
                }
                

                // Create a new GP model and extract computed variables.
                GaussianProcess gp(kernel_, noise_, points_, *targets_, points_->size());
                const Eigen::LLT<Eigen::MatrixXd>& llt = gp.ImmutableCholesky();
                const Eigen::VectorXd& regressed = gp.ImmutableRegressedTargets();
                const Eigen::MatrixXd& L = llt.matrixL();

                const size_t N = points_->size();

                // Compute log det of covariance matrix: log(|K(X,X)|)
                double logdet = 0.0;
                for(size_t ii = 0; ii < N; ii++){
                    logdet += std::log(L(ii,ii));
                }
        
                // Normalization constant
                double normconst = (points_->size() / 2) * std::log(2.0*M_PI);

                // Eqn (5.8): negative of the log marginal likelihood
                *cost = 0.5 * targets_->head(N).dot(regressed.head(N)) + logdet + normconst;

                // Update gradient
                if (gradient){
                    Eigen::MatrixXd dK(N,N);

                    for(size_t ii = 0; ii < NumParameters(); ii++){
                        for (size_t jj = 0; jj < N ; jj++){
                            dK(jj,jj) = kernel_->Partial(points_->at(jj),points_->at(jj),ii);

                            for (size_t kk = 0; kk < jj; kk++){
                                dK(jj,kk) = kernel_->Partial(points_->at(jj),points_->at(kk),ii);
                                dK(kk,jj) = dK(jj,kk);
                            }
                        }
                        // Compute the gradient. Eqn(5.9)
                        gradient[ii] = llt.solve(dK).trace() - 1.0 / parameters[ii] - regressed.head(N).dot(dK * regressed.head(N));
                    }
                }
                
                return true;
            }

            int NumParameters() const{
                return static_cast<int>(kernel_->ImmutableParams().size());
            }

            private:
                // Inputs: training points, training targets, kernel, and noise.
                // Optimization variables: kernel parameters.
                const std::shared_ptr<std::vector<Eigen::VectorXd>> points_;
                const Eigen::VectorXd* targets_;
                const Kernel::Ptr kernel_;
                double noise_;
    };

}

#endif /* LOGLIKELIHOOD_HPP */