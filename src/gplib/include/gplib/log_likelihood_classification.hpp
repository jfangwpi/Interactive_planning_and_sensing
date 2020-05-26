#ifndef LOGLIKELIHOODCLASSIFICATION_HPP
#define LOGLIKELIHOODCLASSIFICATION_HPP

#include "gplib/gaussian_process_classification.hpp"
#include "gplib/rbf_kernel.hpp"

#include <ceres/ceres.h>
#include <math.h>

namespace librav{
    // Compute twice the negative log-likelihood of the training data of a GP
    // and the gradient against all the parameters of the kernel.
    class TrainingLogLikelihoodClassification: public ceres::FirstOrderFunction{
        public: 
            // Inputs: training points, training targets, kernel, and noise.
            // Optimization variables: kernel parameters.
            TrainingLogLikelihoodClassification(const std::shared_ptr<std::vector<Eigen::VectorXd>>& points,
                                   const Eigen::VectorXd* targets,
                                   const Kernel::Ptr& kernel,
                                   double noise);

            // Evaluate objective fun and gradient.
            bool Evaluate(const double* const parameters, double* cost, double* gradient) const;
            bool Optimizer(const size_t kDim, double* parameters, double* cost, double* gradient);

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
            
        public:
            Eigen::VectorXd *pi_trained_;
            Eigen::VectorXd *W_sr_trained_;
            Eigen::MatrixXd *LLT_trained_;

    };

}

#endif /* LOGLIKELIHOODCLASSIFICATION_HPP */