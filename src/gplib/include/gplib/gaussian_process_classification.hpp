#ifndef GAUSSIAN_PROCESS_CLASSIFICATION_HPP
#define GAISSIAN_PROCESS_CLASSIFICATION_HPP

#include "gplib/kernel.hpp"
#include <Eigen/Cholesky>
#include <vector>

namespace librav{
    const int64_t kDimension = 2;

    class GaussianProcessClassification{
        public:
            ~GaussianProcessClassification() {};
            // The keyword explicit avoid the implicit type conversion constructor
            explicit GaussianProcessClassification(const Kernel::Ptr& kernel, double noise, const std::shared_ptr<std::vector<Eigen::VectorXd>> points, const Eigen::VectorXd& targets, size_t max_points = 100);

            // Evaluate mean and variance at a point
            void Evaluate(const Eigen::VectorXd& x, double& mean, double& variance) const;
            void EvaluateTrainingPoint(size_t ii, double& mean, double& variance) const;

            // Add new point(s). Return whether or not points were added (points will
            // only be added until 'max_points' is reached).
            bool Add(const Eigen::VectorXd& x, double target);
            bool Add(const std::vector<Eigen::VectorXd>& points, const Eigen::VectorXd& targets);

            // Update the training targets in the direction of the gradient of the
            // mean squared error at the given points. Returns the mean squared error.
            // If 'finalize' is set, computes regressed targets - only set to false if
            // you are doing repeated updates, and be sure to set true on final update.
            double UpdateTargets(const std::vector<Eigen::VectorXd>& points,
                                 const std::vector<double>& targets,
                                 double step_size, bool finalize = true);

            std::pair<bool,double> Predict(const Eigen::VectorXd& x, Eigen::VectorXd* pi_, Eigen::VectorXd *W_sr_, Eigen::MatrixXd *LLT_);
            
            // Immutable accessors
            const Eigen::MatrixXd& ImmutableCovariance() const { return covariance_; };
            const Eigen::MatrixXd& ImmutableCovarianceGradient() const {return covariance_gradient_;};
            const Eigen::VectorXd& ImmutableRegressedTargets() const { return regressed_; };
            const Eigen::VectorXd& ImmutableTargerts() const { return targets_; };
            const std::shared_ptr<const std::vector<Eigen::VectorXd>> ImmutablePoints() const { return points_; };
            const Eigen::LLT<Eigen::MatrixXd>& ImmutableCholesky() const { return llt_; };
            size_t Dimension() const { return dimension_; };

        private:
            // Compute the covariance and cross covariance against the trainning points.
            void Covariance();
            void CrossCovariance(const Eigen::VectorXd& x, Eigen::VectorXd& cross) const;
            void CovarianceGradient();
            Eigen::MatrixXd Covariance(const Eigen::VectorXd& target);

            // Kernel.
            const Kernel::Ptr kernel_;

            // Noise variance
            double noise_;

            // Trainning points, targets, and regressed targets (inv(cov) * targets).
            const std::shared_ptr<std::vector<Eigen::VectorXd>> points_;
            const std::shared_ptr<std::vector<int>> targets_y_;
            size_t dimension_;
            Eigen::VectorXd targets_;
            Eigen::VectorXd regressed_;

            // Maximum number of points.
            const size_t max_points_;

            // Covariance matrix, with Cholesky decomposition.
            Eigen::MatrixXd covariance_;
            Eigen::MatrixXd covariance_gradient_;
            Eigen::LLT<Eigen::MatrixXd> llt_;
    };
}

#endif /* GAUSSIAN_PROCESS_CLASSIFICATION_HPP */