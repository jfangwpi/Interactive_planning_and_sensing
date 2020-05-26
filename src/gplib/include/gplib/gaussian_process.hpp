#ifndef GAUSSIAN_PROCESS_HPP
#define GAISSIAN_PROCESS_HPP

#include "gplib/kernel.hpp"
#include <Eigen/Cholesky>
#include <vector>

namespace librav{

    class GaussianProcess{
        public:
            ~GaussianProcess() {};
            // The keyword explicit avoid the implicit type conversion constructor
            explicit GaussianProcess(const Kernel::Ptr& kernel, double noise, const std::shared_ptr<std::vector<Eigen::VectorXd>> points, const Eigen::VectorXd& targets, size_t max_points = 100);

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

            // Learn kernel hyperparameters by maximizing log-likelihood of the
            // training data.
            bool LearnHyperparams();

            // Immutable accessors
            const Eigen::MatrixXd& ImmutableCovariance() const { return covariance_; };
            const Eigen::VectorXd& ImmutableRegressedTargets() const { return regressed_; };
            const Eigen::VectorXd& ImmutableTargerts() const { return targets_; };
            const std::shared_ptr<const std::vector<Eigen::VectorXd>> ImmutablePoints() const { return points_; };
            const Eigen::LLT<Eigen::MatrixXd>& ImmutableCholesky() const { return llt_; };
            size_t Dimension() const { return dimension_; };

        private:
            // Compute the covariance and cross covariance against the trainning points.
            void Covariance();
            void CrossCovariance(const Eigen::VectorXd& x, Eigen::VectorXd& cross) const;

            // Kernel.
            const Kernel::Ptr kernel_;

            // Noise variance
            double noise_;

            // Trainning points, targets, and regressed targets (inv(cov) * targets).
            const std::shared_ptr<std::vector<Eigen::VectorXd>> points_;
            size_t dimension_;
            Eigen::VectorXd targets_;
            Eigen::VectorXd regressed_;

            // Maximum number of points.
            const size_t max_points_;

            // Covariance matrix, with Cholesky decomposition.
            Eigen::MatrixXd covariance_;
            Eigen::LLT<Eigen::MatrixXd> llt_;
    };
}

#endif /* GAUSSIAN_PROCESS_HPP */