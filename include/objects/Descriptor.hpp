/**
 * @file Descriptor.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Descritors object.
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#ifndef DESCRIPTOR_HPP
#define DESCRIPTOR_HPP

#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <pcl/point_types.h>

/**
 * @brief The Descritor object is .
 * It is responsible for:
 * - Compute the mean, variance, skewness, kurtosis, etc. of a set of points.
 * - Provide the minimum and maximum values of the set of points.
 * - Compute covariance matrix and the normal vector.
 * - Compute the correlation coefficient rho.
 */

template <typename PointT>
class Descriptor{
    private:

        static constexpr float static_max = std::numeric_limits<float>::max();
        static constexpr float static_min = std::numeric_limits<float>::lowest();

        /* -------------------------- Private Attributes -------------------------- */


        std::array<float, 3> mean_; /**< Mean of the points in each dimension */
        std::array<float, 3> M2_;   /**< Sum of squares of differences from the mean */
        std::array<float, 3> M3_;   /**< Sum of cubes of differences from the mean */
        std::array<float, 3> M4_;   /**< Sum of fourth powers of differences from the mean */
        std::array<float, 3> C_;    /**< Sum of cross products of differences from the mean */

        std::array<float, 3> min_{static_max,static_max,static_max}; /**< Minimum value in each dimension */
        std::array<float, 3> max_{static_min,static_min,static_min}; /**< Maximum value in each dimension */

        long long points_; /**< Number of points */


        /* ---------------------------- Private Methods --------------------------- */

        /**
         * @brief Updates the descriptor with a new point.
         * 
         * This function updates the descriptor with a new point by
         * updating the sum of n-th powers of differences from the mean.
         * 
         * @param p Point to add to the descriptor.
         * 
         * @see https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
         * 
         */
        void update(const PointT &p){

            static std::array<float, 3> p_;
            p_[0] = p.x;
            p_[1] = p.y;
            p_[2] = p.z;

            for(int i = 0; i < 3; i++){
                min_[i] = std::min(p_[i], min_[i]);
                max_[i] = std::max(p_[i], max_[i]);
                
                float delta = p_[i] - mean_[i];
                float delta_n = delta / points_;
                float delta_n2 = delta_n * delta_n;
                float term1 = delta * delta_n * (points_ - 1);

                mean_[i] += delta_n;
                M4_[i] += term1 * delta_n2 * (points_ * points_ - 3 * points_ + 3) + 6 * delta_n2 * M2_[i] - 4 * delta_n * M3_[i];
                M3_[i] += term1 * delta_n * (points_ - 2) - 3 * delta_n * M2_[i];
                M2_[i] += term1;

                // Covariance matrix
                for (int j = i + 1; j < 3; j++) {
                    float mean_j = mean_[j] + (p_[j] - mean_[j]) / points_;
                        C_[i+j-1] += delta * (p_[j] - mean_j);
                }
            }
        }


        /**
         * @brief It computes the normal vector of the descriptor.
         * 
         * @return Eigen::Vector3f 
         */
        Eigen::Vector3f computeNormal(){
            Eigen::Matrix3f Sigma = covarianceMatrix();

            Eigen::JacobiSVD<Eigen::Matrix3f> svd(Sigma, Eigen::DecompositionOptions::ComputeFullU);
            Eigen::Vector3f singular_values = svd.singularValues();

            Eigen::Vector3f normal = (svd.matrixU().col(2));
            if (normal(2) < 0) { for(int i=0; i<3; i++) normal(i) *= -1; }
            return normal;
        }

    public:

        /* ---------------------------- Constructor ---------------------------- */

        Descriptor() : points_(0), mean_({0,0,0}), M2_({0,0,0}), 
                        M3_({0,0,0}), M4_({0,0,0}), C_({0,0,0}) {} 

        /* ---------------------------- Getters ---------------------------- */

        /**
         * @brief It returns the number of points in the descriptor.
         * 
         * @return int 
         */
        inline long long points() const {
            return points_;
        }

        /**
         * @brief It returns the minimum value of the descriptor.
         * 
         * @param i x=0, y=1, z=2
         * @return float 
         */
        inline float min(int i) const {
            return min_[i];
        }

        /**
         * @brief It returns the maximum value of the descriptor.
         * 
         * @param i x=0, y=1, z=2
         * @return float 
         */
        inline float max(int i) const {
            return max_[i];
        }

        /**
         * @brief It returns the mean of the descriptor.
         * 
         * @param i x=0, y=1, z=2
         * @return float 
         */
        inline float mean(int i) const { 
            return mean_[i]; 
        }

        /**
         * @brief It returns the median of the descriptor.
         * 
         * @param i x=0, y=1, z=2
         * @return float 
         */
        inline float median(int i) const {
            return (max_[i] + min_[i]) / 2;
        }
        
        /**
         * @brief It returns the standard deviation of the descriptor.
         * 
         * @param i x=0, y=1, z=2
         * @return float 
         */
        inline float variance(int i) const { 
            if(points_ < 2) return std::numeric_limits<float>::quiet_NaN();
            return M2_[i] / (points_ - 1);
        }

        /**
         * @brief It returns the standard deviation of the descriptor.
         * 
         * @param i x=0, y=1, z=2
         * @return float 
         */
        inline float stdev(int i) const { 
            if(points_ < 2) return std::numeric_limits<float>::quiet_NaN();
            return sqrt(variance(i));
        }

        /**
         * @brief It returns the standard deviation of the descriptor.
         * 
         * @param i x=0, y=1, z=2
         * @return float 
         */
        inline float skewness(int i) const { 
            if(points_ < 3) return std::numeric_limits<float>::quiet_NaN();
            return sqrt(float(points_)) * M3_[i] / std::pow(M2_[i], 1.5);
        }

        /**
         * @brief It returns the Pearson skewness of the descriptor.
         * 
         * @param i x=0, y=1, z=2
         * @return float
        */
        inline float skewPearson(int i) const {
            if(points_ < 2) return std::numeric_limits<float>::quiet_NaN();
            return 3*(mean_[i] - median(i))/stdev(i);
        }

        /**
         * @brief It returns the kurtosis of the descriptor.
         * 
         * @param i x=0, y=1, z=2
         * @return float 
         */
        inline float kurtosis(int i) const {  
            if(points_ < 4) return std::numeric_limits<float>::quiet_NaN();
            return (float(points_) * M4_[i]) / (M2_[i] * M2_[i]) - 3.0;
        }

        /**
         * @brief It returns the covariance of the descriptor.
         * 
         * @param i 
         * @return float 
         */
        inline float covariance(int i) const {
            if(points_ < 2) return std::numeric_limits<float>::quiet_NaN();
            return C_[i] / (points_ - 1);
        }

        /**
         * @brief It returns the covariance matrix of the descriptor.
         * 
         * @return Eigen::Matrix3f 
         */
        Eigen::Matrix3f covarianceMatrix() const {
            if(points_ < 2) return Eigen::Matrix3f::Zero();
            Eigen::Matrix3f Sigma;
            Sigma << variance(0), covariance(0), covariance(1),
                    covariance(0),  variance(1), covariance(2),
                    covariance(1), covariance(2),  variance(2);
            return Sigma;
        }

        /**
         * @brief It returns the normal vector of the descriptor.
         * 
         * @return Eigen::Vector3f 
         */
        Eigen::Vector3f normal() {
            if(points_ < 3) return Eigen::Vector3f::Zero();
            return computeNormal();
        }

        /**
         * @brief It returns the correlation coefficient of the descriptor.
         * 
         * @param i 
         * @param j 
         * @return float 
         */
        float rho(int i, int j) const {
            if(points_ < 2) return std::numeric_limits<float>::quiet_NaN();
            Eigen::Matrix3f Sigma = covarianceMatrix();
            return Sigma(i,j) / (stdev(i) * stdev(j));
        }

        /* ---------------------------- Public Methods ---------------------------- */

        /**
         * @brief It adds a point to the descriptor.
         * 
         * @param p 
         */
        void addPoint(const PointT &p){
            points_++;
            update(p);
        }

};

#endif 