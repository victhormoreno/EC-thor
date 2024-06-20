/**
 * @file Cluster.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Cluster class.
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#ifndef CLUSTER_HPP
#define CLUSTER_HPP

#include "objects/Descriptor.hpp"

#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <omp.h>

class Cluster {
    private:
        /* -------------------------- Private Attributes -------------------------- */

        int id_;
        std::vector<pcl::PointXYZI> points_;
        static std::vector<float> centroides;

        // static int num_clusters;
        bool compute_descriptors;
        Descriptor<pcl::PointXYZI> descriptor_;

        /* ---------------------------- Private Methods --------------------------- */
        
    public:
        /* ------------------ Constructor And Destructor ------------------ */
        
        Cluster(bool compute_descriptors = true) : compute_descriptors(compute_descriptors) {
            if(compute_descriptors) this->descriptor_ = Descriptor<pcl::PointXYZI>();
        }

        /* -------------------------- Getters --------------------------- */
        inline int size() const { return this->points_.size(); }

        inline float getCentroid(int i) const { return descriptor_.mean(i); }
        inline float getVariance(int i) const { return descriptor_.variance(i); }
        inline float getStandardDeviation(int i) const { return descriptor_.stdev(i); }        
        inline float getSkewness(int i) const { return descriptor_.skewPearson(i); }
        inline float getKurtosis(int i) const { return descriptor_.kurtosis(i); }

        std::vector<pcl::PointXYZI>::const_iterator begin() const {
            return points_.begin();
        }

        std::vector<pcl::PointXYZI>::const_iterator end() const {
            return points_.end();
        }

       /* ---------------------------- Public Methods ---------------------------- */

       void addPoint(const pcl::PointXYZI &point) {
            this->points_.push_back(point);
            if(this->compute_descriptors) this->descriptor_.addPoint(point);
        }

};

#endif  