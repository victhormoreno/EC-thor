/**
 * @file Cluster.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Cluster class.
 * @version 2.0
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2023 BCN eMotorsport
 * 
 */

#ifndef CLUSTER_HPP
#define CLUSTER_HPP

#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <omp.h>

class Cluster {
    private:
        /* -------------------------- Private Attributes -------------------------- */
        int id_;
        pcl::PointXYZI centroid_;
        
        /* ---------------------------- Private Methods --------------------------- */
        void calculateCentroid();

    public:
        /* ------------------ Constructor And Destructor ------------------ */
        Cluster();
        ~Cluster();

        /* -------------------------- Getters --------------------------- */
        inline pcl::PointXYZI getCentroid() const { 
            return centroid_; 
        }

        /* -------------------------- Public Attributes -------------------------- */
        std::vector<pcl::PointXYZI> points_;

       /* ---------------------------- Public Methods ---------------------------- */
        double maxDistance(); 
        double zDistribution();
        double xDistribution();
        double yDistribution();
        void xyzDistribution(std::vector<double> *xyz);
        static void computeCentroids(std::vector<Cluster> &c, std::vector<pcl::PointXYZI> &centroides){
            #pragma omp parallel for
            for (int i = 0; i < c.size(); i++) {
                c[i].calculateCentroid();
            }
            for (int i = 0; i < c.size(); i++) {
                centroides.push_back(c[i].getCentroid());
            }
        }
};

#endif  