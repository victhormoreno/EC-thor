/**
 * @file Detector.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Detector module.
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <vector>

#include "objects/Cluster.hpp"
#include "objects/Params.hpp"
#include "utils/FEC.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

/**
 * @brief The Detector module .
 * It is responsible for:
 * - Clustering stage 
 * - Classification stage
 */

class Detector {
  private:
    /* ------------------ Private Constructor And Destructor ------------------ */
    Detector();
    ~Detector();

    /* -------------------------- Private Utils -------------------------- */
    FEC fec;
    
    /* -------------------------- Private Attributes -------------------------- */
    float eps_;
    int maxPts_, minPts_;

    std::vector<Cluster> clusters;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    /* ---------------------------- Private Methods --------------------------- */

    /**
     * @brief It uses the Fast Euclidean Clustering algorithm.
     * 
     */
    void fastEuclideanClustering();

  public:
    /* --------------------------- Singleton Pattern -------------------------- */
    static Detector& getInstance() {
      static Detector instance;
      return instance;
    }

    Detector(Detector const &) = delete;
    void operator=(Detector const &) = delete;

    /* -------------------------- Getters --------------------------- */
    inline std::vector<Cluster> &getClusters() { return clusters; }
    inline pcl::PointCloud<pcl::PointXYZI>::Ptr getCloudPoints() { return cloud; }

    /* ---------------------------- Public Methods ---------------------------- */
   
    /**
     * Initializes the point cloud from a vector, and the octree.
     * 
     * @param[in] params The parameters for the Detector module.
     */
    void init(const Params::Detector &params);
    
    /**
     * Update the cloud with the data of the vector.
     * 
     * @param[in] buffer The parameters for the Detector module.
     */
    void update(const std::vector<pcl::PointXYZI> buffer);
    void update(const pcl::PointCloud<pcl::PointXYZI>::Ptr &buffer);

    void generateClusters(void);

    void classification(std::vector<Cluster> &clusters, std::vector<Cluster> &cones);
};

#endif
