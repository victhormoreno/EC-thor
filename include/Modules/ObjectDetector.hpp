/**
 * @file ObjectDetector.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the ObjectDetector module.
 * @version 2.0
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2023 BCN eMotorsport
 * 
 */

#ifndef OBJECTDETECTOR_HPP
#define OBJECTDETECTOR_HPP

#include <vector>

#include "Objects/Cluster.hpp"
#include "Objects/Params.hpp"
#include "Utils/FEC.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

/**
 * @brief The ObjectDetector module .
 * It is responsible for:
 * - Clustering detection
 */

class ObjectDetector {
  private:
    /* ------------------ Private Constructor And Destructor ------------------ */
    ObjectDetector();
    ~ObjectDetector();

    /* -------------------------- Private Utils -------------------------- */
    FEC fec;
    
    /* -------------------------- Private Attributes -------------------------- */
    float eps_;
    int maxPts_, minPts_;
    bool mode_;

    Params::ObjectDetector::PostProc::FSGCone fsg_cone;
    Params::ObjectDetector::PostProc::FSGBigCone fsg_big_cone;
    Params::ObjectDetector::PostProc::Distr distr;

    std::vector<Cluster> pre_clusters, clusters;
    std::vector<pcl::PointXYZI> clustersCentroids;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    /* ---------------------------- Private Methods --------------------------- */

    /**
     * @brief It uses the Fast Euclidean Clustering algorithm.
     * 
     */
    void fastEuclideanClustering();

  public:
    /* --------------------------- Singleton Pattern -------------------------- */
    static ObjectDetector& getInstance() {
      static ObjectDetector instance;
      return instance;
    }

    ObjectDetector(ObjectDetector const &) = delete;
    void operator=(ObjectDetector const &) = delete;

    /* -------------------------- Getters --------------------------- */
    inline std::vector<Cluster> &getClusters() { return clusters; }
    inline pcl::PointCloud<pcl::PointXYZI>::Ptr getCloudPoints() { return cloud; }

    /* ---------------------------- Public Methods ---------------------------- */
   
    /**
     * Initializes the point cloud from a vector, and the octree.
     * 
     * @param[in] params The parameters for the ObjectDetector module.
     */
    void init(const Params::ObjectDetector &params);
    
    /**
     * Update the cloud with the data of the vector.
     * 
     * @param[in] buffer The parameters for the ObjectDetector module.
     */
    void update(const std::vector<pcl::PointXYZI> buffer);
    void update(const pcl::PointCloud<pcl::PointXYZI>::Ptr &buffer);

    void generateClusters();

    std::vector<Cluster> postProc(std::vector<Cluster> &clusters);
};

#endif
