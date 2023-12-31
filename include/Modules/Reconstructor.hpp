/**
 * @file Reconstructor.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Reconstructor module.
 * @version 2.0
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2023 BCN eMotorsport
 * 
 */

#ifndef RECONSTRUCTOR_HPP
#define RECONSTRUCTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "Objects/Params.hpp"
#include "Objects/Cluster.hpp"
#include "Utils/ikdTree.hpp"

/**
 * @brief The Reconstructor module .
 * It is responsible for:
 * - Storing the point cloud in an ikdTree
 * - Reconstructing the clusters
 */


using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>::Ptr;
using PointVector = std::vector<PointType,Eigen::aligned_allocator<PointType>>;

class Reconstructor {
  private:
    /* -------------------------- Private Constructor and Destructor -------------------------- */
    Reconstructor()  {}
    ~Reconstructor() {}

    /* -------------------------- Private Utils -------------------------- */
    KD_TREE<PointType>::Ptr ikd_Tree;

    
    /* -------------------------- Private Attributes -------------------------- */
    float delete_param, balance_param, box_length;
    PointVector increment, decrement;
    
    int k_nearest; // K-nearest 
    double max_dist; // K-nearest 
    float radius;  // Radius search

    /* ---------------------------- Private Methods --------------------------- */
    bool exists();
    int size();
    PointType add_intensity(pcl::PointXYZ point);
    void add(PointCloud &points, bool downsample);
    void add_points(bool downsample);
    void delete_points();
    void build_tree(PointCloud &points);
    void reconstruct(pcl::PointXYZI centroid, Cluster &cluster);
    void generate_increment_cloud(PointCloud cloud);
    void generate_decrement_cloud(PointCloud cloud);


  public:
    /* --------------------------- Singleton Pattern -------------------------- */
    static Reconstructor& getInstance() {
      static Reconstructor instance;
      return instance;
    }

    Reconstructor(Reconstructor const &) = delete;
    void operator=(Reconstructor const &) = delete;

    /* -------------------------- Getters --------------------------- */

    /* ---------------------------- Public Methods ---------------------------- */
   
    /**
     * Initializes the point cloud from a vector, and the octree.
     * 
     * @param[in] params The parameters for the Recontructor module.
     */
    void init(const Params::Reconstructor &params);

    void updateIkdTree(const sensor_msgs::PointCloud2 &cloud_msg);
    void reconstruct(std::vector<PointType> centroides, std::vector<Cluster> &clusters);
    
};

#endif