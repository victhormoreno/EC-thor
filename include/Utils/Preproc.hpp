/**
 * @file Preproc.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Preproc module.
 * @version 2.0
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2023 BCN eMotorsport
 * 
 */

#ifndef PREPROC_HPP
#define PREPROC_HPP

#include "Objects/Params.hpp"
#include "nav_msgs/Odometry.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>


/**
 * @brief This module preprocesses all data that arrives in each iteration.
 * The aim of this module is to filter and clean the data so all the other
 * modules can use it directly.
 */

class Preproc {
 private:
  /* -------------------------- Private Attributes -------------------------- */
    bool vxg_active_; 
    float res_x, res_y, res_z;
    bool passthrough_active_, x_active_, y_active_, z_active_;
    float limx_, limy_, limz_;
    bool gridfilter_active;
    float box_dim, max_height;
    float lastx_, lasty_, lastz_;

  /* -------------------------- Private Utils -------------------------- */
  pcl::PassThrough<pcl::PointXYZI> pass;  
  pcl::VoxelGrid<pcl::PointXYZI> vxg;   

  /* ---------------------------- Private Methods --------------------------- */
  
  /**
   * @brief Down sampling the given point cloud using Voxel Grid
   * 
   */
  void downSampling(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

  /**
   * @brief Removing points too far away from the origin
   * 
   */
  void passThrough(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

  /**
   * @brief Filter points based on their heigh distribution over a xy grid
   * 
   */
  void gridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

  /* ---------------------------- Aux Methods --------------------------- */

  double zDistribution(pcl::PointCloud<pcl::PointXYZ>* leaf);

 public:
  /* --------------------------- Public Constructor and Destructor ------------------------- */
  Preproc() = default;
  Preproc(const Params::Accumulator &params);
  ~Preproc();

  /* --------------------------- Public Methodes -------------------------- */
  /**
   * @brief Preprocess all data from the buffer_.
   * 
   * @param[in] cloud are points of the buffer_.
   */
  void preprocess(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

  void updateState(const nav_msgs::Odometry::ConstPtr &state);


};

#endif  // MODULES_PREPROC_PREPROC_HPP