/**
 * @file Ransac.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Ransac class.
 * @version 2.0
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2023 BCN eMotorsport
 * 
 */

#ifndef RANSAC_HPP
#define RANSAC_HPP

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>

#include "Objects/Params.hpp"

/**
 * @brief The Ransac class.
 * It is responsible for:
 * - 
 * - 
 */

class Ransac {
  private:
    /* -------------------------- Private Utils -------------------------- */

    /* -------------------------- Private Attributes -------------------------- */
    float minz_, maxz_;
    int max_iterations_;
    double dist_threshold_, plane_angle_;
    bool vis_outliers_;

    /* -------------------------- Private Methods --------------------------- */

  public:
    Ransac() = default;
    Ransac(const Params::GroundSegmenter &params);
    ~Ransac();
    /* -------------------------- Getters --------------------------- */
    /* ---------------------------- Public Methods ---------------------------- */

    void removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &ground
                                                                    ,pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles);
   
};

#endif
