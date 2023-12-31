/**
 * @file GroundSegmenter.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the GroundSegmenter module.
 * @version 2.0
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2023 BCN eMotorsport
 * 
 */

#ifndef GROUNDSEGMENTER_HPP
#define GROUNDSEGMENTER_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/PointCloud2.h"

#include "Objects/Params.hpp"
#include "Utils/Ransac.hpp"


/**
 * @brief The GroundSegmenter module is responsible to segment input pointcloud.
 * It is responsible for:
 * - Ground Segmentation
 */

class GroundSegmenter{
    private:
        /* ------------------ Private Constructor And Destructor ------------------ */
        GroundSegmenter();
        ~GroundSegmenter();

        /* -------------------------- Private Attributes -------------------------- */
        bool mode_; 
        Ransac ransac;

        /* ---------------------------- Private Methods --------------------------- */

    public:
        /* --------------------------- Singleton Pattern -------------------------- */
        static GroundSegmenter& getInstance() {
            static GroundSegmenter grdseg;
            return grdseg;
        }
        GroundSegmenter(const GroundSegmenter&) = delete;
        GroundSegmenter& operator=(const GroundSegmenter&) = delete;

        /* ---------------------------- Public Methods ---------------------------- */

        /**
         * @brief It initalizes the module.
         * 
         * @param params  
         */
        void init(const Params::GroundSegmenter &params);

        /**
         * @brief It segments the input pointcloud.
         * 
         * @param cloud_in 
         * @param ground 
         * @param obstacles 
         */ 
        void segment(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &ground
                                                                    ,pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles);
};
#endif 