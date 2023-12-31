/**
 * @file Accumulator.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Accumualtor module.
 * @version 2.0
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2023 BCN eMotorsport
 * 
 */

#ifndef ACCUMULATOR_HPP
#define ACCUMULATOR_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"

#include "Objects/Buffer.hpp"
#include "Objects/Params.hpp"
#include "Utils/Preproc.hpp"

typedef std::vector<pcl::PointXYZI> Points;
typedef pcl::PointXYZI Point;


/**
 * @brief The Accumulator module is what stores all data.
 * It is responsible for:
 * - Data accumulation
 */

class Accumulator{
    private:
        /* ------------------ Private Constructor And Destructor ------------------ */
        Accumulator();
        ~Accumulator();

        /* -------------------------- Private Attributes -------------------------- */
        Preproc preproc;
        Buffer<Point> *buffer_, *raw_buffer_; 
        pcl::PointCloud<Point>::Ptr cloud_, raw_cloud_;
        int size_, raw_size_;

        pcl::VoxelGrid<Point> vxg;
        pcl::PassThrough<pcl::PointXYZI> pass;  

        /* ---------------------------- Private Methods --------------------------- */
        void add(Point);
        void add(Points);
        void buffer2PointCloud(bool raw);

    public:
        /* --------------------------- Singleton Pattern -------------------------- */
        static Accumulator& getInstance() {
            static Accumulator accum;
            return accum;
        }
        Accumulator(const Accumulator&) = delete;
        Accumulator& operator=(const Accumulator&) = delete;

        /* ---------------------------- Public Methods ---------------------------- */

        /**
         * @brief Initializes the Accumulator module.
         * 
         * @param params The parameters of the Accumulator module.
         */
        void init(const Params::Accumulator &params);

        /**
         * @brief Receives a PointCloud2 message and stores it in the buffer (not in use).
         * 
         * @param msg The PointCloud2 message.
         */
        void receiveLidar(const sensor_msgs::PointCloud2&);

        /**
         * @brief Receives an obstacle point cloud and stores it in the buffer.
         * 
         * @param obstacles The obstacle point cloud.
         */
        void receiveObstacles(pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles);

        /**
         * @brief Receives the state of the vehicle.
         * 
         * @param state The state of the vehicle.
         */
        void receiveState(const nav_msgs::Odometry::ConstPtr &state);

        /**
         * @brief Returns the point cloud stored in the buffer.
         * 
         * @return std::vector<Point> The point cloud stored in the buffer.
         */
        Points getPoints();

        /**
         * @brief Returns the point cloud stored in the buffer.
         * 
         * @return pcl::PointCloud<Point>::Ptr The point cloud stored in the buffer.
         */
        pcl::PointCloud<Point>::Ptr getCloud();

        /**
         * @brief Returns the raw point cloud stored in the buffer (not in use).
         * 
         * @return pcl::PointCloud<Point>::Ptr The raw point cloud stored in the buffer.
         */
        pcl::PointCloud<Point>::Ptr getRawCloud();
        
        /**
         * @brief Determines if the Buffer buff_ has valid data (not in use).
         */
        inline bool buffHasValidData() const{
            return !buffer_->empty();
        };


};
#endif 