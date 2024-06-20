/**
 * @file Accumulator.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Accumualtor module.
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#ifndef ACCUMULATOR_HPP
#define ACCUMULATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"

#include "objects/Buffer.hpp"
#include "objects/Params.hpp"
#include "utils/Waggner.hpp"

typedef std::vector<pcl::PointXYZI> Points;
typedef pcl::PointXYZI Point;


/**
 * @brief The Accumulator module is what stores all data 
 * and prepares it for detection stage.
 * It is responsible for:
 * - Data accumulation
 * - Point cloud geometry filtering
 */

class Accumulator{
    private:
        /* ------------------ Private Constructor And Destructor ------------------ */
       
        Accumulator();
        ~Accumulator();

        /* -------------------------- Private Attributes -------------------------- */
        
        float lastx_, lasty_, lastz_;
        float last_yaw_;

        bool filter_;
        float radius_, field_of_view_;

        Buffer<Point> *buffer_; 
        int size_;

        /* ---------------------------- Private Methods --------------------------- */
       
        void add(Point);
        void add(Points);

        /**
         * @brief Converts the buffer to a point cloud doing range and angle-based filtering
         * 
         */
        void buffer2PointCloud(pcl::PointCloud<Point>::Ptr &cloud);

        /**
         * @brief Applies a range and angle-based filtering to the point cloud.
        */
        void radialFiltering(pcl::PointCloud<Point>::Ptr &cloud);

        /**
         * @brief Determines if a point is included in a semicercle.
        */
        bool includedInSemicercle(Point p);


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
         * @brief Receives an obstacle point cloud and stores it in the buffer.
         * 
         * @param obstacles The obstacle point cloud.
         */
        void receiveObstacles(pcl::PointCloud<MyPointType>::Ptr &obstacles);

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
        void getCloud(pcl::PointCloud<Point>::Ptr &cloud);
     
        /**
         * @brief Determines if the Buffer buff_ has valid data (not in use).
         */
        inline bool buffHasValidData() const{
            return !buffer_->empty();
        };


};
#endif 