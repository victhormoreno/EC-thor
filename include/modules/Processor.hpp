/**
 * @file Processor.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Processor module.
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#ifndef Processor_HPP
#define Processor_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "objects/Params.hpp"
#include "utils/Waggner.hpp"

/**
 * @brief The Processor module is responsible to segment input pointcloud.
 * It is responsible for:
 * - Grid map creation
 * - Walls Removing 
 * - Ground Removing
 */

class Processor{
    private:
        /* ------------------ Private Constructor And Destructor ------------------ */
        Processor();
        ~Processor();

        /* -------------------------- Private Attributes -------------------------- */

        Waggner *waggner; /**< Waggner object */

        bool remove_walls_; /**< Flag to remove walls */
        int init_iteration_; /**< Number of init iterations */

        /* ---------------------------- Private Methods --------------------------- */

    public:
        /* --------------------------- Singleton Pattern -------------------------- */
        static Processor& getInstance() {
            static Processor proc;
            return proc;
        }
        Processor(const Processor&) = delete;
        Processor& operator=(const Processor&) = delete;

        /* ---------------------------- Public Methods ---------------------------- */

        /**
         * @brief It initalizes the Processor module.
         * 
         * @param params  
         */
        void init(const Params::Processor &params);

        /**
         * @brief Filters the walls and ground from the input cloud
         * 
         * @param cloud_in 
         * @param walls
         * @param ground 
         * @param obstacles 
         */ 
        void process(pcl::PointCloud<LimoPoint>::Ptr &cloud_in, 
                    pcl::PointCloud<MyPointType>::Ptr &walls,
                    pcl::PointCloud<MyPointType>::Ptr &ground,
                    pcl::PointCloud<MyPointType>::Ptr &obstacles);

        void getGrid(nav_msgs::OccupancyGrid &grid);
};
#endif 