/**
 * @file Waggner.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Remover module.
 * @version 2.0
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2023 BCN eMotorsport
 * 
 */

#ifndef WAGGNER_HPP
#define WAGGNER_HPP

#include <omp.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/common/copy_point.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include "objects/Descriptor.hpp"
#include "objects/Grid.hpp"
#include "objects/Params.hpp"

/**
 * @brief The Waggner module is what coordinates all different Modules.
 * It is responsible for:
 * - Dynamic reconfigure objects
 * - Pipeline calls
 * - Data publishing
 */

struct MyPointType {
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    float intensity;
    float range;
    double timestamp;
    int u;
    int v;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, r, r)
    (float, g, g)
    (float, b, b)
    (int , u, u)
    (int, v, v)
    (float, intensity, intensity)
    (float, range, range)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

struct LimoPoint {
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    float intensity;
    float range;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


POINT_CLOUD_REGISTER_POINT_STRUCT(LimoPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, r, r)
    (float, g, g)
    (float, b, b)
    (float, intensity, intensity)
    (float, range, range)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

enum CellType {
    GROUND, 
    OBSTACLE, 
    WALL, 
    UNKNOWN
};

struct Cell{
    CellType type; 
    Descriptor<MyPointType> descriptor;
    float confidence;
};


class Waggner {
    private:
    /* ------------------ Private Constructor And Destructor ------------------ */

    Waggner();
    ~Waggner();

    /* -------------------------- Private Attributes -------------------------- */

    std::vector<std::vector<std::pair<float,float>>> surface_;

    Grid<Cell> grid_; 
    std::vector<std::vector<bool>> walls_;

    float size_, cell_size_; 
    int total_points_, num_cells_;  

    float last_x_, last_y_, last_yaw_;

    int max_iter_;
    float dist_thresh_, max_slope_, max_error_, range_;
    float min_z_, max_z_;

    /* ---------------------------- Private Methods --------------------------- */
    void updateGrid(Cell cell, CellType type);

    void createWallMask(void);

    float distance2Plane(float x, float y, float z) const;
    
    void dilatation(void);

    inline bool insideGrid(int x, int y) const{
        return x >= 0 && x < num_cells_ && y >= 0 && y < num_cells_;
    }

    public:
    /* --------------------------- Singleton Pattern -------------------------- */
    static Waggner& getInstance() {
    static Waggner instance;
    return instance;
    }

    Waggner(Waggner const &) = delete;
    void operator=(Waggner const &) = delete;

    /**
     * @brief It initalizes the module.
     * 
     * @param params 
     */
    void init(const Params::Processor &params);

    /* ---------------------------- Public Methods ---------------------------- */

    void updateRemover(const nav_msgs::Odometry::ConstPtr &state);

    void updateGrid(pcl::PointCloud<MyPointType>::Ptr &cloud);
        
    void removeWallsGrid(pcl::PointCloud<MyPointType>::Ptr &cloud,
                        pcl::PointCloud<MyPointType>::Ptr &walls, 
                        pcl::PointCloud<MyPointType>::Ptr &filtered);

    void removeGroundRansac(const pcl::PointCloud<MyPointType>::Ptr &cloud,
                        pcl::PointCloud<MyPointType>::Ptr &ground,
                        pcl::PointCloud<MyPointType>::Ptr &obstacles);
    
    void removeGroundSurface(const pcl::PointCloud<MyPointType>::Ptr &cloud,
                        pcl::PointCloud<MyPointType>::Ptr &ground,
                        pcl::PointCloud<MyPointType>::Ptr &obstacles); 

    void getSurface(pcl::PointCloud<MyPointType>::Ptr &cloud);

    void getGrid(nav_msgs::OccupancyGrid &grid);
    };

#endif