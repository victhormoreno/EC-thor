#include "modules/Processor.hpp"

/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */
Processor::Processor(){}
Processor::~Processor(){}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

void Processor::init(const Params::Processor &params){
    this->waggner = &Waggner::getInstance();
    this->waggner->init(params);

    this->remove_walls_ = params.remove_walls;
    this->init_iteration_ = params.init_iterations;
}

void Processor::process(pcl::PointCloud<LimoPoint>::Ptr &cloud_in, 
                        pcl::PointCloud<MyPointType>::Ptr &walls,
                        pcl::PointCloud<MyPointType>::Ptr &ground, 
                        pcl::PointCloud<MyPointType>::Ptr &obstacles){
    
    static int iteration = 0;

    pcl::PointCloud<MyPointType>::Ptr cloud(new pcl::PointCloud<MyPointType>);
    pcl::copyPointCloud(*cloud_in, *cloud);

    waggner->updateGrid(cloud);

    if(iteration < this->init_iteration_){
        iteration++;
        return;
    }

    pcl::PointCloud<MyPointType>::Ptr filtered(new pcl::PointCloud<MyPointType>);
    if(this->remove_walls_) {
        waggner->removeWallsGrid(cloud, walls, filtered);
        //waggner->removeGroundRansac(filtered, ground, obstacles);
        waggner->removeGroundSurface(filtered, ground, obstacles);
    } else {
        //waggner->removeGroundRansac(cloud, ground, obstacles);
        waggner->removeGroundSurface(cloud, ground, obstacles);
    }
    //waggner->removeGroundSurface(filtered, ground_, obstacles_); // Under Developing
    
}

void Processor::getGrid(nav_msgs::OccupancyGrid &grid){
    waggner->getGrid(grid);
}