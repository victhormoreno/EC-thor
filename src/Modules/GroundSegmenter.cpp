#include "Modules/GroundSegmenter.hpp"

/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */
GroundSegmenter::GroundSegmenter(){}
GroundSegmenter::~GroundSegmenter(){}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

void GroundSegmenter::init(const Params::GroundSegmenter &params){
    this->mode_ = params.mode;
    this->ransac = Ransac(params);
}

void GroundSegmenter::segment(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &ground, pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles){
    if (this->mode_){
        this->ransac.removeGround(cloud_in, ground, obstacles);
    }
    else{}
}