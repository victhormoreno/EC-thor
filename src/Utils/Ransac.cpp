#include "Utils/Ransac.hpp"

Ransac::Ransac(const Params::GroundSegmenter &params){
    this->minz_ = params.ransac.minz;
    this->maxz_ = params.ransac.maxz;
    this->max_iterations_ = params.ransac.max_iterations;
    this->dist_threshold_ = params.ransac.dist_threshold;
    this->plane_angle_ = params.ransac.plane_angle;
    this->vis_outliers_ = params.ransac.vis_outliers;
}

Ransac::~Ransac(){}


void Ransac::removeGround(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &ground
                                                                        ,pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles){

    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(this->minz_, this->maxz_);
    pass.filter(*preprocCloud);

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    if (preprocCloud->size() != 0) {
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(this->max_iterations_);
        seg.setDistanceThreshold(this->dist_threshold_);
        seg.setInputCloud(preprocCloud);
        seg.setAxis(axis);
        seg.setEpsAngle(this->plane_angle_);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() != 0) {
            // Extract Ground
            pcl::ExtractIndices<pcl::PointXYZI> extractGround;
            extractGround.setInputCloud(preprocCloud);
            extractGround.setIndices(inliers);
            extractGround.setNegative(false);
            extractGround.filter(*ground);

            // Extract Obstacles
            pcl::ExtractIndices<pcl::PointXYZI> extractObstacles;
            extractObstacles.setInputCloud(preprocCloud);
            extractObstacles.setIndices(inliers);
            extractObstacles.setNegative(true);
            extractObstacles.filter(*obstacles);
        }
    }
}

