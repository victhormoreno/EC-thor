#include "Utils/Preproc.hpp"

Preproc::Preproc(const Params::Accumulator &params){
    // Voxelgrid atributes
    this->vxg_active_ = params.preproc.voxelgrid.active;
    this->res_x = params.preproc.voxelgrid.res_x;
    this->res_y = params.preproc.voxelgrid.res_y;
    this->res_z = params.preproc.voxelgrid.res_z;
    
    // Passthrough atributes
    this->passthrough_active_ = params.preproc.passthrough.active;
    this->x_active_ = params.preproc.passthrough.filter_x;
    this->y_active_ = params.preproc.passthrough.filter_y;
    this->z_active_ = params.preproc.passthrough.filter_z;
    this->limx_ = params.preproc.passthrough.limit_x;
    this->limy_ = params.preproc.passthrough.limit_y;
    this->limz_ = params.preproc.passthrough.limit_z;
    
    // Grid Filter attributes
    this->gridfilter_active = params.preproc.gridfilter.active;
    this->box_dim = params.preproc.gridfilter.box_dim;
    this->max_height = params.preproc.gridfilter.max_height;

    // Init last state
    this->lastx_ = 0.0;
    this->lasty_ = 0.0;
    this->lastz_ = 0.0;
}

Preproc::~Preproc(){}

void Preproc::preprocess(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){
    if(this->passthrough_active_) passThrough(cloud);
    if(this->vxg_active_) downSampling(cloud);
    if(this->gridfilter_active) gridFilter(cloud);
}

void Preproc::downSampling(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){
    this->vxg.setInputCloud(cloud);
    this->vxg.setLeafSize(this->res_x, this->res_y, this->res_z); 
    this->vxg.filter(*cloud);
}

void Preproc::gridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){

    // Set projection to X-Y plane
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::copyPointCloud(*cloud, input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>(input));
    for(auto& pt : *cloud_projected)
        pt.z = 0.0f;

    // Create octree
    pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ> octree(this->box_dim);
    octree.setInputCloud(cloud_projected);
    octree.addPointsFromInputCloud();

    // Store indices for each leaf
    std::vector<std::vector<int>> idx_vec;
    idx_vec.reserve(octree.getLeafCount());

    const auto it_end = octree.leaf_depth_end();
    for(auto it = octree.leaf_depth_begin(); it != it_end; ++it){
        auto leaf = it.getLeafContainer();
        std::vector<int> idx;
        leaf.getPointIndices(idx);
        idx_vec.push_back(idx);
    }

    // Remove high height distribution points
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    for(const auto indices : idx_vec){
        pcl::PointCloud<pcl::PointXYZ> leaf(input, indices); // pcl for each leaf
        double height = zDistribution(&leaf);
        if( height > this->max_height ){
            for(auto index : indices){ inliers->indices.push_back(index); }
        }
    }

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
}

void Preproc::passThrough(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){

    if(this->x_active_){
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(this->lastx_-this->limx_,this->lastx_+this->limx_);
        pass.filter(*cloud);
    }

    if(this->y_active_){
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(this->lasty_-this->limy_,this->lasty_+this->limy_);
        pass.filter(*cloud);
    }  

    if(this->z_active_){
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(this->lastz_-this->limz_,this->lastz_+this->limz_);
        pass.filter(*cloud);
    }

}

void Preproc::updateState(const nav_msgs::Odometry::ConstPtr &state){
    this->lastx_ = state->pose.pose.position.x;
    this->lasty_ = state->pose.pose.position.y;
    this->lastz_ = state->pose.pose.position.z;
}

double Preproc::zDistribution(pcl::PointCloud<pcl::PointXYZ>* leaf){
  double max_ = -100.0;
  double min_ = 100.0;
  for(auto& pt : *leaf){
    if (pt.z > max_) max_ = pt.z;
    if (pt.z < min_) min_ = pt.z;
  }
  return fabs(max_ - min_);
}
