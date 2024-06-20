#include "modules/Accumulator.hpp"

/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */

Accumulator::Accumulator() {}
Accumulator::~Accumulator() {
  delete buffer_;
}

void Accumulator::add(Points ps){
    for (Point p : ps){
        this->add(p);
    }
}

void Accumulator::add(Point p){
    this->buffer_->add(p);
}

void Accumulator::buffer2PointCloud(pcl::PointCloud<Point>::Ptr &cloud){
    
    std::vector<Point> bufferVec = this->buffer_->toVector();  
    cloud->width = bufferVec.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width);

    //omp_set_num_threads(4);
    
    // #pragma omp parallel for
    for (size_t j = 0; j < bufferVec.size(); ++j)    {
        cloud->points[j].x = bufferVec[j].x;
        cloud->points[j].y = bufferVec[j].y;
        cloud->points[j].z = bufferVec[j].z;
        cloud->points[j].intensity = bufferVec[j].intensity;
    }

}

void Accumulator::radialFiltering(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<Point> extract;

    omp_set_num_threads(4);
    #pragma omp parallel
    {
        pcl::PointIndices::Ptr inliers_thread(new pcl::PointIndices());

        #pragma omp for nowait
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            if(this->includedInSemicercle(cloud->points[i])){
                inliers_thread->indices.push_back(static_cast<int>(i));
            }      
        }

        #pragma omp critical
        {
            inliers->indices.insert(inliers->indices.end(), inliers_thread->indices.begin(), inliers_thread->indices.end());
        }
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
}


bool Accumulator::includedInSemicercle(Point p){
    
    double ptx = p.x - this->lastx_;
    double pty = p.y - this->lasty_;
    double x  = ptx * cos(-this->last_yaw_) - pty * sin(-this->last_yaw_);
    double y = ptx * sin(-this->last_yaw_) + pty * cos(-this->last_yaw_);

    double angle = atan2(y, x);

    if(-field_of_view_/2 < angle && angle < field_of_view_/2){
        if(std::hypot(x,y) < this->radius_) return true;
        else return false;
    } 
    else return false;
}


/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

void Accumulator::init(const Params::Accumulator &params){
    
    this->size_ = params.buffer_size;
    
    this->buffer_ = new Buffer<pcl::PointXYZI>(this->size_);

    this->filter_ = params.filter;
    this->radius_ = params.radius;
    this->field_of_view_ = params.field_of_view*M_PI/180.0;

    this->lastx_ = 0.0;
    this->lasty_ = 0.0;
    this->lastz_ = 0.0;
    this->last_yaw_ = 0.0;
}

void Accumulator::receiveObstacles(pcl::PointCloud<MyPointType>::Ptr &obstacles){
    Points ps;
    for (const auto& p : obstacles->points){
        Point p_;
        p_.x = p.x;
        p_.y = p.y;
        p_.z = p.z;
        p_.intensity = p.intensity;
        ps.push_back(p_);
    }
    this->add(ps);
}

void Accumulator::receiveState(const nav_msgs::Odometry::ConstPtr &state){
    this->lastx_ = state->pose.pose.position.x;
    this->lasty_ = state->pose.pose.position.y;
    this->lastz_ = state->pose.pose.position.z;

    auto qw = state->pose.pose.orientation.w;
    auto qx = state->pose.pose.orientation.x;
    auto qy = state->pose.pose.orientation.y;
    auto qz = state->pose.pose.orientation.z;

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    this->last_yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

Points Accumulator::getPoints(){
    return this->buffer_->toVector();
}

void Accumulator::getCloud(pcl::PointCloud<Point>::Ptr &cloud){
    this->buffer2PointCloud(cloud);
    this->radialFiltering(cloud);
}