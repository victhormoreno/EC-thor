#include "Modules/Accumulator.hpp"

/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */

Accumulator::Accumulator() {}
Accumulator::~Accumulator() {
  delete buffer_;
  delete raw_buffer_;
}

void Accumulator::add(Points ps){
    for (Point p : ps){
        this->add(p);
    }
}

void Accumulator::add(Point p){
    this->buffer_->add(p);
}

void Accumulator::buffer2PointCloud(bool raw){
    
    if(raw){
        std::vector<Point> raw_buff = this->raw_buffer_->toVector();    
        this->raw_cloud_->width = raw_buff.size();
        this->raw_cloud_->height = 1;
        this->raw_cloud_->points.resize(this->raw_cloud_->width * this->raw_cloud_->height);

        for (size_t j = 0; j < raw_buff.size(); ++j)    {
            this->raw_cloud_->points[j].x = raw_buff[j].x;
            this->raw_cloud_->points[j].y = raw_buff[j].y;
            this->raw_cloud_->points[j].z = raw_buff[j].z;
        }

    }else{
        std::vector<Point> bufferVec = this->buffer_->toVector();  
        this->cloud_->width = bufferVec.size();
        this->cloud_->height = 1;
        this->cloud_->points.resize(this->cloud_->width * this->cloud_->height);

        for (size_t j = 0; j < bufferVec.size(); ++j)    {
            this->cloud_->points[j].x = bufferVec[j].x;
            this->cloud_->points[j].y = bufferVec[j].y;
            this->cloud_->points[j].z = bufferVec[j].z;
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

void Accumulator::init(const Params::Accumulator &params){
    this->size_ = params.buffer_size;
    this->raw_size_ = params.buffer_raw_size;
    
    this->buffer_ = new Buffer<pcl::PointXYZI>(this->size_);
    this->cloud_ = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    this->raw_buffer_ = new Buffer<pcl::PointXYZI>(this->raw_size_);
    this->raw_cloud_ = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);

    this->preproc = Preproc(params);
}

void Accumulator::receiveLidar(const sensor_msgs::PointCloud2& cloud_msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(cloud_msg, *cloud);
    
    for (const auto& p : cloud->points)
        this->raw_buffer_->add(p);
}

void Accumulator::receiveObstacles(pcl::PointCloud<pcl::PointXYZI>::Ptr &obstacles){
    Points ps;
    for (const auto& p : obstacles->points)
        ps.push_back(p);
    this->add(ps);
}

void Accumulator::receiveState(const nav_msgs::Odometry::ConstPtr &state){
    this->preproc.updateState(state);
}

Points Accumulator::getPoints(){
    return this->buffer_->toVector();
}

pcl::PointCloud<Point>::Ptr Accumulator::getCloud(){
    this->buffer2PointCloud(false);
    this->preproc.preprocess(this->cloud_);
    return this->cloud_;
}

pcl::PointCloud<Point>::Ptr Accumulator::getRawCloud(){
    this->buffer2PointCloud(true);
    return this->raw_cloud_;
}