#include "Objects/Cluster.hpp"

Cluster::Cluster() { 
  this->id_ = 0;
}

Cluster::~Cluster(){}


void Cluster::calculateCentroid() {
  if (this->points_.size() > 0) {
    for (auto &point : this->points_) {
      this->centroid_.x += point.x;
      this->centroid_.y += point.y;
      this->centroid_.z += point.z;
    }
    this->centroid_.x = this->centroid_.x /= this->points_.size();
    this->centroid_.y = this->centroid_.y /= this->points_.size();
    this->centroid_.z = this->centroid_.z /= this->points_.size();
  }
}

double Cluster::maxDistance(){
  double max_distance = 0.0;
  for (size_t i = 0; i < this->points_.size(); ++i){
    for (size_t j = i + 1; j < this->points_.size(); ++j)    {
      double distance = pcl::euclideanDistance(this->points_[i], this->points_[j]);
      if (distance > max_distance) max_distance = distance;
    }
  }
  return max_distance;
}

double Cluster::zDistribution(){
  double max_ = -100.0;
  double min_ = 100.0;
  for (size_t i = 0; i < this->points_.size(); ++i){
    if (this->points_[i].z > max_) max_ = this->points_[i].z;
    if (this->points_[i].z < min_) min_ = this->points_[i].z;
  }
  return fabs(max_ - min_);
}

double Cluster::xDistribution(){
  double max_ = -9999.9;
  double min_ = 9999.9;
  for (size_t i = 0; i < this->points_.size(); ++i){
    if (this->points_[i].x > max_) max_ = this->points_[i].x;
    if (this->points_[i].x < min_) min_ = this->points_[i].x;
  }
  return fabs(max_ - min_);
}

double Cluster::yDistribution(){
  double max_ = -9999.9;
  double min_ = 9999.9;
  for (size_t i = 0; i < this->points_.size(); ++i){
    if (this->points_[i].y > max_) max_ = this->points_[i].y;
    if (this->points_[i].y < min_) min_ = this->points_[i].y;
  }
  return fabs(max_ - min_);
}

void Cluster::xyzDistribution(std::vector<double> *xyz /* must be of size >= 3*/){ 
  if(xyz->size() < 3) return;
  double maxX, maxY, maxZ, minX, minY, minZ;
  maxX = maxY = maxZ = -9999.9;
  minX = minY = minZ = 9999.9;
  for (size_t i = 0; i < this->points_.size(); ++i){
    if (this->points_[i].x > maxX) maxX = this->points_[i].x;
    if (this->points_[i].x < minX) minX = this->points_[i].x;

    if (this->points_[i].y > maxY) maxY = this->points_[i].y;
    if (this->points_[i].y < minY) minY = this->points_[i].y;

    if (this->points_[i].z > maxZ) maxZ = this->points_[i].z;
    if (this->points_[i].z < minZ) minZ = this->points_[i].z;
  }
  xyz->at(0) = fabs(maxX - minX);
  xyz->at(1) = fabs(maxY - minY);
  xyz->at(2) = fabs(maxZ - minZ);
}
