#include "Modules/ObjectDetector.hpp"

ObjectDetector::ObjectDetector() {}

ObjectDetector::~ObjectDetector() {}

void ObjectDetector::init(const Params::ObjectDetector &params) {
  this->mode_ = params.mode;

  // Hiperparameters
  this->eps_ = params.hiperparams.eps;
  this->minPts_ = params.hiperparams.minPts;
  this->maxPts_ = params.hiperparams.maxPts;

  // PostProcessing
  this->fsg_cone = params.postproc.fsg_cone;
  this->fsg_big_cone = params.postproc.fsg_big_cone;
  this->distr = params.postproc.distr;

  // Modules
  this->fec = FEC(params);
}


void ObjectDetector::update(const std::vector<pcl::PointXYZI> buffer){
  this->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->width = buffer.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = buffer[i].x;
    cloud->points[i].y = buffer[i].y;
    cloud->points[i].z = buffer[i].z;
  }
}

void ObjectDetector::update(const pcl::PointCloud<pcl::PointXYZI>::Ptr &buffer){
  this->cloud = buffer;
}

void ObjectDetector::generateClusters() {
  this->clusters.clear();
  this->pre_clusters.clear();

  // Clustering 
  this->fastEuclideanClustering();
}

void ObjectDetector::fastEuclideanClustering(){

  // Primera Etapa
  std::vector<pcl::PointIndices> cluster_indices;
  cluster_indices = this->fec.fastClustering(this->cloud, this->eps_, this->minPts_, 20);

  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
    Cluster cluster;
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
      cluster.points_.push_back(this->cloud->points[*pit]);
    }
    this->pre_clusters.push_back(cluster);
  }

  // Segunda Etapa
  for(Cluster c : this->pre_clusters){
    auto size = c.points_.size();
    if(size<this->maxPts_ /*&&size>10*/) {
      double max_dist = c.maxDistance();
      if((max_dist<sqrt(2)*fsg_cone.z)/*&&(max_dist>0.2)*/) this->clusters.push_back(c);  
    }
  }

}

std::vector<Cluster> ObjectDetector::postProc(std::vector<Cluster> &clusters){
  std::vector<Cluster> filter_clusters;
  std::vector<double> dist; // distribution of points in x,y,z axis
  dist.resize(3);
  for(Cluster c : clusters){
    auto size = c.points_.size();
    c.xyzDistribution(&dist);
    if((size>this->minPts_)&&(size<this->maxPts_))
      if( (dist[2]<fsg_big_cone.z) && (dist[0]<fsg_big_cone.x) && (dist[1]<fsg_big_cone.y) )
        if( (dist[2]>distr.z) && (dist[0]>distr.x) && (dist[1]>distr.y) )
          filter_clusters.push_back(c);
  }
  return filter_clusters;
}
