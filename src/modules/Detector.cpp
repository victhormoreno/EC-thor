#include "modules/Detector.hpp"

Detector::Detector() {}

Detector::~Detector() {}

void Detector::init(const Params::Detector &params) {

  // Hiperparameters
  this->eps_ = params.hiperparams.eps;
  this->minPts_ = params.hiperparams.minPts;
  this->maxPts_ = params.hiperparams.maxPts;

  // Modules
  this->fec = FEC(params);
}


void Detector::update(const std::vector<pcl::PointXYZI> buffer){
  this->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->width = buffer.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = buffer[i].x;
    cloud->points[i].y = buffer[i].y;
    cloud->points[i].z = buffer[i].z;
    cloud->points[i].intensity = buffer[i].intensity;
  }
}

void Detector::update(const pcl::PointCloud<pcl::PointXYZI>::Ptr &buffer){
  this->cloud = buffer;
}

void Detector::generateClusters(void) {
  this->clusters.clear();
  this->fastEuclideanClustering();
}

void Detector::fastEuclideanClustering(){

  std::vector<pcl::PointIndices> cluster_indices;
  this->fec.fastClustering(this->cloud, cluster_indices);

  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
    Cluster cluster;
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
      cluster.addPoint(this->cloud->points[*pit]);
    }
    //if(cluster.size() >= this->maxPts_) continue;
    this->clusters.push_back(cluster);
  }  
}

void Detector::classification(std::vector<Cluster> &clusters, std::vector<Cluster> &cones){

  // for(Cluster c : clusters){
  //   if(c.getStandardDeviation(0) <= 0.084){
  //     if(c.getStandardDeviation(1) <= 0.137) cones.push_back(c);
  //   } else if(c.getKurtosis(0) >= 4.174){
  //     if(c.getStandardDeviation(2) <= 0.15) cones.push_back(c);
  //   }
  // }

  // Decision Tree Ransac v2
  for(Cluster c : clusters){
      if(c.getStandardDeviation(0) <= 0.09){
        if(c.getStandardDeviation(1) <= 0.132) cones.push_back(c);
        else if(c.getKurtosis(1) >= 2.896) cones.push_back(c);
      } else if(c.getKurtosis(1) >= 2.772){
        if(c.getStandardDeviation(2) >= 0.044) cones.push_back(c);
      }
  }

  // Decision Tree Waggner v2
  // for(Cluster c : clusters){
  //     if(c.getStandardDeviation(0) <= 0.076){
  //       if(c.getStandardDeviation(1) <= 0.082) cones.push_back(c);
  //     } else if(c.getStandardDeviation(1) <= 0.106) cones.push_back(c);
  // }

  // Decision Tree Ransac Recontruct v1
  // for(Cluster c : clusters){
  //     if(c.getStandardDeviation(0) <= 0.124){
  //       if(c.getStandardDeviation(1) <= 0.154 && c.getStandardDeviation(2) <= 0.164) cones.push_back(c);
  //       else if (c.size() >= 560) cones.push_back(c);                                            
  //     } else if(c.getKurtosis(1) >= 3.517 && c.getKurtosis(0) >= 2.807) cones.push_back(c);
  // }

  // Decision Tree Waggner Recontruct v2
  // for(Cluster c : clusters){
  //     if(c.getStandardDeviation(0) <= 0.09){
  //       if(c.getStandardDeviation(1) <= 0.093 && c.getStandardDeviation(2) <= 0.145) cones.push_back(c);
  //       else if (c.getKurtosis(1) <= 1.843) cones.push_back(c);                                            
  //     } else if(c.getStandardDeviation(1) <= 0.107 && c.getStandardDeviation(0) <= 0.098) cones.push_back(c);
  // }


}
