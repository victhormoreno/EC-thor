#include "Modules/Manager.hpp"

/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */

Manager::Manager() {}
Manager::~Manager() {}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

void Manager::init(const Params &params,
                   const ros::Publisher &pubObs,
                   const ros::Publisher &pubGround,
                   const ros::Publisher &pubObstacle,
                   const ros::Publisher &pubBuffer,  
                   const ros::Publisher &pubClusters) {
  this->pubObs_ = pubObs;
  this->pubGround_ = pubGround;
  this->pubObstacle_ = pubObstacle;
  this->pubBuffer_ = pubBuffer;
  this->pubClusters_ = pubClusters;
  this->params_ = params.manager;

  /* Initialize modules */
  this->grdseg = &GroundSegmenter::getInstance();
  this->grdseg->init(params.ground_segmenter);

  this->accum = &Accumulator::getInstance();
  this->accum->init(params.accumulator);

  this->objdet = &ObjectDetector::getInstance();
  this->objdet->init(params.objdet);

  this->reconstr = &Reconstructor::getInstance();
  this->reconstr->init(params.reconstr);
}


void Manager::run(){
  // Get points from buffer and check if there are points
  if(!this->accum->buffHasValidData()) return;
  pcl::PointCloud<pcl::PointXYZI>::Ptr buff = accum->getCloud();

  // Update ObjectDetector with new points and generate clusters
  objdet->update(buff);
  objdet->generateClusters();
  std::vector<Cluster> clusters = objdet->postProc(objdet->getClusters());

  // Reconstrucut the clusters
  if(this->params_.reconstruct){
    std::vector<pcl::PointXYZI> centroides;
    Cluster::computeCentroids(clusters, centroides);
    clusters.clear();
    reconstr->reconstruct(centroides, clusters);
  }
 
  // Publish Observations
  this->publishObservations(clusters);
  
  // Publish Debug Topics
  if(this->params_.publish_debug){
    if(this->params_.how_publish_buffer) this->publishBuff(this->objdet->getCloudPoints());
    else this->publishBuff(this->accum->getRawCloud());
    this->publishClusters(clusters);
  }
}


/* -------------------------------- Callback ------------------------------- */
void Manager::callbackNewData(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, const nav_msgs::Odometry::ConstPtr &state){
  
  this->header_ = state->header;

  accum->receiveState(state);
  reconstr->updateIkdTree(*cloud_msg); // Update the Incremental KD-Tree with new points

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZI>);

  // Segment input Point Cloud into ground and obstacles points
  grdseg->segment(cloud, ground, obstacles);

  // Publish ground and obstacles points, if required
  if(this->params_.publish_debug){
    this->publishGround(ground);
    this->publishObstacles(obstacles);
  } 

  // Accumulate obstacles points into the buffer
  accum->receiveObstacles(obstacles);
}


void Manager::createObservations(std::vector<Cluster> &clusters, as_msgs::ObservationArray &obs_vector) const{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (Cluster c : clusters){
      as_msgs::Observation obs;
      pcl::PointXYZI centroid = c.getCentroid();
      obs.centroid.x = centroid.x;
      obs.centroid.y = centroid.y;
      obs.centroid.z = centroid.z;
      for(pcl::PointXYZI p : c.points_) cloud->push_back(p);
      pcl::toROSMsg(*cloud,obs.cloud);
      obs_vector.observations.push_back(obs);
      cloud->clear();
  }
}

/* -------------------------------- Publishers ------------------------------- */
void Manager::publishGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "global";
  msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*cloud,msg);
  this->pubGround_.publish(msg);
}

void Manager::publishObstacles(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "global";
  msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*cloud,msg);
  this->pubObstacle_.publish(msg);
}

void Manager::publishObservations(std::vector<Cluster> &clusters) const{
  as_msgs::ObservationArray obs_vector;  
  this->createObservations(clusters,obs_vector);
  obs_vector.header = this->header_;
  this->pubObs_.publish(obs_vector);
}

void Manager::publishBuff(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
  sensor_msgs::PointCloud2 msg_buff;
  pcl::toROSMsg(*cloud,msg_buff);
  msg_buff.header.frame_id = "global"; 
  msg_buff.header.stamp = ros::Time::now();
  this->pubBuffer_.publish(msg_buff);
}

void::Manager::publishClusters(const std::vector<Cluster> &clusters) const {
  sensor_msgs::PointCloud2 msg_clust;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for(Cluster c : clusters){
    for(pcl::PointXYZI p : c.points_){
      pcl::PointXYZ point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      cloud->push_back(point);
    }
  }

  // // Asigna un color único a cada clúster
  // for (size_t i = 0; i < clusters.size(); ++i) {
  //   for (pcl::PointXYZI p : clusters[i].points_) {
  //     pcl::PointXYZRGB coloredPoint;
  //     coloredPoint.x = p.x;
  //     coloredPoint.y = p.y;
  //     coloredPoint.z = p.z;

  //     // Asigna un color único basado en el índice del clúster
  //     coloredPoint.r = static_cast<uint8_t>((i * 119 + 103) % 256);
  //     coloredPoint.g = static_cast<uint8_t>((i * 61 + 139) % 256);
  //     coloredPoint.b = static_cast<uint8_t>((i * 31 + 173) % 256);

  //     coloredCloud->push_back(coloredPoint);
  //   }
  // }
  pcl::toROSMsg(*cloud, msg_clust);
  msg_clust.header = this->header_;
  pubClusters_.publish(msg_clust);
}

void Manager::saveCSV(const std::vector<Cluster> &clusters) const{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  int clusterId = 0;  // Inicializa el ID del cluster en 0
  std::ofstream file("/home/victhor/bcnemotorsport/ws/src/ec-thor/data/clusters_reconstruction.csv");
  file <<  "id,x,y,z,i"  << "\n";
  for(Cluster c : clusters){
    for(pcl::PointXYZI p : c.points_){
      cloud->push_back(p);
      file << clusterId << ","  << p.x << "," << p.y << "," << p.z << "," << p.intensity << "\n";
    }
    ++clusterId; 
  }
  file.close();
}