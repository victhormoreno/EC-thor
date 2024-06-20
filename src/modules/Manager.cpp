#include "modules/Manager.hpp"

/* -------------------------------------------------------------------------- */
/*                      PRIVATE CONSTRUCTOR & DESTRUCTOR                      */
/* -------------------------------------------------------------------------- */

Manager::Manager() {}
Manager::~Manager() {}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

void Manager::init(const Params &params,
                   const ros::Publisher &pubObs,
                   const ros::Publisher &pubState,
                   const ros::Publisher &pubWalls,
                   const ros::Publisher &pubGround,
                   const ros::Publisher &pubObstacle,
                   const ros::Publisher &pubBuffer,  
                   const ros::Publisher &pubClusters) {

  this->pubObs_ = pubObs;
  this->pubState_ = pubState;
  this->pubWalls_ = pubWalls;
  this->pubGround_ = pubGround;
  this->pubObstacle_ = pubObstacle;
  this->pubBuffer_ = pubBuffer;
  this->pubClusters_ = pubClusters;
  this->params_ = params.manager;

  /* Initialize modules */
  this->proc = &Processor::getInstance();
  this->proc->init(params.processor);

  this->accum = &Accumulator::getInstance();
  this->accum->init(params.accumulator);

  this->detect = &Detector::getInstance();
  this->detect->init(params.detect);

  this->reconstr = &Reconstructor::getInstance();
  this->reconstr->init(params.reconstr);
}


void Manager::run(){

  // Check if there are points in the buffer and get them 
  if(!this->accum->buffHasValidData()) return;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr buff(new pcl::PointCloud<pcl::PointXYZI>);
  accum->getCloud(buff);

  // Update Detector with new points and generate clusters
  detect->update(buff);
  detect->generateClusters();
  std::vector<Cluster> clusters = detect->getClusters();

  // Reconstrucut the clusters
  if(this->params_.reconstruct) reconstr->reconstruct(clusters);

  // Classify the clusters
  std::vector<Cluster> cones;
  detect->classification(clusters, cones);

  // Publish Observations and State
  this->publishObservations(cones);
  this->pubState_.publish(this->last_state_);
  
  // Publish Debug Topics
  if(this->params_.publish_debug){
    this->publishBuff(buff); 
    this->publishClusters(cones);
  }
}


/* -------------------------------- Callback ------------------------------- */
void Manager::callbackNewData(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, const nav_msgs::Odometry::ConstPtr &state){
  
  this->header_ = cloud_msg->header; 
  this->last_state_ = *state;

  accum->receiveState(state);
  if(this->params_.reconstruct) reconstr->updateIkdTree(*cloud_msg); // Update the Incremental KD-Tree with new points

  pcl::PointCloud<LimoPoint>::Ptr cloud(new pcl::PointCloud<LimoPoint>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  pcl::PointCloud<MyPointType>::Ptr walls(new pcl::PointCloud<MyPointType>);
  pcl::PointCloud<MyPointType>::Ptr ground(new pcl::PointCloud<MyPointType>);
  pcl::PointCloud<MyPointType>::Ptr obstacles(new pcl::PointCloud<MyPointType>);

  // Segment input Point Cloud into walls, ground and obstacles points
  proc->process(cloud, walls, ground, obstacles);

  // Publish walls, ground and obstacles points, if required
  if(this->params_.publish_debug){
    this->publishWalls(walls);
    this->publishGround(ground);
    this->publishObstacles(obstacles);
  } 
  
  // Accumulate obstacles points into the buffer
  accum->receiveObstacles(obstacles);
}

/* -------------------------------- Publishers ------------------------------- */
void Manager::publishWalls(const pcl::PointCloud<MyPointType>::ConstPtr &cloud) const {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud,msg);
  msg.header.frame_id = "global";
  msg.header.stamp = ros::Time::now();
  this->pubWalls_.publish(msg);
}

void Manager::publishGround(const pcl::PointCloud<MyPointType>::ConstPtr &cloud) const {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud,msg);
  msg.header.frame_id = "global";
  msg.header.stamp = ros::Time::now();
  this->pubGround_.publish(msg);
}

void Manager::publishObstacles(const pcl::PointCloud<MyPointType>::ConstPtr &cloud) const {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud,msg);
  msg.header.frame_id = "global";
  msg.header.stamp = ros::Time::now();
  this->pubObstacle_.publish(msg);
}

void Manager::publishObservations(std::vector<Cluster> &clusters) const{
  as_msgs::ObservationArray obs_vector;  
  this->createObservations(clusters,obs_vector);
  obs_vector.header.stamp = this->header_.stamp;
  obs_vector.header.frame_id = "global";
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Asigna un color único a cada clúster
  for (size_t i = 0; i < clusters.size(); ++i) {
    for (auto it = clusters[i].begin(); it != clusters[i].end(); ++it) {
      pcl::PointXYZI p = *it;
      pcl::PointXYZRGB coloredPoint;

      coloredPoint.x = p.x;
      coloredPoint.y = p.y;
      coloredPoint.z = p.z;

      // Asigna un color único basado en el índice del clúster
      coloredPoint.r = static_cast<uint8_t>((i * 119 + 103) % 256);
      coloredPoint.g = static_cast<uint8_t>((i * 61 + 139) % 256);
      coloredPoint.b = static_cast<uint8_t>((i * 31 + 173) % 256);

      coloredCloud->push_back(coloredPoint);
    }
  }

  pcl::toROSMsg(*coloredCloud, msg_clust);
  msg_clust.header = this->header_;
  pubClusters_.publish(msg_clust);
}


/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */

void Manager::createObservations(std::vector<Cluster> &clusters, as_msgs::ObservationArray &obs_vector) const{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (Cluster c : clusters){
      as_msgs::Observation obs;
      for(auto it = c.begin(); it != c.end(); ++it) cloud->push_back(*it);
      pcl::toROSMsg(*cloud,obs.cloud);
      obs.centroid.x = c.getCentroid(0);
      obs.centroid.y = c.getCentroid(1);
      obs.centroid.z = c.getCentroid(2);
      obs_vector.observations.push_back(obs);
      cloud->clear();
  }
}


void Manager::saveCSV(const std::vector<Cluster> &clusters) const{
  int clusterId = 0;  // Inicializa el ID del cluster en 0
  std::ofstream file("/home/victhor/bcnemotorsport/code/data/clusters_reconstruction.csv");
  file <<  "id,x,y,z,i"  << "\n";
  for(Cluster c : clusters){
    for(auto it = c.begin(); it != c.end(); ++it){
      pcl::PointXYZI p = *it;
      file << clusterId << ","  << p.x << "," << p.y << "," << p.z << "," << p.intensity << "\n";
    }
    ++clusterId; 
  }
  file.close();
}