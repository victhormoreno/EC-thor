/**
 * @file Manager.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Manager module.
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#ifndef MANAGER_HPP
#define MANAGER_HPP

#include <as_msgs/ObservationArray.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

#include "modules/Processor.hpp"
#include "modules/Accumulator.hpp"
#include "modules/Detector.hpp"
#include "modules/Reconstructor.hpp"

/**
 * @brief The Manager module is what coordinates all different Modules.
 * It is responsible for:
 * - Dynamic reconfigure objects
 * - Pipeline calls
 * - Data receiving
 * - Data publishing
 */

class Manager {
 private:
  /* ------------------ Private Constructor And Destructor ------------------ */

  Manager();
  ~Manager();

  /* -------------------------------- Modules ------------------------------- */
  Processor *proc;          /**< Processor module */
  Accumulator *accum;       /**< Accumulator module */
  Detector *detect;         /**< Detector module */
  Reconstructor *reconstr;  /**< Reconstructor module */

  /* -------------------------- Private Attributes -------------------------- */

  ros::Publisher pubObs_, pubState_;
  ros::Publisher pubWalls_, pubGround_, pubObstacle_;
  ros::Publisher pubBuffer_, pubClusters_;

  std_msgs::Header header_;
  nav_msgs::Odometry last_state_;

  Params::Manager params_;

  /* ---------------------------- Private Methods --------------------------- */
  

  /**
   * @brief Testing methode for saving Clusters on a csv file.
   * 
   * @param clusters 
   */
  void saveCSV(const std::vector<Cluster> &clusters) const;

  /**
   * @brief Creates the ObservationArray message from the clusters.
   * 
   * @param clusters 
   * @param obs
   */
  void createObservations(std::vector<Cluster> &clusters, as_msgs::ObservationArray &obs_vector) const;

  /* ---------------------------- Publish Methods --------------------------- */
  void publishWalls(const pcl::PointCloud<MyPointType>::ConstPtr &walls) const;
  void publishGround(const pcl::PointCloud<MyPointType>::ConstPtr &ground) const;
  void publishObstacles(const pcl::PointCloud<MyPointType>::ConstPtr &obstacles) const;
  void publishBuff(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const;
  void publishClusters(const std::vector<Cluster> &clusters) const;
  void publishObservations(std::vector<Cluster> &clusters) const;

 public:
  /* --------------------------- Singleton Pattern -------------------------- */
  static Manager& getInstance() {
    static Manager instance;
    return instance;
  }

  Manager(Manager const &) = delete;
  void operator=(Manager const &) = delete;

  /**
   * @brief It initalizes the module.
   * 
   * @param params 
   * @param pubObs 
   * @param pubGround
   * @param pubObstacle
   * @param pubBuffer
   * @param pubTester 
   */
  void init(const Params &params,
            const ros::Publisher &pubObs,
            const ros::Publisher &pubState,
            const ros::Publisher &pubWalls,
            const ros::Publisher &pubGround,
            const ros::Publisher &pubObstacle,
            const ros::Publisher &pubBuffer,
            const ros::Publisher &pubClusters);

  /* ---------------------------- Public Methods ---------------------------- */

  /**
   * @brief Recive the limovelo data and filters the point cloud.
   * 
   * @param pointcloud
   * @param state
   */
  void callbackNewData(const sensor_msgs::PointCloud2::ConstPtr &pointcloud, const nav_msgs::Odometry::ConstPtr &state);
  
  /**
   * @brief Runs the pipeline with last synchronized data.
   */
  void run();

  void getGrid(nav_msgs::OccupancyGrid &grid){
    proc->getGrid(grid);
  }
};

#endif 
