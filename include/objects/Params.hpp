/**
 * @file Params.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Params class.
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <ros/ros.h>
#include <string>
#include <vector>

struct Params {

  /* -------------------------- Public Constructor -------------------------- */

  Params(const ros::NodeHandle &nh){

  /* ------------------------------------------------------------------------ */
  /*                                  COMMON                                  */
  /* ------------------------------------------------------------------------ */

  nh.param<std::string>("/ecthor/common/topics/input/point_cloud", common.topics.input.limovelo, "/limovelo/full_pcl");
  nh.param<std::string>("/ecthor/common/topics/input/odom", common.topics.input.odom, "/limovelo/state");
  nh.param<std::string>("/ecthor/common/topics/output/observations", common.topics.output.observations, "/ecthor/observations");
  nh.param<std::string>("/ecthor/common/topics/output/state", common.topics.output.state, "/ecthor/state");
  nh.param<std::string>("/ecthor/common/topics/output/walls", common.topics.output.walls, "/ecthor/walls");
  nh.param<std::string>("/ecthor/common/topics/output/ground", common.topics.output.ground, "/ecthor/ground");
  nh.param<std::string>("/ecthor/common/topics/output/obstacles", common.topics.output.obstacle, "/ecthor/obstacle");
  nh.param<std::string>("/ecthor/common/topics/output/buffer", common.topics.output.buffer, "/ecthor/buffer");
  nh.param<std::string>("/ecthor/common/topics/output/clusters", common.topics.output.clusters, "/ecthor/clusters");
  nh.param<std::string>("/ecthor/common/topics/output/centroides", common.topics.output.centroides, "/ecthor/centroides");



  /* ------------------------------------------------------------------------ */
  /*                                  MANAGER                                 */
  /* ------------------------------------------------------------------------ */
  nh.param<bool>("/ecthor/manager/publish_debug", manager.publish_debug, false);
  nh.param<bool>("/ecthor/manager/reconstruct", manager.reconstruct, true);
  

  /* ------------------------------------------------------------------------ */
  /*                                  PROCESSOR                               */
  /* ------------------------------------------------------------------------ */
  
  nh.param<bool>("/ecthor/processor/remove_walls", processor.remove_walls, true);
  nh.param<int>("/ecthor/processor/init_iterations", processor.init_iterations, 10);
  nh.param<float>("/ecthor/processor/waggner/grid_size", processor.waggner.grid_size, 0.5);
  nh.param<float>("/ecthor/processor/waggner/cell_size", processor.waggner.cell_size, 0.1);
  nh.param<float>("/ecthor/processor/waggner/min_height", processor.waggner.min_height, 1.0);
  nh.param<int>("/ecthor/processor/waggner/max_iterations", processor.waggner.max_iterations, 100);
  nh.param<float>("/ecthor/processor/waggner/distance_threshold", processor.waggner.distance_threshold, 0.2);
  nh.param<float>("/ecthor/processor/waggner/max_slope", processor.waggner.max_slope, 0.1);
  nh.param<float>("/ecthor/processor/waggner/max_error", processor.waggner.max_error, 0.1);
  nh.param<float>("/ecthor/processor/waggner/max_z", processor.waggner.max_z, 0.5);
  nh.param<float>("/ecthor/processor/waggner/min_z", processor.waggner.min_z, -0.5);
  nh.param<float>("/ecthor/processor/waggner/max_range", processor.waggner.max_range, 100.0);
  
    
  /* ------------------------------------------------------------------------ */
  /*                                  ACCUMULATOR                             */
  /* ------------------------------------------------------------------------ */

  nh.param<int>("/ecthor/accumulator/buffer_size", accumulator.buffer_size, 10000);
  nh.param<bool>("/ecthor/accumulator/filter", accumulator.filter, true);
  nh.param<float>("/ecthor/accumulator/field_of_view", accumulator.field_of_view, 180);
  nh.param<float>("/ecthor/accumulator/radius", accumulator.radius, 10);
  
  /* ------------------------------------------------------------------------ */
  /*                                  DETECTOR                                */
  /* ------------------------------------------------------------------------ */

  nh.param<int>("/ecthor/detect/hiperparams/maxPts", detect.hiperparams.maxPts, 100);
  nh.param<float>("/ecthor/detect/hiperparams/eps", detect.hiperparams.eps, 40);
  nh.param<int>("/ecthor/detect/hiperparams/minPts", detect.hiperparams.minPts, 5);

  /* ------------------------------------------------------------------------ */
  /*                                RECONSTRUCTOR                             */
  /* ------------------------------------------------------------------------ */

  nh.param<float>("/ecthor/reconstr/delete_param", reconstr.delete_param, 0.3);
  nh.param<float>("/ecthor/reconstr/balance_param", reconstr.balance_param, 0.6);
  nh.param<float>("/ecthor/reconstr/box_length", reconstr.box_length, 0.2);
  nh.param<int>("/ecthor/reconstr/k_nearest", reconstr.k_nearest, 10);
  nh.param<double>("/ecthor/reconstr/max_dist", reconstr.max_dist, 0.5);
  nh.param<float>("/ecthor/reconstr/radius", reconstr.radius, 0.5);
  nh.param<bool>("/ecthor/reconstr/downsample", reconstr.downsample, true);
  nh.param<bool>("/ecthor/reconstr/mode", reconstr.mode, true);

  }
  
  /* ------------------------------------------------------------------------ */
  /*                                  COMMON                                  */
  /* ------------------------------------------------------------------------ */
  
  struct Common {
    struct {
      struct {
        std::string limovelo;
        std::string odom;
      } input;
      struct {
        std::string observations;
        std::string state;
        std::string walls;
        std::string ground;
        std::string obstacle;
        std::string buffer;
        std::string clusters;
        std::string centroides;
      } output;
    } topics;
  } common;

  /* ------------------------------------------------------------------------ */
  /*                                  MANAGER                                 */
  /* ------------------------------------------------------------------------ */

  struct Manager {
    bool publish_debug;
    bool reconstruct;
  } manager;

  /* ------------------------------------------------------------------------ */
  /*                                 PROCESSOR                                */
  /* ------------------------------------------------------------------------ */

  struct Processor {
    bool remove_walls;
    int init_iterations;
    struct {
      float grid_size, cell_size;  // Grid Parameters
      float min_height; // Min height for wall
      int max_iterations; 
      float distance_threshold, max_slope, max_error; 
      float max_z, min_z, max_range;
    } waggner;
  } processor;


  /* ------------------------------------------------------------------------ */
  /*                                  ACCUMULATOR                             */
  /* ------------------------------------------------------------------------ */

  struct Accumulator {
    int buffer_size;
    bool filter;
    float field_of_view, radius;
  } accumulator;

  /* ------------------------------------------------------------------------ */
  /*                                  DETECTOR                                */
  /* ------------------------------------------------------------------------ */

  struct Detector {
    struct {
      float eps; 
      int maxPts, minPts;
    } hiperparams;
  } detect;

    /* ------------------------------------------------------------------------ */
    /*                               RECONSTRUCTOR                              */
    /* ------------------------------------------------------------------------ */

  struct Reconstructor {
    float delete_param, balance_param, box_length;
    int k_nearest; // K-nearest 
    double max_dist; // K-nearest 
    float radius; // Radius search
    bool downsample, mode;
  } reconstr;
};

#endif 
