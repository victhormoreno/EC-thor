/**
 * @file Params.hpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief It contains the specification of the Params class.
 * @version 2.0
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2023 BCN eMotorsport
 * 
 */

#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <ros/ros.h>

#include <string>
#include <vector>

struct Params {

  /* -------------------------- Public Constructor -------------------------- */

  Params(const ros::NodeHandle &nh);
  
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
    bool how_publish_buffer;
    bool reconstruct;
  } manager;

  /* ------------------------------------------------------------------------ */
  /*                             GROUND SEGMENTER                             */
  /* ------------------------------------------------------------------------ */

  struct GroundSegmenter {
    bool mode;
    struct{
      float minz, maxz;
      int max_iterations;
      double dist_threshold, plane_angle;
      bool vis_outliers;
    } ransac;
    struct {

    } linefit;
  } ground_segmenter;


  /* ------------------------------------------------------------------------ */
  /*                                  ACCUMULATOR                             */
  /* ------------------------------------------------------------------------ */

  struct Accumulator {
    int buffer_size, buffer_raw_size;
    struct {
      struct {
        bool active;
        float res_x, res_y, res_z;
      } voxelgrid;
      struct {
        bool active;
        float box_dim;
        float max_height;
      } gridfilter;
      struct {
        bool active, filter_x, filter_y, filter_z;
        float limit_x, limit_y,limit_z;
      } passthrough;
    } preproc;
  } accumulator;

  /* ------------------------------------------------------------------------ */
  /*                                  OBJECT DETECTOR                                  */
  /* ------------------------------------------------------------------------ */

  struct ObjectDetector {
    bool mode;
    struct {
      float eps; 
      int maxPts, minPts;
    } hiperparams;
    struct PostProc {
      struct FSGCone{
        float x,y,z;
      } fsg_cone;
      struct FSGBigCone{
        float x,y,z;
      } fsg_big_cone;
      struct Distr{
        float x,y,z;
      } distr;
    } postproc;
  } objdet;

    /* ------------------------------------------------------------------------ */
    /*                               RECONSTRUCTOR                              */
    /* ------------------------------------------------------------------------ */

  struct Reconstructor {
    float delete_param, balance_param, box_length;
  } reconstr;
};
#endif 
