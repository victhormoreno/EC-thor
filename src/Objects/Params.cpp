#include "Objects/Params.hpp"

Params::Params(const ros::NodeHandle &nh) {

  /* ------------------------------------------------------------------------ */
  /*                                  COMMON                                  */
  /* ------------------------------------------------------------------------ */

  nh.param<std::string>("/ecthor/common/topics/input/point_cloud", common.topics.input.limovelo, "/limovelo/full_pcl");
  nh.param<std::string>("/ecthor/common/topics/input/odom", common.topics.input.odom, "/limovelo/state");
  nh.param<std::string>("/ecthor/common/topics/output/observations", common.topics.output.observations, "/ecthor/observations");
  nh.param<std::string>("/ecthor/common/topics/output/ground", common.topics.output.ground, "/ecthor/ground");
  nh.param<std::string>("/ecthor/common/topics/output/obstacles", common.topics.output.obstacle, "/ecthor/obstacle");
  nh.param<std::string>("/ecthor/common/topics/output/buffer", common.topics.output.buffer, "/ecthor/buffer");
  nh.param<std::string>("/ecthor/common/topics/output/clusters", common.topics.output.clusters, "/ecthor/clusters");
  nh.param<std::string>("/ecthor/common/topics/output/centroides", common.topics.output.centroides, "/ecthor/centroides");



  /* ------------------------------------------------------------------------ */
  /*                                  MANAGER                                 */
  /* ------------------------------------------------------------------------ */
  nh.param<bool>("/ecthor/manager/publish_debug", manager.publish_debug, false);
  nh.param<bool>("/ecthor/manager/how_publish_buffer", manager.how_publish_buffer, true);
  nh.param<bool>("/ecthor/manager/reconstruct", manager.reconstruct, true);
  

  /* ------------------------------------------------------------------------ */
  /*                            GROUND SEGMENTER                              */
  /* ------------------------------------------------------------------------ */
  
  nh.param<bool>("/ecthor/groundsegmenter/mode", ground_segmenter.mode, true);

  nh.param<float>("/ecthor/groundsegmenter/ransac/minz", ground_segmenter.ransac.minz, -2.0f);
  nh.param<float>("/ecthor/groundsegmenter/ransac/maxz", ground_segmenter.ransac.maxz, 1.5f);
  nh.param<int>("/ecthor/groundsegmenter/ransac/max_iterations", ground_segmenter.ransac.max_iterations, 100);
  nh.param<double>("/ecthor/groundsegmenter/ransac/dist_threshold", ground_segmenter.ransac.dist_threshold, 0.2);
  nh.param<double>("/ecthor/groundsegmenter/ransac/plane_angle", ground_segmenter.ransac.plane_angle, 0.05);
  nh.param<bool>("/ecthor/groundsegmenter/ransac/vis_outliers", ground_segmenter.ransac.vis_outliers, false);

  
  
  /* ------------------------------------------------------------------------ */
  /*                                  ACCUMULATOR                             */
  /* ------------------------------------------------------------------------ */

  nh.param<int>("/ecthor/accumulator/buffer_size", accumulator.buffer_size, 10000);
  nh.param<int>("/ecthor/accumulator/buffer_raw_size", accumulator.buffer_raw_size, 100000);

  nh.param<bool>("/ecthor/accumulator/preproc/voxelgrid/active", accumulator.preproc.voxelgrid.active, true);
  nh.param<float>("/ecthor/accumulator/preproc/voxelgrid/res_x", accumulator.preproc.voxelgrid.res_x, 0.1);
  nh.param<float>("/ecthor/accumulator/preproc/voxelgrid/res_y", accumulator.preproc.voxelgrid.res_y, 0.1);
  nh.param<float>("/ecthor/accumulator/preproc/voxelgrid/res_z", accumulator.preproc.voxelgrid.res_z, 0.1);

  nh.param<bool>("/ecthor/accumulator/preproc/gridfilter/active", accumulator.preproc.gridfilter.active, true);
  nh.param<float>("/ecthor/accumulator/preproc/gridfilter/box_dim", accumulator.preproc.gridfilter.box_dim, 0.2);
  nh.param<float>("/ecthor/accumulator/preproc/gridfilter/max_height", accumulator.preproc.gridfilter.max_height, 0.2);

  nh.param<bool>("/ecthor/accumulator/preproc/passthrough/active", accumulator.preproc.passthrough.active, false);
  nh.param<bool>("/ecthor/accumulator/preproc/passthrough/filter_x", accumulator.preproc.passthrough.filter_x, false);
  nh.param<bool>("/ecthor/accumulator/preproc/passthrough/filter_y", accumulator.preproc.passthrough.filter_y, false);
  nh.param<bool>("/ecthor/accumulator/preproc/passthrough/filter_z", accumulator.preproc.passthrough.filter_z, false);
  nh.param<float>("/ecthor/accumulator/preproc/passthrough/limit_x", accumulator.preproc.passthrough.limit_x, 50);
  nh.param<float>("/ecthor/accumulator/preproc/passthrough/limit_y", accumulator.preproc.passthrough.limit_y, 50);
  nh.param<float>("/ecthor/accumulator/preproc/passthrough/limit_z", accumulator.preproc.passthrough.limit_z, 2);

  /* ------------------------------------------------------------------------ */
  /*                                  ObjectDetecthor                                  */
  /* ------------------------------------------------------------------------ */
  nh.param<bool>("/ecthor/objdet/mode", objdet.mode, true);

  nh.param<int>("/ecthor/objdet/hiperparams/maxPts", objdet.hiperparams.maxPts, 100);
  nh.param<float>("/ecthor/objdet/hiperparams/eps", objdet.hiperparams.eps, 40);
  nh.param<int>("/ecthor/objdet/hiperparams/minPts", objdet.hiperparams.minPts, 5);

  nh.param<float>("/ecthor/objdet/postproc/cone/x", objdet.postproc.fsg_cone.x, 0.228);
  nh.param<float>("/ecthor/objdet/postproc/cone/y", objdet.postproc.fsg_cone.y, 0.228);
  nh.param<float>("/ecthor/objdet/postproc/cone/z", objdet.postproc.fsg_cone.z, 0.325);

  nh.param<float>("/ecthor/objdet/postproc/bigCone/x", objdet.postproc.fsg_big_cone.x, 0.285);
  nh.param<float>("/ecthor/objdet/postproc/bigCone/y", objdet.postproc.fsg_big_cone.y, 0.285);
  nh.param<float>("/ecthor/objdet/postproc/bigCone/z", objdet.postproc.fsg_big_cone.z, 0.505);

  nh.param<float>("/ecthor/objdet/postproc/min_distribution/x", objdet.postproc.distr.x, 0.05);
  nh.param<float>("/ecthor/objdet/postproc/min_distribution/y", objdet.postproc.distr.y, 0.05);
  nh.param<float>("/ecthor/objdet/postproc/min_distribution/z", objdet.postproc.distr.z, 0.05);

  /* ------------------------------------------------------------------------ */
  /*                                RECONSTRUCTOR                             */
  /* ------------------------------------------------------------------------ */

  nh.param<float>("/ecthor/reconstr/delete_param", reconstr.delete_param, 0.3);
  nh.param<float>("/ecthor/reconstr/balance_param", reconstr.balance_param, 0.6);
  nh.param<float>("/ecthor/reconstr/box_length", reconstr.box_length, 0.2);

};