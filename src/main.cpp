/**
 * @file main.cpp
 * @author Víctor Moreno Borràs (victor.moreno.borras@estudiantat.upc.edu)
 * @brief Main file of EC-thor
 * @version 2.0
 * @date 2024-04-20
 * 
 * @copyright Copyright (c) 2024 BCN eMotorsport
 * 
 */

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float32.h>

#include "modules/Manager.hpp"


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;


int main(int argc, char **argv){

    ros::init(argc, argv, "ecthor");
    ros::NodeHandle *const nh= new ros::NodeHandle;

    /* Fill configurations Params with YAML */
    Params params(*nh);

    /* EC-thor Publishers */

    // Observations Publisher
    ros::Publisher pubObs = nh->advertise<as_msgs::ObservationArray>(params.common.topics.output.observations, 20);

    // Debug Publisher
    ros::Publisher pubObstacle = nh->advertise<sensor_msgs::PointCloud2>(params.common.topics.output.obstacle, 20);
    ros::Publisher pubState = nh->advertise<nav_msgs::Odometry>(params.common.topics.output.state, 20);
    ros::Publisher pubWalls = nh->advertise<sensor_msgs::PointCloud2>(params.common.topics.output.walls, 20);
    ros::Publisher pubGround = nh->advertise<sensor_msgs::PointCloud2>(params.common.topics.output.ground, 20);
    ros::Publisher pubBuffer = nh->advertise<sensor_msgs::PointCloud2>(params.common.topics.output.buffer, 20);
    ros::Publisher pubClusters = nh->advertise<sensor_msgs::PointCloud2>(params.common.topics.output.clusters, 1);
    ros::Publisher pubTime = nh->advertise<std_msgs::Float32>("/AS/P/ecthor/debug/time", 1);

    /* EC-thor Subscribers */
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub;
    message_filters::Subscriber<nav_msgs::Odometry> state_sub;

    /* Initialize Manager */
    Manager& manager = Manager::getInstance();
    manager.init(params, pubObs, pubState, pubWalls, pubGround, pubObstacle, pubBuffer, pubClusters);

    /* Time syncronizer */
    pcl_sub.subscribe(*nh, params.common.topics.input.limovelo, 1000);
    state_sub.subscribe(*nh,params.common.topics.input.odom, 4);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pcl_sub, state_sub);
    sync.registerCallback(boost::bind(&Manager::callbackNewData, &manager, _1, _2));

    ros::Rate rate(10);
    std_msgs::Float32 time;

    /* -------------  Main loop ------------- */ 
    while (ros::ok()) {
        
        auto beginMain = std::chrono::high_resolution_clock::now();

        // Program execution
        manager.run();
                
        auto endMain = std::chrono::high_resolution_clock::now();
        time.data = std::chrono::duration_cast<std::chrono::milliseconds>(endMain - beginMain).count();
        pubTime.publish(time);
        std::cout << "MAIN LOOP ROSRATE :  " << time.data << "ms" << std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
