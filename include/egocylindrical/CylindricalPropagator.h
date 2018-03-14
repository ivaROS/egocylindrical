//
// Created by root on 3/6/18.
//

#ifndef EGOCYLINDRICAL_CYLINDRICALPROPAGATOR_H
#define EGOCYLINDRICAL_CYLINDRICALPROPAGATOR_H

//#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include "egocylindrical.h"
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class CylindricalPropagator
{
private:
    pcl::PointCloud<pcl::PointXYZ> worldPointCloud;
    pcl::PointCloud<pcl::PointXYZI> cylindricalPointCloud;
    bool registered = false;
    ros::Time last;
    ros::NodeHandle nh_;
    tf2_ros::TransformListener listener;
    tf2_ros::Buffer buffer;
    std::string frame_id;
    double x, y;
public:
    CylindricalPropagator();
    explicit CylindricalPropagator(ros::NodeHandle &nh);
    pcl::PointCloud<pcl::PointXYZ> getWorldPointCloud(){return worldPointCloud;};
    pcl::PointCloud<pcl::PointXYZI> getCylindricalPointCloud(){return cylindricalPointCloud;};
    void registerOriginal(EgoCylindrical t, std::string frame_, double x_, double y_, ros::Time time);
    void propagate(EgoCylindrical t, ros::Time now);
    void propagate(pcl::PointCloud<pcl::PointXYZ> pointCloud, ros::Time now);
    bool getRegisterd(){return registered;}
};


#endif //EGOCYLINDRICAL_CYLINDRICALPROPAGATOR_H
