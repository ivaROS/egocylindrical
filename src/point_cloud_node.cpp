//
// Created by root on 2/5/18.
//

#include <egocylindrical/point_cloud_generator.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_pointcloud_publisher");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::EgoCylinderPointCloudGenerator s(nh, pnh);
    s.init();
    ros::spin();
}
