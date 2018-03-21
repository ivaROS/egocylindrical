//
// Created by root on 2/5/18.
//

#include <egocylindrical/cylindricalVisualization.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"

CylindricalVisualization::CylindricalVisualization() :it_(nh_), propagator_(nh_)
{
    std::cout<<"Visualization Node Initialized"<<std::endl;
    message_filters::Subscriber<sensor_msgs::Image> depthSub(nh_, "/camera/depth/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depthInfoSub(nh_, "/camera/depth/camera_info", 10);
    
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_listener_(buffer_);
    
    tf2_ros::MessageFilter<sensor_msgs::CameraInfo> tf_filter(depthInfoSub, buffer_, "odom", 10,nh_);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> timeSynchronizer(depthSub, depthInfoSub, 10);
    timeSynchronizer.registerCallback(boost::bind(&CylindricalVisualization::cameraCb, this, _1, _2));
    pub = it_.advertise("projected_image", 20);
    ptPub = nh_.advertise<sensor_msgs::PointCloud2>("cylindrical", 100);
    ptPub2 = nh_.advertise<sensor_msgs::PointCloud2>("cylindrical_original", 100);
    ros::spin();
}

void CylindricalVisualization::cameraCb(const sensor_msgs::ImageConstPtr &image,
                                        const sensor_msgs::CameraInfoConstPtr &cam_info)
{
    ROS_DEBUG("Received images and camera info");

    propagator_.update(image, cam_info);

    ROS_DEBUG("publish egocylindrical image");
    
    ptPub.publish(propagator_.getPropagatedPointCloud());
    
    pub.publish(propagator_.getRawRangeImage());
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_model");
    CylindricalVisualization s;
    ros::spin();
}
