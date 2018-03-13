//
// Created by root on 2/5/18.
//

#include "cylindricalVisualization.h"
CylindricalVisualization::CylindricalVisualization() :it_(nh_), propagator(nh_)
{
    std::cout<<"Visualization Node Initialized"<<std::endl;
    message_filters::Subscriber<sensor_msgs::Image> depthSub(nh_, "/camera/depth/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depthInfoSub(nh_, "/camera/depth/camera_info", 10);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> timeSynchronizer(depthSub, depthInfoSub, 10);
    timeSynchronizer.registerCallback(boost::bind(&CylindricalVisualization::cameraCb, this, _1, _2));
    pub = it_.advertise("projected_image", 20);
    ptPub = nh_.advertise<sensor_msgs::PointCloud2>("cylindrical", 100);
    ros::spin();
}

void CylindricalVisualization::cameraCb(const sensor_msgs::ImageConstPtr &image,
                                        const sensor_msgs::CameraInfoConstPtr &cam_info)
{
    ROS_DEBUG("Received images and camera info");
    EgoCylindrical translated = EgoCylindrical(*image, *cam_info);

    if(!propagator.getRegisterd()) {
        propagator.registerOriginal(translated, translated.getFrame(), translated.getX(), translated.getY(), image->header.stamp);
    } else {
        propagator.propagate(translated, image->header.stamp);
    }


    ROS_INFO("%f, %f", translated.getX(), translated.getY());

    ROS_DEBUG("publish egocylindrical image");
    sensor_msgs::PointCloud2 pcloud;
    pcl::toROSMsg(translated.getCylindricalPointCloud(), pcloud);
    pcloud.header.frame_id = image->header.frame_id;
    ptPub.publish(pcloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_model");
    CylindricalVisualization s;
    ros::spin();
}