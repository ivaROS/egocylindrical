//
// Created by root on 2/5/18.
//

#include "cylindricalVisualization.h"
CylindricalVisualization::CylindricalVisualization() :it_(nh_)
{
    std::cout<<"Visualization Node Initialized"<<std::endl;
    message_filters::Subscriber<sensor_msgs::Image> depthSub(nh_, "/camera/depth/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depthInfoSub(nh_, "/camera/depth/camera_info", 10);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> timeSynchronizer(depthSub, depthInfoSub, 30);
    timeSynchronizer.registerCallback(boost::bind(&CylindricalVisualization::cameraCb, this, _1, _2));
    pub = it_.advertise("projected_image", 20);
    ros::spin();
}

void CylindricalVisualization::cameraCb(const sensor_msgs::ImageConstPtr &image,
                                        const sensor_msgs::CameraInfoConstPtr &cam_info)
{
    EgoCylindrical translated = EgoCylindrical(*image, *cam_info);

    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time(0);
    msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, translated.toImage()).toImageMsg();
    pub.publish(msg);

}