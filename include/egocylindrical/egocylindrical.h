//
// Created by root on 2/5/18.
//

#ifndef EGOCYLINDRICAL_EGOCYLINDRICAL_H
#define EGOCYLINDRICAL_EGOCYLINDRICAL_H

#include <egocylindrical/utils.h>

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
//#include <pcl.h>

namespace egocylindrical
{

class EgoCylindricalPropagator{
private:
    int cylinder_height_;
    int cylinder_width_;
    double hfov_, vfov_;
        

    utils::ECWrapperPtr new_pts_, old_pts_;
    cv::Mat image_mapping_;
    image_geometry::PinholeCameraModel model_t;
    
    
    ros::NodeHandle nh_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer buffer_;
    
    utils::CylindricalCoordsConverter ccc_;

    void propagateHistory(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, std_msgs::Header new_header);
    void addDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info);
    
    
    
public:
    EgoCylindricalPropagator(ros::NodeHandle& nh);
    ~EgoCylindricalPropagator();
    
    void update(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info);
    sensor_msgs::PointCloud2  getPropagatedPointCloud();
    sensor_msgs::Image::ConstPtr getRawRangeImage();

    void init();

    //pcl::PointCloud<pcl::PointXYZI> getCylindricalPointCloud();
    //pcl::PointCloud<pcl::PointXYZ> getWorldPointCloud();


};


}


#endif //EGOCYLINDRICAL_EGOCYLINDRICAL_H
