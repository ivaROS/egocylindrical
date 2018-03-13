//
// Created by root on 2/5/18.
//

#include "egocylindrical.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>


namespace egocylindrical
{



    void EgoCylindricalPropagator::propagateHistory(cv::Mat& old_pnts, cv::Mat& new_pnts, std_msgs::Header old_header, std_msgs::Header new_header)
    {
        ROS_DEBUG("Getting Transformation details");
                geometry_msgs::TransformStamped trans = buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                old_header.frame_id, old_header.stamp,
                                "odom", ros::Duration(1));

        utils::transformPoints(old_pnts, trans);
        utils::fillImage(new_pnts, old_pnts, ccc_, false);
    }




    void EgoCylindricalPropagator::addDepthImage(cv::Mat& cylindrical_points, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        model_t.fromCameraInfo(cam_info);
        
        cv::Mat new_pnts = utils::depthImageToWorld(image,model_t);
        
        utils::fillImage(cylindrical_points, new_pnts, ccc_, true);
        
    }


    void EgoCylindricalPropagator::update(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        new_pts_ = cv::Mat(cylinder_height_,cylinder_width_, CV_32FC3, utils::dNaN);
        
        EgoCylindricalPropagator::propagateHistory(old_pts_, new_pts_, old_header_, image->header);
        EgoCylindricalPropagator::addDepthImage(new_pts_, image, cam_info);
        
        cv::swap(new_pts_, old_pts_);
        old_header_ = image->header;
    }

    void EgoCylindricalPropagator::init()
    {
        
        double pi = std::acos(-1);
        hfov_ = 2*pi;
        vfov_ = pi/3;
        cylinder_width_ = 1024;
        cylinder_height_ = 320;
        
        ccc_ = utils::CylindricalCoordsConverter(cylinder_width_, cylinder_height_, hfov_, vfov_);
    }

    EgoCylindricalPropagator::EgoCylindricalPropagator(ros::NodeHandle& nh):
        nh_(nh),
        tf_listener_(buffer_)
    {
        init();
        
        
    }


}
