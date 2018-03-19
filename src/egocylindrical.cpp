//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>
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
                                "odom");

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
        
        
        try
        {
            if(!old_pts_.empty())
            {
                EgoCylindricalPropagator::propagateHistory(old_pts_, new_pts_, old_header_, image->header);
            }
            EgoCylindricalPropagator::addDepthImage(new_pts_, image, cam_info);
            
            cv::swap(new_pts_, old_pts_);
            old_header_ = image->header;
            
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
        }
    }
    
    
    sensor_msgs::PointCloud2 EgoCylindricalPropagator::getPropagatedPointCloud()
    {
        pcl::PointCloud<pcl::PointXYZ> pcloud;
        
        
        cv::MatIterator_<cv::Point3f> it, end;
        for(it=old_pts_.begin<cv::Point3f>(), end=old_pts_.end<cv::Point3f>(); it != end; ++it)
        {
            pcl::PointXYZ point((*it).x, (*it).y, (*it).z);
            pcloud.push_back(point);
        }
        
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(pcloud, msg);
        msg.header = old_header_;
        
        return msg;
    }
    
    sensor_msgs::Image::ConstPtr EgoCylindricalPropagator::getRawRangeImage()
    {
        cv::Mat range_im = utils::getRawRangeImage(old_pts_);
        //sensor_msgs::ImageConstPtr msg = cv_bridge::CvImage(old_header_, sensor_msgs::image_encodings::TYPE_32FC1, range_im).toImageMsg();
        
        if(range_im.rows >0 && range_im.cols > 0)
        {
            cv::Mat scaled_range_im;
            cv::normalize(range_im, scaled_range_im, 0, 1, cv::NORM_MINMAX);
            cv::imshow("Raw range image", scaled_range_im);
            cv::waitKey(1);
        }
        
        sensor_msgs::Image::ConstPtr msg;
        
        return msg;
    }
    

    void EgoCylindricalPropagator::init()
    {
        
        double pi = std::acos(-1);
        hfov_ = 2*pi;
        vfov_ = pi/3;
        cylinder_width_ = 2048;
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
