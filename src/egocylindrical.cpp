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

//#include <valgrind/callgrind.h>

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
        
        utils::remapDepthImage(image, ccc_, model_t, cylindrical_points);
        
    }


    void EgoCylindricalPropagator::update(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        ros::WallTime start = ros::WallTime::now();
        
        new_pts_ = cv::Mat(cylinder_height_,cylinder_width_, CV_32FC3, utils::dNaN);
        
        try
        {
            if(!old_pts_.empty())
            {
                EgoCylindricalPropagator::propagateHistory(old_pts_, new_pts_, old_header_, image->header);
                ROS_INFO_STREAM("Propagation took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
            }
   
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
        }
        
        
        
        start = ros::WallTime::now();
        
        EgoCylindricalPropagator::addDepthImage(new_pts_, image, cam_info);
        ROS_INFO_STREAM("Adding depth image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        cv::swap(new_pts_, old_pts_);
        old_header_ = image->header;
        
        
    }
    
    
    sensor_msgs::PointCloud2 EgoCylindricalPropagator::getPropagatedPointCloud()
    {
        ros::WallTime start = ros::WallTime::now();
      
        pcl::PointCloud<pcl::PointXYZ> pcloud;
        pcloud.points.resize(old_pts_.cols*old_pts_.rows);
        pcloud.width = cylinder_width_;
        pcloud.height = cylinder_height_;
        

        old_pts_.forEach<cv::Point3f>
        (
          [&pcloud](cv::Point3f &point, const int* position) -> void
          {
            int i = position[0];
            int j = position[1];
            
            pcloud.at(j,i) = pcl::PointXYZ(point.x, point.y, point.z);          
          }
        );
        
        
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(pcloud, msg);
        msg.header = old_header_;
        
        ROS_INFO_STREAM("Generating point cloud took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
                
        return msg;
    }
    
    sensor_msgs::Image::ConstPtr EgoCylindricalPropagator::getRawRangeImage()
    {
        ros::WallTime start = ros::WallTime::now();
      
        cv::Mat range_im = utils::getRawRangeImage(old_pts_);
        
        if(range_im.rows >0 && range_im.cols > 0)
        {
            cv::Mat scaled_range_im;
            cv::normalize(range_im, scaled_range_im, 0, 1, cv::NORM_MINMAX);
            cv::imshow("Raw range image", scaled_range_im);
            cv::waitKey(1);
        }
         
        sensor_msgs::Image::ConstPtr msg = cv_bridge::CvImage(old_header_, sensor_msgs::image_encodings::TYPE_32FC1, range_im).toImageMsg();
        
        ROS_INFO_STREAM("Generating egocylindrical image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        
        return msg;
    }
    

    void EgoCylindricalPropagator::init()
    {
        
        double pi = std::acos(-1);
        hfov_ = 2*pi;
        vfov_ = pi/2;        
        
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
    
    EgoCylindricalPropagator::~EgoCylindricalPropagator()
    {
      
    }
      


}
