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
        
        const int matsizes[] = {3,cylinder_height_, cylinder_width_};
        
        new_pts_ = cv::Mat(3, cylinder_height_ * cylinder_width_, CV_32FC1, utils::dNaN);
        
        
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
        pcloud.points.resize(old_pts_.cols);
        pcloud.width = cylinder_width_*cylinder_height_;
        pcloud.height = 1; //cylinder_height_;
        

        float* x = old_pts_.ptr<float>(0,0);
        float* y = old_pts_.ptr<float>(1,0);
        float* z = old_pts_.ptr<float>(2,0);

        bool a;
        for(int j = 0; j < old_pts_.cols; ++j)
        {   pcl::PointXYZ point(x[j],y[j],z[j]);
            if(point.x == point.x)
            {
                    a = true;
            }
            pcloud.at(j) = point;
        }

        
        
        
        
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
        
        range_im = range_im.reshape(1, cylinder_height_);
        
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
