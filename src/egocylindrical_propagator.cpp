//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>
//#include <tf/LinearMath/Matrix3x3.h>
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



    void EgoCylindricalPropagator::propagateHistory(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, std_msgs::Header new_header)
    {
        ros::WallTime start = ros::WallTime::now();
        
        std_msgs::Header old_header = old_pnts.getHeader();
        
        new_pnts.setHeader(new_header);
        
        ROS_DEBUG("Getting Transformation details");
                geometry_msgs::TransformStamped trans = buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                old_header.frame_id, old_header.stamp,
                                "odom");
        
        ROS_INFO_STREAM_NAMED("timing", "Finding transform took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                
        
        start = ros::WallTime::now();        
        utils::transformPoints(old_pnts, trans);
        ROS_INFO_STREAM_NAMED("timing", "Transform points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        start = ros::WallTime::now();
        utils::addPoints(new_pnts, old_pnts, ccc_, false);
        ROS_INFO_STREAM_NAMED("timing", "Inserting transformed points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

    }


    void EgoCylindricalPropagator::addDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        model_t.fromCameraInfo(cam_info);
        
        utils::addDepthImage(cylindrical_points, image, ccc_, model_t);
        
    }


    void EgoCylindricalPropagator::update(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        ros::WallTime start = ros::WallTime::now();

        new_pts_ = utils::getECWrapper(cylinder_height_,cylinder_width_,vfov_);

        try
        {
            if(old_pts_)
            {
                EgoCylindricalPropagator::propagateHistory(*old_pts_, *new_pts_, image->header);
                ROS_INFO_STREAM("Propagation took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
            }
   
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
        }
        
        
        
        start = ros::WallTime::now();
        
        EgoCylindricalPropagator::addDepthImage(*new_pts_, image, cam_info);
        ROS_INFO_STREAM("Adding depth image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        
        if(ec_pub_.getNumSubscribers() > 0)
        {
            egocylindrical::EgoCylinderPoints::ConstPtr msg = new_pts_->getEgoCylinderPointsMsg();
            ec_pub_.publish(msg);
        }
        
        
        std::swap(new_pts_, old_pts_);        
        
    }
    
    /*
    
    sensor_msgs::PointCloud2 EgoCylindricalPropagator::getPropagatedPointCloud()
    {
        ros::WallTime start = ros::WallTime::now();
      
        sensor_msgs::PointCloud2 msg = utils::generate_point_cloud(*old_pts_);
                
        ROS_INFO_STREAM("Generating point cloud took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
                
        return msg;
    }
    
    */
    
    /*
    
    sensor_msgs::Image::ConstPtr EgoCylindricalPropagator::getRawRangeImage()
    {
        ros::WallTime start = ros::WallTime::now();
              
        sensor_msgs::Image::Ptr image_ptr = utils::getRawRangeImageMsg(*old_pts_);

        ROS_INFO_STREAM("Generating egocylindrical image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        return image_ptr;
    }
    
    */
    

    void EgoCylindricalPropagator::init()
    {
        
        double pi = std::acos(-1);
        hfov_ = 2*pi;
        vfov_ = pi/2;        
        
        cylinder_width_ = 2048;
        cylinder_height_ = 320;
        
        ccc_ = utils::CylindricalCoordsConverter(cylinder_width_, cylinder_height_, hfov_, vfov_);
        
        ec_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>("egocylindrical", 5);
        
        
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
