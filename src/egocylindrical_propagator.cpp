//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>
#include <egocylindrical/point_transformer.h>
#include <egocylindrical/depth_image_core.h>

//#include <tf/LinearMath/Matrix3x3.h>
//#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
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
        utils::transformPoints(old_pnts, *transformed_pts_, new_pnts, trans);
        ROS_INFO_STREAM_NAMED("timing", "Transform points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        start = ros::WallTime::now();
        utils::addPoints(new_pnts, *transformed_pts_, false);
        ROS_INFO_STREAM_NAMED("timing", "Inserting transformed points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

    }


    void EgoCylindricalPropagator::addDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        depth_remapper_.update(cylindrical_points, image, cam_info);
        
    }


    void EgoCylindricalPropagator::update(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        ros::WallTime start = ros::WallTime::now();
        
        //new_pts_ = utils::getECWrapper(cylinder_height_,cylinder_width_,vfov_);
        // NOTE: It may be better to only create the necessary wrappers once and just 'swap' the msg_ pointers
        new_pts_ = next_pts_;
        bool allocate_next = !old_pts_ || old_pts_->isLocked();
        
//        #pragma omp parallel sections num_threads(2) if(allocate_next)
        {
//          #pragma omp section
          {
            try
            {
                if(old_pts_)
                {
                    ros::WallTime start = ros::WallTime::now();
                    
                    EgoCylindricalPropagator::propagateHistory(*old_pts_, *new_pts_, image->header);
                    //ROS_INFO_STREAM("Propagation took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                }
                else
                {
                    new_pts_->setHeader(image->header);
                }
                
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
            }
            
            
            if(use_depth_ < 2)
            {
                ros::WallTime temp = ros::WallTime::now();
                
                EgoCylindricalPropagator::addDepthImage(*new_pts_, image, cam_info);
                ROS_INFO_STREAM("Adding depth image took " <<  (ros::WallTime::now() - temp).toSec() * 1e3 << "ms");
            }
            
            if(ec_pub_.getNumSubscribers() > 0)
            {
                // TODO: if no one is subscribing, we can propagate the points in place next time (if that turns out to be faster)
                ros::WallTime temp = ros::WallTime::now();
                
                //TODO: this function call should trigger the ecwrapper to mark its message as locked
                utils::ECMsgConstPtr msg = new_pts_->getEgoCylinderPointsMsg();
                ROS_INFO_STREAM("Copying EgoCylinderPoints took " <<  (ros::WallTime::now() - temp).toSec() * 1e3 << "ms");
                
                ec_pub_.publish(msg);
            }
          }
          
 //         #pragma omp section
          {
            ros::WallTime start = ros::WallTime::now();
            
            //Lock mutex
            ReadLock lock(config_mutex_);
            
           // if(allocate_next)
            //{
                next_pts_ = utils::getECWrapper(config_.height, config_.width,config_.vfov);
                /*
            }
            else
            {
                std::swap(next_pts_,old_pts_);

                next_pts_->init(config_.height, config_.width, config_.vfov, true);   //This is probably not necessary every time                
            }
            */
            ROS_INFO_STREAM("Creating new datastructure took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          }
          
        
        }
        
        std::swap(new_pts_, old_pts_);  
        
        ROS_INFO_STREAM("Total time: " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        
    }
    
    void EgoCylindricalPropagator::connectCB()
    {
        // If no one is listening, we can propagate points in place
        if (ec_pub_.getNumSubscribers() == 0)
        {
            
        }
    }
    
    void EgoCylindricalPropagator::configCB(const egocylindrical::PropagatorConfig &config, uint32_t level)
    {
        use_depth_ +=1;
        WriteLock lock(config_mutex_);
     
        ROS_INFO_STREAM("Updating propagator config: height=" << config.height << ", width=" << config.width << ", vfov=" << config.vfov);
        config_ = config;
    }
    
    
    // TODO: add dynamic reconfigure for cylinder height/width, vfov, etc
    bool EgoCylindricalPropagator::init()
    {
        reconfigure_server_->setCallback(boost::bind(&EgoCylindricalPropagator::configCB, this, _1, _2));
        
        double pi = std::acos(-1);
        hfov_ = 2*pi;
        vfov_ = pi/2;        
        
        cylinder_width_ = 2048;
        cylinder_height_ = 320;
        
        transformed_pts_ = utils::getECWrapper(config_.height, config_.width,config_.vfov,true);
        
        next_pts_ = utils::getECWrapper(config_.height, config_.width,config_.vfov);
                
        
        // Get topic names
        std::string depth_topic="/camera/depth/image_raw", info_topic= "/camera/depth/camera_info", pub_topic="egocylindrical_points";
        
        pnh_.getParam("image_in", depth_topic );
        pnh_.getParam("info_in", info_topic );
        pnh_.getParam("points_out", pub_topic );
        
        // Setup publishers
        ros::SubscriberStatusCallback image_cb = boost::bind(&EgoCylindricalPropagator::connectCB, this);        
        ec_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>(pub_topic, 1, image_cb, image_cb);
        
        
        // Setup subscribers
        depthSub.subscribe(it_, depth_topic, 3);
        depthInfoSub.subscribe(nh_, info_topic, 3);
        
        // Ensure that CameraInfo is transformable
        info_tf_filter = boost::make_shared<tf_filter>(depthInfoSub, buffer_, "odom", 2,nh_);
        
        // Synchronize Image and CameraInfo callbacks
        timeSynchronizer = boost::make_shared<synchronizer>(depthSub, *info_tf_filter, 2);
        timeSynchronizer->registerCallback(boost::bind(&EgoCylindricalPropagator::update, this, _1, _2));
        
        return true;
    }

    EgoCylindricalPropagator::EgoCylindricalPropagator(ros::NodeHandle& nh, ros::NodeHandle& pnh):
        nh_(nh),
        pnh_(pnh),
        tf_listener_(buffer_),
        it_(nh)
    {
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        
        
    }
    
    EgoCylindricalPropagator::~EgoCylindricalPropagator()
    {
      
    }
      


}
