//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>
#include <egocylindrical/point_transformer.h>
#include <egocylindrical/depth_image_inserter.h>

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
    
    utils::ECParams getParams(const egocylindrical::PropagatorConfig &config)
    {
      utils::ECParams params;
      params.height = config.height;
      params.width = config.width;
      params.vfov = config.vfov;
      params.can_width = config.can_width;
      params.v_offset = config.v_offset;
      params.cyl_radius = config.cyl_radius;
      return params;
    }
    
    namespace utils
    {
      utils::ECWrapperPtr getECWrapper(const egocylindrical::PropagatorConfig &config, bool allocate_arrays=false)
      {
        return utils::getECWrapper(getParams(config), allocate_arrays);
      }
    }

    void EgoCylindricalPropagator::propagateHistory(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, std_msgs::Header new_header)
    {
        ros::WallTime start = ros::WallTime::now();
        
        std_msgs::Header old_header = old_pnts.getHeader();
        
        new_pnts.setHeader(new_header);
        
        ROS_DEBUG("Getting Transformation details");
                geometry_msgs::TransformStamped trans = buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                old_header.frame_id, old_header.stamp,
                                fixed_frame_id_);
        
        ROS_DEBUG_STREAM_NAMED("timing", "Finding transform took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                
        
        start = ros::WallTime::now();    
        utils::transformPoints(old_pnts, *transformed_pts_, new_pnts, trans, config_.num_threads);
        ROS_DEBUG_STREAM_NAMED("timing", "Transforming points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        start = ros::WallTime::now();
        utils::addPoints(new_pnts, *transformed_pts_, false);
        ROS_DEBUG_STREAM_NAMED("timing", "Inserting transformed points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

    }


    void EgoCylindricalPropagator::addDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        dii_.insert(cylindrical_points, image, cam_info);
        
//         if(pc_pub_.getNumSubscribers()>0)
//         {
//           ReadLock lock(config_mutex_);
//           sensor_msgs::PointCloud2::Ptr pcloud_msg;
//           depth_remapper_.update(cylindrical_points, image, cam_info, pcloud_msg, config_.filter_y_min, config_.filter_y_max);
//           pc_pub_.publish(pcloud_msg);
//         }
//         else
//         {
//             depth_remapper_.update(cylindrical_points, image, cam_info);
//         }
    }


    void EgoCylindricalPropagator::update(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        //TODO: Reset between runs so that the reset function returns once reset is complete
        {
            WriteLock lock(reset_mutex_);
            if(should_reset_)
            {
                old_pts_ = nullptr;
                should_reset_ = false;
            }
        }
        if(old_pts_)
        {
            if(old_pts_->getHeader().stamp > cam_info->header.stamp)
            {
                old_pts_ = nullptr;
            }
            else if(old_pts_->getHeader().stamp == cam_info->header.stamp)
            {
              ROS_WARN_STREAM_NAMED("msg_timestamps","Repeat stamps received! " << cam_info->header.stamp);
              return;
            }
        }

        //TODO: Warn of out-of-order images
        //TODO: If clock jumps back in time, reset egocylinder
        
        ROS_DEBUG_STREAM_NAMED("msg_timestamps","Current stamp: " << cam_info->header.stamp);
        ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[egocylinder] Received [" << cam_info->header.stamp << "] at [" << ros::WallTime::now() << "]");
        
        if(old_pts_)
        {
            ROS_DEBUG_STREAM_NAMED("msg_timestamps","Previous stamp " << old_pts_->getHeader().stamp);
        }
        ros::WallTime start = ros::WallTime::now();
        
        // NOTE: It may be better to only create the necessary wrappers once and just 'swap' the msg_ pointers
        new_pts_ = next_pts_;
        bool allocate_next = !old_pts_ || old_pts_->isLocked();
        
        if(!cfh_.updateTransforms(image->header))
        {
            ROS_WARN_STREAM("Failed to update transforms!");
            return;
        }
        
        std_msgs::Header target_header = cfh_.getTargetHeader();
        
        #pragma omp parallel sections num_threads(2) if(allocate_next)
        {
          #pragma omp section
          {
            try
            {
                if(old_pts_)
                {
                    ros::WallTime start = ros::WallTime::now();
                    
                    EgoCylindricalPropagator::propagateHistory(*old_pts_, *new_pts_, target_header);
                    //ROS_INFO_STREAM("Propagation took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                }
                else
                {
                    new_pts_->setHeader(target_header);
                }
                
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
            }
            //TODO: Decide how to handle transform failure
            
            {
                ros::WallTime temp = ros::WallTime::now();
                EgoCylindricalPropagator::addDepthImage(*new_pts_, image, cam_info);
                ROS_DEBUG_STREAM_NAMED("timing","Adding depth image took " <<  (ros::WallTime::now() - temp).toSec() * 1e3 << "ms");
            }
            
            if(ec_pub_.getNumSubscribers() > 0 && shouldPublish(new_pts_))
            {
                // TODO: if no one is subscribing, we can propagate the points in place next time (if that turns out to be faster)
                utils::ECMsgConstPtr msg = new_pts_->getEgoCylinderPointsMsg();
                
                ec_pub_.publish(msg);
                ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[egocylinder] Sent [" << msg->header.stamp << "] at [" << ros::WallTime::now() << "]");
                published(new_pts_);
            }
            if(info_pub_.getNumSubscribers() > 0)
            {
              info_pub_.publish(new_pts_->getEgoCylinderInfoMsg());
            }
          }
          
          #pragma omp section
          {
            ros::WallTime start = ros::WallTime::now();
            
            //Lock mutex
            ReadLock lock(config_mutex_);
            
            if(allocate_next)
            {
                ROS_DEBUG_STREAM("Create new ECWrapper for next time");
                next_pts_ = utils::getECWrapper(config_);
                
            }
            else
            {
                ROS_DEBUG_STREAM("Reuse old ECWrapper for next time");
                std::swap(next_pts_,old_pts_);

                next_pts_->init(getParams(config_), true);
            }
            
            if((!next_pts_) || next_pts_->getNumPts()==0 || (!next_pts_->msg_) || next_pts_->msg_->points.data.size()==0)
            {
              int tempa = 0;  //Just a place to put a breakpoint
            }
            
            ROS_DEBUG_STREAM_NAMED("timing", "Creating new datastructure took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          }
          
        
        }
        
        std::swap(new_pts_, old_pts_);  
        
        ROS_DEBUG_STREAM_NAMED("timing", "Total time: " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        
    }
    
    void EgoCylindricalPropagator::connectCB()
    {
        // If no one is listening, we can propagate points in place
        if (ec_pub_.getNumSubscribers() == 0)
        {
            
        }
    }
    
    void EgoCylindricalPropagator::reset()
    {
        WriteLock lock(reset_mutex_);
        should_reset_ = true;
    }
    
    void EgoCylindricalPropagator::configCB(const egocylindrical::PropagatorConfig &config, uint32_t level)
    {
        WriteLock lock(config_mutex_);
     
        ROS_INFO_STREAM("Updating propagator config: height=" << config.height << ", width=" << config.width << ", vfov=" << config.vfov << ", can_width=" << config.can_width
        << ", v_offset=" << config.v_offset << ", cyl_radius=" << config.cyl_radius);
        config_ = config;
    }
    
    
    bool EgoCylindricalPropagator::init()
    {
        reconfigure_server_->setCallback(boost::bind(&EgoCylindricalPropagator::configCB, this, _1, _2));
        
        transformed_pts_ = utils::getECWrapper(config_, true);
        
        next_pts_ = utils::getECWrapper(config_);
                
        
        // Get topic names
        std::string depth_topic="/camera/depth/image_raw", info_topic= "/camera/depth/camera_info", points_topic="egocylindrical_points", filtered_pc_topic="filtered_points", egocylinder_info_topic="egocylinder_info";
        fixed_frame_id_ = "odom";
        
        pnh_.getParam("image_in", depth_topic );
        pnh_.getParam("info_in", info_topic );
        pnh_.getParam("points_out", points_topic );
        pnh_.getParam("filtered_points", filtered_pc_topic);
        
        pnh_.getParam("fixed_frame_id", fixed_frame_id_);
        
        dii_.init();
        cfh_.init();
        
        reset_sub_ = nh_.subscribe<std_msgs::Empty>("reset", 1, [this](const std_msgs::Empty::ConstPtr&) { reset(); });

        // Setup publishers
        ros::SubscriberStatusCallback image_cb = boost::bind(&EgoCylindricalPropagator::connectCB, this);        
        ec_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>(points_topic, 1, image_cb, image_cb);
        
        //ros::SubscriberStatusCallback pc_cb = boost::bind(&EgoCylindricalPropagator::connectCB, this);
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_pc_topic, 3);
        info_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>(egocylinder_info_topic, 1);
        
        // Setup subscribers
        depthSub.subscribe(it_, depth_topic, 3);
        depthInfoSub.subscribe(nh_, info_topic, 3);
        
        // Ensure that CameraInfo is transformable
        info_tf_filter = boost::make_shared<tf_filter>(depthInfoSub, buffer_, fixed_frame_id_, 2,nh_);
        
        // Synchronize Image and CameraInfo callbacks
        timeSynchronizer = boost::make_shared<synchronizer>(depthSub, *info_tf_filter, 2);
        timeSynchronizer->registerCallback(boost::bind(&EgoCylindricalPropagator::update, this, _1, _2));
        
        return true;
    }

    EgoCylindricalPropagator::EgoCylindricalPropagator(ros::NodeHandle& nh, ros::NodeHandle& pnh):
        nh_(nh),
        pnh_(pnh),
        buffer_(),
        tf_listener_(buffer_),
        dii_(buffer_, pnh),
        cfh_(buffer_, pnh),
        it_(nh),
        should_reset_(false)
    {
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        
    }
    
    EgoCylindricalPropagator::~EgoCylindricalPropagator()
    {
      
    }
      


}
