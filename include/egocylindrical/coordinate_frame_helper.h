#ifndef EGOCYLINDRICAL_COORDINATE_FRAME_HELPER_H
#define EGOCYLINDRICAL_COORDINATE_FRAME_HELPER_H

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ros/node_handle.h>
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace egocylindrical
{
    namespace utils 
    {
        
        struct CoordinateFrameDefinition
        {
            std::string origin_fixed_frame_id, orientation_fixed_frame_id;
            geometry_msgs::PoseStamped pose;
        };
        
        class CoordinateFrameHelper
        {
        protected:
            tf2_ros::Buffer& buffer_;
            std::string fixed_frame_id_;
            ros::NodeHandle pnh_;
            tf2_ros::TransformBroadcaster tf_br_;
            std::string target_frame_id_;
            geometry_msgs::TransformStamped offset_transform_, ecs_, ecc_;
            CoordinateFrameDefinition cfd_;
            std::string tf_prefix_;
            //ros::Time last_update_;
            std_msgs::Header target_header_;
            bool new_cfd_;
            ros::Subscriber pose_sub_;
            
        public:
          
            CoordinateFrameHelper(tf2_ros::Buffer& buffer, ros::NodeHandle pnh):
                buffer_(buffer),
                pnh_(pnh),
                tf_br_(),
                tf_prefix_(""),
                new_cfd_(false)
            {
              
            }
            
            bool init()
            {
                //TODO: use pips::param_utils
                bool status = pnh_.getParam("fixed_frame_id", fixed_frame_id_);
                
                CoordinateFrameDefinition cfd;
                status &= pnh_.getParam("origin_fixed_frame_id", cfd.origin_fixed_frame_id);
                status &= pnh_.getParam("orientation_fixed_frame_id", cfd.orientation_fixed_frame_id);
                
                cfd.pose.header.frame_id = cfd.origin_fixed_frame_id;
                cfd.pose.pose.orientation.w=1;
                
                if(status)
                {
                    ROS_INFO_STREAM("Found all necessary coordinate frame parameters!");
                    updateDefinition(cfd);
                    pose_sub_ = pnh_.subscribe("desired_pose", 2, &CoordinateFrameHelper::desPoseCB, this);
                }
                else
                {
                    ROS_WARN_STREAM("Did not find all necessary coordinate frame parameters! Values are: origin_fixed_frame_id=" << cfd.origin_fixed_frame_id << ", orientation_fixed_frame_id=" << cfd.orientation_fixed_frame_id << ", fixed_frame_id=" << fixed_frame_id_);
                }

                return status;
            }
            
            bool updateTransforms(std_msgs::Header sensor_header)
            {
                //TODO: Lock a recursive mutex
                              //geometry_msgs::TransformStamped ecs;
                //TODO: figure out how to properly handle cases of both frames being the same, etc
                //TODO: possibly only specify orientation fixed frame, since should really be using same origin as the camera regardless
                if(cfd_.orientation_fixed_frame_id=="" || cfd_.origin_fixed_frame_id=="")
                {
                    ROS_WARN_STREAM("[updateTransforms] Frame not specified!");//, using camera frame");
                    //cfd_.origin_fixed_frame_id = cfd_.origin_fixed_frame_id= sensor_header.frame_id;
                    target_header_ = sensor_header;
                    return true;
                }
              
                bool status = updateECSTransform(sensor_header.stamp);
                status &= updateOffsetTransform(sensor_header.stamp);
                status &= updateECCTransform(sensor_header.stamp);
                
                if(status)
                {
                    target_header_.frame_id = getECCFrameId();
                    target_header_.stamp = sensor_header.stamp;
                }
                return status;
            }
            
            void updateDefinition(CoordinateFrameDefinition cfd)
            {
                //TODO: Lock a mutex
                
                
                cfd_ = cfd;
                new_cfd_ = true;
                
                
                
                if(!updateECSTransform(ros::Time()))
                {
                    return;
                }
                if(!updateOffsetTransform(ros::Time()))
                {
                    return;
                }

                ROS_INFO_STREAM("Successfully updated offset definition!");
                return;
            }
            
            std_msgs::Header getTargetHeader()
            {
                return target_header_;
            }
            
            
        protected:
            std::string getECSFrameId()
            {
                return tf_prefix_ + "egocan_stabilized";
            }
            
            std::string getECFrameId()
            {
                return tf_prefix_ + "egocan";
            }
            
            std::string getECCFrameId()
            {
                return tf_prefix_ + "egocan_camera";
            }
            
            void desPoseCB(const geometry_msgs::PoseStamped::ConstPtr& pose)
            {
                cfd_.pose = *pose;
                updateDefinition(cfd_);
            }
                
            bool updateECSTransform(ros::Time stamp)
            {            
                //TODO: Lock a mutex

              
//                 if(cfd_.orientation_fixed_frame_id==cfd_.origin_fixed_frame_id)
//                 {
//                     ROS_WARN_STREAM("[updateECSTransform] Not publishing redundant transform! " << stamp);
//                     return true;
//                 }

                //If we've already computed it, don't do it again
                if(stamp == ecs_.header.stamp && stamp != ros::Time())
                {
                    ROS_WARN_STREAM("[updateECSTransform] Not publishing redundant transform! " << stamp);
                    return true;
                }
                                              
                try
                {
                    ecs_ = buffer_.lookupTransform(cfd_.orientation_fixed_frame_id, stamp, cfd_.origin_fixed_frame_id, stamp, fixed_frame_id_);
                    
                    ecs_.transform.rotation = geometry_msgs::Quaternion();
                    ecs_.transform.rotation.w=1;
                    ecs_.child_frame_id = tf_prefix_ + "egocan_stabilized";
                }
                catch (tf2::TransformException &ex) 
                {
                    ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
                    return false;
                }
                
                ROS_INFO_STREAM("[updateECSTransform] Updated transform! " << stamp);

                
                buffer_.setTransform(ecs_, "coordinate_frame_helper", false);
                tf_br_.sendTransform(ecs_);
                
                return true;
            }
            
            bool updateECCTransform(ros::Time stamp)
            {

                if(stamp == ecc_.header.stamp && stamp != ros::Time())
                {
                    ROS_WARN_STREAM("[updateECCTransform] Not publishing redundant transform! " << stamp);
                    return true;
                }
                
                //geometry_msgs::TransformStamped ecc;
                ecc_.header.stamp = stamp;
                ecc_.header.frame_id = getECFrameId();
                ecc_.child_frame_id = getECCFrameId();
                auto& q = ecc_.transform.rotation;
                q.x=-0.500;
                q.y=0.500;
                q.z=-0.500;
                q.w=-0.500;

                ROS_INFO_STREAM("[updateECCTransform] Updated transform! " << stamp);

                buffer_.setTransform(ecc_, "coordinate_frame_helper", false);
                tf_br_.sendTransform(ecc_);
                
                return true;
            }
            
            bool updateOffsetTransform(ros::Time stamp)
            {
                if(new_cfd_)
                {
                    ROS_INFO_STREAM("[updateOffsetTransform] Have new CoordinateFrameDefinition!");
                    
                    geometry_msgs::PoseStamped des_origin_pose;
                    try
                    {
                        des_origin_pose = buffer_.transform(cfd_.pose, getECSFrameId(), stamp, fixed_frame_id_);
                    }
                    catch (tf2::TransformException &ex) 
                    {
                        ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
                        return false;
                    }
                    
                    auto& t = offset_transform_.transform.translation;
                    const auto& pt = des_origin_pose.pose.position;
                    
                    t.x = pt.x;
                    t.y = pt.y;
                    t.z = pt.z;
                    offset_transform_.transform.rotation = des_origin_pose.pose.orientation;
                    offset_transform_.header.frame_id =  getECSFrameId();
                    offset_transform_.header.stamp = des_origin_pose.header.stamp;
                    offset_transform_.child_frame_id = getECFrameId();
                    ROS_INFO_STREAM("Successfully processed new offset definition!");

                    new_cfd_ = false;
                }
                else
                {                    
                    if(stamp == offset_transform_.header.stamp && stamp != ros::Time())
                    {
                        ROS_WARN_STREAM("[updateOffsetTransform] Not publishing redundant transform! " << stamp);
                        return true;
                    }
                    offset_transform_.header.stamp = stamp;
                }

                ROS_INFO_STREAM("[updateOffsetTransform] Updated transform! " << stamp);

                buffer_.setTransform(offset_transform_, "coordinate_frame_helper", false);
                tf_br_.sendTransform(offset_transform_);
                
                return true;
            }
          
        };
    } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_COORDINATE_FRAME_HELPER_H
