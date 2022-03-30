#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_INSERTER_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_INSERTER_H

#include <egocylindrical/depth_image_core.h>
#include <egocylindrical/depth_image_common.h>
#include <egocylindrical/ecwrapper.h>

#include <ros/node_handle.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>



namespace egocylindrical
{
    namespace utils
    {

        class DepthImageInserter
        {
            tf2_ros::Buffer& buffer_;
            std::string fixed_frame_id_;
            ros::NodeHandle pnh_;
            CleanCameraModel cam_model_;
            utils::DepthImageRemapper depth_remapper_;
          
        public:
            DepthImageInserter(tf2_ros::Buffer& buffer, ros::NodeHandle pnh);
            
            bool init();
            
            bool insert(ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info);

        };
        
    } //end namespace utils
} //end namespace egocylindrical
#endif //EGOCYLINDRICAL_DEPTH_IMAGE_INSERTER_H
