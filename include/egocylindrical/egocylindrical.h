//
// Created by root on 2/5/18.
//

#ifndef EGOCYLINDRICAL_EGOCYLINDRICAL_H
#define EGOCYLINDRICAL_EGOCYLINDRICAL_H

#include <egocylindrical/utils.h>
#include <egocylindrical/depth_image_core.h>

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl.h>

#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <tf2_ros/message_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <dynamic_reconfigure/server.h>
#include <egocylindrical/PropagatorConfig.h>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

typedef boost::shared_mutex Mutex;
typedef boost::unique_lock< Mutex > WriteLock;
typedef boost::shared_lock< Mutex > ReadLock;

namespace egocylindrical
{

class EgoCylindricalPropagator{
private:

    
    Mutex config_mutex_;
    
    int cylinder_height_;
    int cylinder_width_;
    double hfov_, vfov_;
            

    utils::ECWrapperPtr new_pts_, old_pts_, transformed_pts_, next_pts_;

    image_geometry::PinholeCameraModel model_t;
    
    utils::DepthImageRemapper depth_remapper_;

    ros::NodeHandle nh_, pnh_;
    tf2_ros::TransformListener tf_listener_;

    
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter depthSub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> depthInfoSub;
    
    typedef tf2_ros::MessageFilter<sensor_msgs::CameraInfo> tf_filter;
    boost::shared_ptr<tf_filter> info_tf_filter;
    
    typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> synchronizer;
    boost::shared_ptr<synchronizer> timeSynchronizer;
    
    ros::Publisher ec_pub_, pc_pub_;
    
    egocylindrical::PropagatorConfig config_;
    typedef dynamic_reconfigure::Server<egocylindrical::PropagatorConfig> ReconfigureServer;
    std::shared_ptr<ReconfigureServer> reconfigure_server_;
    


    void propagateHistory(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, std_msgs::Header new_header);
    void addDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info);
    
    void connectCB();
    
    void configCB(const egocylindrical::PropagatorConfig &config, uint32_t level);
    
    virtual bool shouldPublish(const utils::ECWrapperPtr& points) { return true;}
    virtual void published(utils::ECWrapperPtr& points) { }

protected:
    tf2_ros::Buffer buffer_;
    std::string fixed_frame_id_;
    
    
public:
    EgoCylindricalPropagator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~EgoCylindricalPropagator();
    
    void update(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info);
    sensor_msgs::PointCloud2  getPropagatedPointCloud();
    sensor_msgs::Image::ConstPtr getRawRangeImage();

    virtual bool init();

    //pcl::PointCloud<pcl::PointXYZI> getCylindricalPointCloud();
    //pcl::PointCloud<pcl::PointXYZ> getWorldPointCloud();


};


}


#endif //EGOCYLINDRICAL_EGOCYLINDRICAL_H
