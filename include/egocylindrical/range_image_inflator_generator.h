#ifndef EGOCYLINDRICAL_RANGE_IMAGE_INFLATOR_GENERATOR_H
#define EGOCYLINDRICAL_RANGE_IMAGE_INFLATOR_GENERATOR_H


//#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <egocylindrical/RangeImageGeneratorConfig.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>

#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <dynamic_reconfigure/server.h>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

typedef boost::shared_mutex Mutex;
typedef boost::unique_lock< Mutex > WriteLock;
typedef boost::shared_lock< Mutex > ReadLock;



namespace egocylindrical
{


    class RangeImageInflatorGenerator
    {
        ros::NodeHandle nh_, pnh_;
        image_transport::ImageTransport it_;
        image_transport::Publisher im_pub_;
        //ros::Subscriber range_img_sub_;
        image_transport::SubscriberFilter im_sub_;
        message_filters::Subscriber<EgoCylinderPoints> ec_sub_;
        
        typedef message_filters::TimeSynchronizer<sensor_msgs::Image, egocylindrical::EgoCylinderPoints> synchronizer;
        boost::shared_ptr<synchronizer> timeSynchronizer_;
        
        
        bool use_raw_;
        
        //Mutex config_mutex_;
        int num_threads_;
        
        sensor_msgs::Image::Ptr preallocated_msg_;
        
        typedef egocylindrical::RangeImageGeneratorConfig ConfigType;
        ConfigType config_;
        typedef dynamic_reconfigure::Server<ConfigType> ReconfigureServer;
        std::shared_ptr<ReconfigureServer> reconfigure_server_;

    public:

        RangeImageInflatorGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        
        bool init();
        
        void configCB(const ConfigType &config, uint32_t level);
        
        void ssCB();



    private:
        
        void imgCB(const sensor_msgs::Image::ConstPtr& range_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg);
      
    };

} // ns egocylindrical



#endif //EGOCYLINDRICAL_RANGE_IMAGE_INFLATOR_GENERATOR_H
