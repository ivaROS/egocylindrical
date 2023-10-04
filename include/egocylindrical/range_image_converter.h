#ifndef EGOCYLINDRICAL_RANGE_IMAGE_CONVERTER_H
#define EGOCYLINDRICAL_RANGE_IMAGE_CONVERTER_H


//#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>

#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

namespace egocylindrical
{


    class RangeImageConverter
    {
        using Mutex = boost::mutex;
        using Lock = Mutex::scoped_lock;

        ros::NodeHandle nh_, pnh_;
        image_transport::ImageTransport it_;

        image_transport::SubscriberFilter im_sub_;
        message_filters::Subscriber<egocylindrical::EgoCylinderPoints> ec_sub_;
        
        typedef message_filters::TimeSynchronizer<sensor_msgs::Image, egocylindrical::EgoCylinderPoints> synchronizer;
        boost::shared_ptr<synchronizer> timeSynchronizer;
        
        ros::Publisher ec_pub_;

        Mutex connect_mutex_;

    public:

        RangeImageConverter(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        
        bool init();
        
        void ssCB();



    private:
        
        void imageCB(const sensor_msgs::Image::ConstPtr& image, const egocylindrical::EgoCylinderPoints::ConstPtr& info);

    };

} // ns egocylindrical



#endif //EGOCYLINDRICAL_RANGE_IMAGE_CONVERTER_H

