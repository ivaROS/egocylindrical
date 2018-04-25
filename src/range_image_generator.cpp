//
// Created by root on 2/5/18.
//

#include <egocylindrical/range_image_generator.h>
#include <egocylindrical/range_image_core.h>

// The below are redundant
#include <egocylindrical/EgoCylinderPoints.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <egocylindrical/ecwrapper.h>


namespace egocylindrical
{



    EgoCylinderRangeImageGenerator::EgoCylinderRangeImageGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        it_(nh_)
    {
        std::cout<<"Egocylindrical Range Image Node Initialized"<<std::endl;


    }
    
    bool EgoCylinderRangeImageGenerator::init()
    {
        use_raw_ = false;
        std::string image_topic = "image";
        
        pnh_.getParam("use_raw", use_raw_ );
        
        pnh_.getParam("image_topic", image_topic );

        image_transport::SubscriberStatusCallback image_cb = boost::bind(&EgoCylinderRangeImageGenerator::ssCB, this);
        im_pub_ = it_.advertise(image_topic, 2, image_cb, image_cb);
        
        return true;
    }
    
    void EgoCylinderRangeImageGenerator::ssCB()
    {
        
        //std::cout << (void*)ec_sub_ << ": " << im_pub_.getNumSubscribers() << std::endl;

        if(im_pub_.getNumSubscribers()>0)
        {
            if((void*)ec_sub_) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
                ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderRangeImageGenerator::ecPointsCB, this);
                ROS_INFO("RangeImage Generator Subscribing");

            }
      
        }
        else
        {
            ec_sub_.shutdown();
            ROS_INFO("RangeImage Generator Unsubscribing");

        }
    }

    
    void EgoCylinderRangeImageGenerator::ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
    {
        ROS_DEBUG("Received EgoCylinderPoints msg");
        
        // This may be redundant now
        if(im_pub_.getNumSubscribers() > 0)
        {

          ros::WallTime start = ros::WallTime::now();
          
          utils::ECWrapper ec_pts(ec_msg);
                
          sensor_msgs::Image::ConstPtr image_ptr = use_raw_ ? utils::getRawRangeImageMsg(ec_pts) : utils::getRangeImageMsg(ec_pts);

          ROS_INFO_STREAM("Generating egocylindrical image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          ROS_INFO("publish egocylindrical image");
          
          im_pub_.publish(image_ptr);
        }
        
    }




}
