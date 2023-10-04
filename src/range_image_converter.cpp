//
// Created by root on 2/5/18.
//

#include <egocylindrical/range_image_converter.h>
#include <egocylindrical/range_to_points.h>

// The below are redundant
#include <egocylindrical/EgoCylinderPoints.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <egocylindrical/ecwrapper.h>


namespace egocylindrical
{



    RangeImageConverter::RangeImageConverter(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        it_(nh_)
    {
        std::cout<<"Egocylindrical Range Image Converter Node Initialized"<<std::endl;


    }
    
    bool RangeImageConverter::init()
    {

        
        // Synchronize Image and CameraInfo callbacks
        timeSynchronizer = boost::make_shared<synchronizer>(im_sub_, ec_sub_, 20);
        timeSynchronizer->registerCallback(boost::bind(&RangeImageConverter::imageCB, this, _1, _2));
        
        ros::SubscriberStatusCallback info_cb = boost::bind(&RangeImageConverter::ssCB, this);
        {
            Lock lock(connect_mutex_);
            ec_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>("data_out", 2, info_cb, info_cb);
        }
        
        return true;
    }
    
    void RangeImageConverter::ssCB()
    {
        
        //std::cout << (void*)ec_sub_ << ": " << im_pub_.getNumSubscribers() << std::endl;
        Lock lock(connect_mutex_);
        if(ec_pub_.getNumSubscribers()>0)
        {
            //Note: should probably add separate checks for each
            if((void*)im_sub_.getSubscriber()) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
                im_sub_.subscribe(it_, "image_in", 2);
                ec_sub_.subscribe(nh_, "info_in", 2);
                
                ROS_INFO("RangeImage Converter Subscribing");

            }
      
        }
        else
        {
            im_sub_.unsubscribe();
            ec_sub_.unsubscribe();
            ROS_INFO("RangeImage Converter Unsubscribing");

        }
    }

    //NOTE: Once the parameters have been moved to their own message, this should subscribe to the parameters instead
    void RangeImageConverter::imageCB(const sensor_msgs::Image::ConstPtr& image, const egocylindrical::EgoCylinderPoints::ConstPtr& info)
    {
        ROS_DEBUG("Received range msg");
        
        // This may be redundant now
        if(ec_pub_.getNumSubscribers() > 0)
        {

          ros::WallTime start = ros::WallTime::now();
          
          utils::ECWrapperPtr ec_pts = utils::range_image_to_wrapper(info, image);
          
          utils::ECMsgConstPtr ec_msg = ec_pts->getEgoCylinderPointsMsg();
          
                

          ROS_DEBUG_STREAM("Generating egocylindrical image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          ROS_DEBUG("publish egocylindrical image");
          
          ec_pub_.publish(ec_msg);
        }
        
    }




}
