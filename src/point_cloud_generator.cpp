//
// Created by root on 2/5/18.
//

#include <egocylindrical/point_cloud_generator.h>
#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>


namespace egocylindrical
{

    namespace utils
    {
        sensor_msgs::PointCloud2::ConstPtr generate_point_cloud(const utils::ECWrapper& points);
    }


    EgoCylinderPointCloudGenerator::EgoCylinderPointCloudGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh)
    {
        std::cout<<"PointCloud publishing Node Initialized"<<std::endl;
    }
    
    bool EgoCylinderPointCloudGenerator::init()
    {
        ros::SubscriberStatusCallback info_cb = boost::bind(&EgoCylinderPointCloudGenerator::ssCB, this);
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cylindrical", 2);
        
        return true;
    }


    void EgoCylinderPointCloudGenerator::ssCB()
    {
        if(pc_pub_.getNumSubscribers()>0)
        {
            if(&ec_sub_) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
                ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderPointCloudGenerator::ecPointsCB, this);
            }
      
        }
        else
        {
            ec_sub_.shutdown();
        }
    }
    
    
    void EgoCylinderPointCloudGenerator::ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
    {
        ROS_DEBUG("Received EgoCylinderPoints msg");

        if(pc_pub_.getNumSubscribers()>0)
        {
          ros::WallTime start = ros::WallTime::now();
          
          utils::ECWrapper ec_pts(ec_msg);
          
          sensor_msgs::PointCloud2::ConstPtr msg = utils::generate_point_cloud(ec_pts);
          
          ROS_INFO_STREAM("Generating point cloud took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          ROS_DEBUG("publish egocylindrical pointcloud");
          
          pc_pub_.publish(msg);
        }
        
    }




}
