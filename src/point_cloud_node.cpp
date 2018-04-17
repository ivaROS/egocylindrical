//
// Created by root on 2/5/18.
//

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

class EgoCylinderPointCloudGenerator
{
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pc_pub_;
    ros::Subscriber ec_sub_;

    
    
public:

    EgoCylinderPointCloudGenerator()
    {
        std::cout<<"PointCloud publishing Node Initialized"<<std::endl;


    }
    
    void init()
    {
        ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderPointCloudGenerator::ecPointsCB, this);
        
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cylindrical", 2);
    }


private:
    //ros::SubscriberStatusCallback info_cb = boost::bind(&camera_compression::RGBDThrottleNodelet::infoCB, this);
    void ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
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

};


}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_pointcloud_publisher");
    egocylindrical::EgoCylinderPointCloudGenerator s;
    s.init();
    ros::spin();
}
