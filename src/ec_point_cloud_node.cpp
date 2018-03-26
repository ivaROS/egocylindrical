//
// Created by root on 2/5/18.
//

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <ros/ros.h>

namespace egocylindrical
{

    namespace utils
    {
        sensor_msgs::PointCloud2 generate_point_cloud(const utils::ECWrapper& points);
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
        ec_sub_ = nh_.subscribe("egocylindrical", 10, &EgoCylinderPointCloudGenerator::ecPointsCB, this);
        
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cylindrical", 5);
    }


private:
    
    void ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
    {
        ROS_INFO("Received EgoCylinderPoints msg");

        ros::WallTime start = ros::WallTime::now();
        
        utils::ECWrapper ec_pts(ec_msg);
        
        sensor_msgs::PointCloud2 msg = utils::generate_point_cloud(ec_pts);
        
        ROS_INFO_STREAM("Generating point cloud took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        

        ROS_INFO("publish egocylindrical pointcloud");
        
        pc_pub_.publish(msg);
        
        
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
