//
// Created by root on 2/5/18.
//

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

namespace egocylindrical
{
    //Forward declare functions from other compilation units
    namespace utils
    {
        sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history);
    }

class EgoCylinderRangeImageGenerator
{
    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher im_pub_;
    ros::Subscriber ec_sub_;
    

    
    
public:

    EgoCylinderRangeImageGenerator() :
        it_(nh_)
    {
        std::cout<<"Egocylindrical Range Image Node Initialized"<<std::endl;


    }
    
    void init()
    {
        ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderRangeImageGenerator::ecPointsCB, this);
        
        im_pub_ = it_.advertise("range_image", 2);
    }


private:
    
    void ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
    {
        ROS_INFO("Received EgoCylinderPoints msg");

        ros::WallTime start = ros::WallTime::now();
        
        utils::ECWrapper ec_pts(ec_msg);
              
        sensor_msgs::Image::ConstPtr image_ptr = utils::getRawRangeImageMsg(ec_pts);

        ROS_INFO_STREAM("Generating egocylindrical image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        

        ROS_INFO("publish egocylindrical pointcloud");
        
        im_pub_.publish(image_ptr);
        
        
    }

};


}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_range_image_publisher");
    egocylindrical::EgoCylinderRangeImageGenerator s;
    s.init();
    ros::spin();
}
