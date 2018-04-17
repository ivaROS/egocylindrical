#include <egocylindrical/range_image_generator.h>

//Redundant
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_range_image_publisher");
        
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::EgoCylinderRangeImageGenerator s(nh, pnh);
    s.init();
    //ros::MultiThreadedSpinner spinner(2);
    //spinner.spin();
    ros::spin();
}
