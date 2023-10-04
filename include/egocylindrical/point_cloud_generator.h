//
// Created by root on 2/5/18.
//

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/mutex.hpp>


namespace egocylindrical
{

    namespace utils
    {
        sensor_msgs::PointCloud2::ConstPtr generate_point_cloud(const utils::ECWrapper& points);
    }

class EgoCylinderPointCloudGenerator
{
    using Mutex = boost::mutex;
    using Lock = Mutex::scoped_lock;

    ros::NodeHandle nh_, pnh_;
    ros::Publisher pc_pub_;
    ros::Subscriber ec_sub_;

    Mutex connect_mutex_;
    
public:

    EgoCylinderPointCloudGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool init();


private:
    
    void ssCB();

    void ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg);

};


}
