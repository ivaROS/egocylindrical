//
// Created by root on 3/6/18.
//

#include "CylindricalPropagator.h"

void CylindricalPropagator::registerOriginal(EgoCylindrical t, std::string frame, double x_, double y_, ros::Time time)
{
    cylindricalPointCloud = t.getCylindricalPointCloud();
    worldPointCloud = t.getWorldPointCloud();
    frame_id = frame;
    x = x_;
    y = y_;
    registered = true;
    last = time;
}
void CylindricalPropagator::propagate(EgoCylindrical t, ros::Time now)
{
    propagate(t.getWorldPointCloud(), now);
}

void CylindricalPropagator::propagate(pcl::PointCloud<pcl::PointXYZ> pointCloud, ros::Time now)
{
    geometry_msgs::TransformStamped transform;
    try{

    transform = buffer.lookupTransform(frame_id, now, frame_id, last, "odom");

    }
    catch (tf2::TransformException &ex){
        ROS_ERROR("%s",ex.what());
    }
    sensor_msgs::PointCloud2 temp;
    pcl::toROSMsg(worldPointCloud, temp);
    tf2::doTransform(temp, temp, transform);
    pcl::PointCloud<pcl::PointXYZ> propagatedPointCloud;
    pcl::fromROSMsg(temp, propagatedPointCloud);
    for(pcl::PointXYZ p: propagatedPointCloud)
    {
        if((p.x > x || p.x < x) || (p.y > y))
        {
            pointCloud.push_back(p);
        }
    }
    worldPointCloud.clear();
    worldPointCloud = pointCloud;
    cylindricalPointCloud.clear();
    for(pcl::PointXYZ p: worldPointCloud)
    {
        pcl::PointXYZI pointXYZI;
        float t = cv::sqrt(cv::pow(p.x, 2) + cv::pow(p.z, 2));
        pointXYZI.x = p.x / t;
        pointXYZI.y = p.y / t;
        pointXYZI.z = p.z / t;
        pointXYZI.intensity = p.z;
        cylindricalPointCloud.push_back(pointXYZI);
    }
    last = now;
}


CylindricalPropagator::CylindricalPropagator():listener(buffer)
{nh_ = ros::NodeHandle();x = 0; y = 0;}
CylindricalPropagator::CylindricalPropagator(ros::NodeHandle &nh):listener(buffer)
{nh_= nh; x= 0; y = 0;}
