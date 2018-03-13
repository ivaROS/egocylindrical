//
// Created by root on 2/5/18.
//

#ifndef EGOCYLINDRICAL_EGOCYLINDRICAL_H
#define EGOCYLINDRICAL_EGOCYLINDRICAL_H

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl.h>


class EgoCylindrical{
private:
    sensor_msgs::Image image;
    sensor_msgs::CameraInfo cam_info;
    int cols;
    int rows;
    int height;
    int width;
    double x, y;
    std::vector<cv::Point3d> coordinate;
    pcl::PointCloud<pcl::PointXYZI> cylidnricalPointCloud;
    pcl::PointCloud<pcl::PointXYZ> worldPointCloud;
    cv::Mat originImage;
public:
    EgoCylindrical();
    EgoCylindrical(sensor_msgs::Image image, sensor_msgs::CameraInfo cam_info);
    pcl::PointCloud<pcl::PointXYZI> getCylindricalPointCloud(){return cylidnricalPointCloud;};
    pcl::PointCloud<pcl::PointXYZ> getWorldPointCloud(){return worldPointCloud;};
    std::string getFrame(){return cam_info.header.frame_id;};
    double getX() {return x;};
    double getY() {return y;};

};





#endif //EGOCYLINDRICAL_EGOCYLINDRICAL_H
