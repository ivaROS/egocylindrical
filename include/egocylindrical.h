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


class EgoCylindrical{
private:
    sensor_msgs::Image image;
    sensor_msgs::CameraInfo cam_info;
    int cols;
    int rows;
    int height;
    int width;
    std::vector<cv::Point3d> coordinate;
    std::vector<cv::Point2d> index;
    cv::Mat originImage;
public:
    EgoCylindrical();
    EgoCylindrical(sensor_msgs::Image image, sensor_msgs::CameraInfo cam_info);
    cv::Mat toImage();


};





#endif //EGOCYLINDRICAL_EGOCYLINDRICAL_H
