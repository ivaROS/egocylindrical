//
// Created by root on 2/5/18.
//

#include "egocylindrical.h"

EgoCylindrical::EgoCylindrical()
{
    image = sensor_msgs::Image();
    cam_info = sensor_msgs::CameraInfo();
    cols = 0;
    rows = 0;
    height = 0;
    width = 0;
    x = 0;
    y = 0;
}


EgoCylindrical::EgoCylindrical(sensor_msgs::Image image, sensor_msgs::CameraInfo cam_info) {
    ROS_INFO("start building cylindrical image");
    this->image = image;
    this->cam_info = cam_info;
    originImage = cv::Mat(image.height, image.width, CV_32FC1, &image.data[0]).clone();
    height = image.height;
    width = image.width;
    rows = height;
    cols = width;
    image_geometry::PinholeCameraModel model_t;
    model_t.fromCameraInfo(cam_info);
    cv::Mat newImage = cv::Mat::zeros(rows, cols, CV_32FC1);
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            cv::Point2d pt;
            pt.x = j;
            pt.y = i;
            cv::Point3d Pcyl = model_t.projectPixelTo3dRay(pt);
            Pcyl *= originImage.at<float>(i, j);
            pcl::PointXYZ pointXYZ;
            pointXYZ.x = (float)Pcyl.x;
            pointXYZ.y = (float)Pcyl.y;
            pointXYZ.z = (float)Pcyl.z;
            worldPointCloud.push_back(pointXYZ);
            cv::Point3d Pcyl_t = Pcyl / cv::sqrt(cv::pow(Pcyl.x, 2) + cv::pow(Pcyl.z, 2));
            coordinate.push_back(Pcyl_t);
            pcl::PointXYZI pointXYZI;
            pointXYZI.x = (float)Pcyl_t.x;
            pointXYZI.y = (float)Pcyl_t.y;
            pointXYZI.z = (float)Pcyl_t.z;
            pointXYZI.intensity = originImage.at<float>(i, j);
            cylidnricalPointCloud.push_back(pointXYZI);
            if (Pcyl_t.x > x) x = Pcyl_t.x;
            if (Pcyl_t.y > y) y = Pcyl_t.y;

        }
    }
}




