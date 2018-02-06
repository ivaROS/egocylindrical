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
}


EgoCylindrical::EgoCylindrical(sensor_msgs::Image image, sensor_msgs::CameraInfo cam_info){
    this->image = image;
    this->cam_info = cam_info;
    originImage = cv::Mat(image.height, image.width, CV_32FC1, &image.data[0]);
    height = image.height;
    width = image.width;
    rows = height;
    cols = width;
    image_geometry::PinholeCameraModel model_t;
    model_t.fromCameraInfo(cam_info);
    double fvp, fhp;
    fvp = fhp = 1 / (tan(2 * M_PI) / cols);
    double hc = cols / 2;
    double vc = rows / 2;
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            cv::Point2d pt;
            pt.x = i;
            pt.y = j;
            cv::Point3d Pcyl = model_t.projectPixelTo3dRay(pt);
            cv::Point3d Pcyl_t = Pcyl / cv::sqrt(cv::pow(Pcyl.x, 2) + cv::pow(Pcyl.z, 2));
            coordinate.push_back(Pcyl_t);
            pt.x = atan(Pcyl_t.x / Pcyl_t.z) * fhp + hc;
            pt.y = Pcyl_t.y * fvp + vc;
            index.push_back(pt);
        }
    }
}


cv::Mat EgoCylindrical::toImage(){
    int cursor = 0;
    cv::Mat newImage = cv::Mat::zeros(rows, cols, CV_32FC1);
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {

            newImage.at<float>(int(index[cursor].x), int(index[cursor].y)) = originImage.at<float>(i, j);
        }
    }
}





int main(int argc, char** argv)
{
    return 0;
}