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


EgoCylindrical::EgoCylindrical(sensor_msgs::Image image, sensor_msgs::CameraInfo cam_info) {
    ROS_INFO("start building cylindrical image");
    this->image = image;
    this->cam_info = cam_info;
    originImage = cv::Mat(image.height, image.width, CV_32FC1, &image.data[0]).clone();
//    cv::imshow("testWindow", originImage);
//    cv::waitKey(0);
    height = image.height;
    width = image.width;
    rows = height;
    cols = width;
    image_geometry::PinholeCameraModel model_t;
    model_t.fromCameraInfo(cam_info);
    double fvp, fhp;
    fvp = fhp = 1 / (tan(2 * M_PI / cols));
//    std::cout<<fvp<<fhp<<std::endl;
    double hc = cols / 2;
    double vc = rows / 2;
//    std::cout<<hc<<vc<<std::endl;
    cv::Mat newImage = cv::Mat::zeros(rows, cols, CV_32FC1);
    double temp_y, temp_x;
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            cv::Point2d pt;
            pt.x = j;
            pt.y = i;
            cv::Point3d Pcyl = model_t.projectPixelTo3dRay(pt);
//            std::cout << Pcyl <<std::endl;

//            temp_y = Pcyl.y;
//            temp_x = Pcyl.x;
//            Pcyl.y = Pcyl.z;
//            Pcyl.z = temp_x;
//            Pcyl.x = temp_y;
            Pcyl *= originImage.at<float>(i, j);

            cv::Point3d Pcyl_t = Pcyl / cv::sqrt(cv::pow(Pcyl.x, 2) + cv::pow(Pcyl.z, 2));

//            std::cout<< Pcyl_t.x << Pcyl_t.y <<Pcyl_t.z<<std::endl;
//            std::exit(0);
            coordinate.push_back(Pcyl_t);
            pt.x = atan(Pcyl_t.x / Pcyl_t.z) * fhp + hc;
            pt.y = Pcyl_t.y * fvp + vc;
//            std::cout<<pt.x << pt.y <<std::endl;
//            std::exit(0);






/*
            //THIS SECTION IS FOR TEST PURPOSE
            cv::Point3d tempPcyl_t;
            tempPcyl_t.x = Pcyl_t.z;
            tempPcyl_t.y = Pcyl_t.x;
            tempPcyl_t.z = Pcyl_t.y;
            cv::Point2d newCoord = model_t.project3dToPixel(tempPcyl_t);
//            std::cout<<newCoord.x<<"    "<<newCoord.y<<std::endl;
            newImage.at<float>((int)newCoord.x, (int)newCoord.y) = originImage.at<float>(i, j);
            testindex.push_back(newCoord);*/




            index.push_back(pt);
        }
    }
//    cv::imshow("testWindow", newImage);
//    cv::waitKey(0);
//    std::exit(0);
}


cv::Mat EgoCylindrical::toImage(){
    int cursor = 0;
//    cv::imshow("testImage", originImage);
//    cv::waitKey(0);
    int x, y;
    cv::Mat newImage = cv::Mat::zeros(rows, cols, CV_32FC1);
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            x = (int)index[cursor].y;
            y = (int)index[cursor].x;
//            std::cout<<"x = "<< x <<"   y = " << y<<std::endl;
            if(x >=0 && x < cols && y >=0 && y < rows)
            {
//                std::cout<<originImage.at<float>(i, j)<<std::endl;
                newImage.at<float>(x, y) = originImage.at<float>(i, j);
//                std::cout<<newImage.at<float>(x, y)<<std::endl;
            }
            cursor++;
//            cv::imshow("testImage", originImage);
//            cv::waitKey(0);
//            cv::imshow("testImage", newImage);
//            cv::waitKey(0);
//
//            std::exit(0);

//
        }
    }
/*    ROS_INFO("to image");
    cv::imshow("testImage", newImage);
    cv::waitKey(0);
    std::exit(0);*/
    return newImage;
}

cv::Mat EgoCylindrical::testToImage()
{
    int x, y;
    int cursor = 0;
    cv::Mat newImage = cv::Mat::zeros(rows, cols, CV_32FC1);
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            x = (int)testindex[cursor].y;
            y = (int)testindex[cursor].x;
            std::cout<<"x = "<< x <<"   y = " << y<<std::endl;
            if(x >=0 && x < cols && y >=0 && y < rows)
            {
//                std::cout<<originImage.at<float>(i, j)<<std::endl;
                newImage.at<float>(x, y) = originImage.at<float>(i, j);
//                std::cout<<newImage.at<float>(x, y)<<std::endl;
            }
//            cv::imshow("testImage", originImage);
//            cv::waitKey(0);
//            cv::imshow("testImage", newImage);
//            cv::waitKey(0);
//
//            std::exit(0);

//
        }
    }
    cv::imshow("testImage", newImage);
    cv::waitKey(0);
    std::exit(0);
    return newImage;
}


