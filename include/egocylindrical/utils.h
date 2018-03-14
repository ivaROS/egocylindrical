#ifndef EGOCYLINDRICAL_UTILS_H
#define EGOCYLINDRICAL_UTILS_H


#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>


namespace egocylindrical
{

namespace utils
{
    constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
    
    


   /*
    
    cv::Point3f worldToCylindrical(cv::Point3f point, int cyl_width, int cyl_height, double hfov, double vfov)
    {
        cv::Point3f Pcyl_t = point / cv::sqrt(cv::pow(point.x, 2) + cv::pow(point.z, 2));   
        
        double theta = std::atan2(point.x,point.z);
        double phi = std::atan2(point.y,point.z);
        
        cyl_width* theta /hfov
        
        
    }
    */
    
    
    inline
    cv::Point cylindricalToImage(cv::Point3f point)
    {
        
    }
    

    inline
    cv::Point worldToCylindricalImage(cv::Point3f point, int cyl_width, int cyl_height, double hfov, double vfov)
    {
        double theta = std::atan2(point.x,point.z);
        double phi = std::atan2(point.y,point.z);
        
        int y = cvRound(cyl_height/2 + cyl_height * phi / vfov);
        int x = (cvRound(cyl_width/2 + cyl_width * theta / hfov) % cyl_width);
        
        cv::Point im_pt(x,y);
        return im_pt;
    }

    
    struct CylindricalCoordsConverter
    {
       double hfov, vfov;
       int width, height;
       
       CylindricalCoordsConverter()
       {
       }
       
       CylindricalCoordsConverter(int width, int height, double hfov, double vfov):
            width(width),
            height(height),
            hfov(hfov),
            vfov(vfov)
        {
        }
        
        cv::Point worldToCylindricalImage(cv::Point3f point) const
        {
            return utils::worldToCylindricalImage(point, width, height, hfov, vfov);
        }
        
    };
    
    inline
    void transformPoints(cv::Mat& points, geometry_msgs::TransformStamped& trans)
    {
        tf::Quaternion rotationQuaternion = tf::Quaternion(trans.transform.rotation.x,
                            trans.transform.rotation.y,
                            trans.transform.rotation.z,
                            trans.transform.rotation.w);


        tf::Matrix3x3 tempRotationMatrix = tf::Matrix3x3(rotationQuaternion);


        ROS_DEBUG("Getting Rotation Matrix");
        float rotationArray[9];
        rotationArray[0] = (float) tempRotationMatrix[0].getX();
        rotationArray[1] = (float) tempRotationMatrix[0].getY();
        rotationArray[2] = (float) tempRotationMatrix[0].getZ();
        rotationArray[3] = (float) tempRotationMatrix[1].getX();
        rotationArray[4] = (float) tempRotationMatrix[1].getY();
        rotationArray[5] = (float) tempRotationMatrix[1].getZ();
        rotationArray[6] = (float) tempRotationMatrix[2].getX();
        rotationArray[7] = (float) tempRotationMatrix[2].getY();
        rotationArray[8] = (float) tempRotationMatrix[2].getZ();
        cv::Mat rotationMatrix = cv::Mat(3, 3, CV_32FC1, &rotationArray[0]);
        
        int img_height = points.rows;
        // maybe create new header cv::Mat points3xn?
        points = points.reshape(1, points.rows*points.cols); //can the next line be combined with this one?
        points *= rotationMatrix.inv();

        ROS_DEBUG("Getting Translation Matrix");
        float translationArray[3];
        translationArray[0] = (float) trans.transform.translation.x;
        translationArray[1] = (float) trans.transform.translation.y;
        translationArray[2] = (float) trans.transform.translation.z;
        
        cv::Vec3f translationVec(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);

        
        points = points.reshape(3,img_height); //can remove if create new ref above
        points += translationVec;   

        ROS_DEBUG("Getting the final matrix");
    
        
        
    }

    inline
    cv::Mat depthImageToWorld(const sensor_msgs::Image::ConstPtr& image, const image_geometry::PinholeCameraModel& cam_model)
    {
        cv::Mat world_pnts(image->height,image->width, CV_32FC3, utils::dNaN);
        
        cv::Mat depth_im = cv_bridge::toCvShare(image)->image;
        for(int i = 0; i < depth_im.rows; i++)
        {
            for(int j = 0; j < depth_im.cols; j++)
            {
                cv::Point2f pt;
                pt.x = j;
                pt.y = i;
                cv::Point3f Pcyl = cam_model.projectPixelTo3dRay(pt);
                Pcyl *= depth_im.at<float>(i, j);
                world_pnts.at<cv::Point3f>(i, j) = Pcyl;
            }
        }
        return world_pnts;
    }
    
    
    inline
    void fillImage(cv::Mat& cylindrical_history, cv::Mat new_points, const CylindricalCoordsConverter& ccc, bool overwrite)
    {
        cv::Rect image_roi(cv::Point(), cylindrical_history.size());

        ROS_DEBUG("Relocated the propagated image");
        for (int j = 0; j < new_points.rows; j++)
        {
            for(int i = 0; i < new_points.cols; ++i)
            {
            
                cv::Point3f world_pnt = new_points.at<cv::Point3f>(j,i);
                float depth = world_pnt.z;
                
                if(!cvIsNaN(depth))
                {
                    cv::Point image_pnt = ccc.worldToCylindricalImage(world_pnt);
                    
                    int yIdx = image_pnt.y;
                    int xIdx = image_pnt.x;
                    

                    //if (xIdx < img_width && yIdx < img_height && xIdx > 0 && yIdx > 0)
                    if(image_roi.contains(image_pnt))
                    {
                        //ROS_DEBUG_STREAM("y,x: " << yIdx << "," << xIdx);
                        float& cur_depth = cylindrical_history.at<cv::Point3f>(yIdx, xIdx).z;
                        
                        if(overwrite || !(cur_depth >= depth))
                        {
                            cylindrical_history.at<cv::Point3f>(image_pnt) = world_pnt;
                        }
                    }
                }
            }
        
        }   
        
    }
    
    
}

}

#endif
