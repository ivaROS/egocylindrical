//
// Created by root on 2/5/18.
//

#include "egocylindrical.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <cv_bridge/cv_bridge.h>



namespace egocylindrical
{

namespace utils
{
    constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
    
    
    //static constexpr float dNan = dNan();
    /*
    dNaN = 0;
        if(std::numeric_limits<float>::has_quiet_NaN)
        {
        dNaN = std::numeric_limits<float>::quiet_NaN();
        }
        */
    
    cv::Point3f worldToCylindrical(cv::Point3f point)
    {
        cv::Point3f Pcyl_t = point / cv::sqrt(cv::pow(point.x, 2) + cv::pow(point.z, 2));   
        
        double theta = std::atan2(point.x,point.z);
        
        double phi = std::atan2(point.y,point.z);
        
        
        
    }

    cv::Point cylindricalToImage(cv::Point3f point)
    {
        
    }

    cv::Point worldToCylindricalImage(cv::Point3f point)
    {
        
        
    }

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
    
    
    
    void fillImage(cv::Mat& cylindrical_history, cv::Mat new_points, bool overwrite)
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
                    cv::Point image_pnt = worldToCylindricalImage(world_pnt);
                    
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



    void EgoCylindricalPropagator::propagateHistory(cv::Mat& old_pnts, cv::Mat& new_pnts, std_msgs::Header old_header, std_msgs::Header new_header)
    {
        ROS_DEBUG("Getting Transformation details");
                geometry_msgs::TransformStamped trans = buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                old_header.frame_id, old_header.stamp,
                                "odom", ros::Duration(1));

        utils::transformPoints(old_pnts, trans);
        utils::fillImage(new_pnts, old_pnts, false);
    }




    void EgoCylindricalPropagator::addDepthImage(cv::Mat& cylindrical_points, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        model_t.fromCameraInfo(cam_info);
        
        cv::Mat new_pnts = utils::depthImageToWorld(image,model_t);
        
        utils::fillImage(cylindrical_points, new_pnts, true);
        
    }


    void EgoCylindricalPropagator::update(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        new_pts_ = cv::Mat(cylinder_height_,cylinder_width_, CV_32FC3, utils::dNaN);
        
        EgoCylindricalPropagator::propagateHistory(old_pts_, new_pts_, old_header_, image->header);
        EgoCylindricalPropagator::addDepthImage(new_pts_, image, cam_info);
        
        cv::swap(new_pts_, old_pts_);
        old_header_ = image->header;
    }

    void EgoCylindricalPropagator::init()
    {
        
        double pi = std::acos(-1);
        hfov_ = 2*pi;
        vfov_ = pi/3;
        cylinder_width_ = 1024;
        cylinder_height_ = 320;
    }

    EgoCylindricalPropagator::EgoCylindricalPropagator(ros::NodeHandle& nh):
        nh_(nh),
        tf_listener_(buffer_)
    {
        init();
        
        
    }


}
