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
#include <omp.h>


namespace egocylindrical
{

namespace utils
{
    constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
    
    

    inline
    float worldToRange(cv::Point3f point)
    {
        return std::sqrt(point.x*point.x + point.z*point.z);
    }
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
    cv::Point3f projectWorldToCylinder(const cv::Point3f& point)
    {
        cv::Point3f Pcyl_t = point / std::sqrt(point.x * point.x + point.z * point.z);
        return Pcyl_t;
    }
    

    inline
    cv::Point worldToCylindricalImage(const cv::Point3f& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
    {
        
        cv::Point3f p_cyl = projectWorldToCylinder(point);
        
        
        float x = std::atan2(p_cyl.x, p_cyl.z) * h_scale + cyl_width / 2;
        float y = p_cyl.y * v_scale + cyl_height / 2;
                
        cv::Point im_pt(x,y);
        return im_pt;
    }

    
    struct CylindricalCoordsConverter
    {
       float hfov, vfov;
       float h_scale, v_scale;
       float h_offset=0, v_offset=0;
       int width, height;
       
       CylindricalCoordsConverter()
       {
       }
       
       CylindricalCoordsConverter(int width, int height, float hfov, float vfov):
            width(width),
            height(height),
            hfov(hfov),
            vfov(vfov)
        {
            h_scale = width/hfov;
            v_scale = height/vfov;
        }
        
        inline
        cv::Point worldToCylindricalImage(cv::Point3f point) const
        {
            return utils::worldToCylindricalImage(point, width, height, h_scale, v_scale, h_offset, v_offset);
        }
        
        inline
        cv::Rect getImageROI() const
        {
            return cv::Rect(cv::Point(), cv::Size(width,height));
            
        }
        
        
        inline
        cv::Point3f getWorldPoint(const cv::Mat& image, const cv::Point& image_pnt) const
        {
            
        }
        
        
        /* TODO: check ROI and perform worldToCylindricalImage call in here
         * Goal: abstract away the underlying data representation from the rest of my code
         */
        
        inline
        bool setWorldPoint(cv::Mat& image, const cv::Point3f& world_pnt, const cv::Point& image_pnt)
        {
            image.at<float>(0,image_pnt.y*width + image_pnt.x) = world_pnt.x;
            image.at<float>(1,image_pnt.y*width + image_pnt.x) = world_pnt.y;
            image.at<float>(2,image_pnt.y*width + image_pnt.x) = world_pnt.z;
        }
        
        
        
    };
    
    

    

    inline
    cv::Mat depthImageToWorld(const sensor_msgs::Image::ConstPtr& image, const image_geometry::PinholeCameraModel& cam_model)
    {
        cv::Mat world_pnts(image->height,image->width, CV_32FC3, utils::dNaN);
        
        const cv::Mat depth_im = cv_bridge::toCvShare(image)->image;
        
        
        world_pnts.forEach<cv::Point3f>
        (
            [&depth_im, &cam_model](cv::Point3f &pixel, const int* position) -> void
            {
                int i = position[0];
                int j = position[1];
                
                cv::Point2f pt;
                pt.x = j;
                pt.y = i;
                cv::Point3f Pcyl = cam_model.projectPixelTo3dRay(pt);
                Pcyl *= depth_im.at<float>(i, j);
                pixel = Pcyl;     
                
            }
        );
        
        return world_pnts;
    }
    
    
    inline
    void fillImage(cv::Mat& cylindrical_history, cv::Mat new_points, const CylindricalCoordsConverter& ccc, bool overwrite)
    {
        cv::Rect image_roi = ccc.getImageROI();
        
        float* x = cylindrical_history.ptr<float>(0,0);
        float* y = cylindrical_history.ptr<float>(1,0);
        float* z = cylindrical_history.ptr<float>(2,0);
        
        const float* n_x = new_points.ptr<float>(0,0);
        const float* n_y = new_points.ptr<float>(1,0);
        const float* n_z = new_points.ptr<float>(2,0);
        

        ROS_DEBUG("Relocated the propagated image");
        //#pragma omp parallel for
        for(int i = 0; i < new_points.cols; ++i)
        {
            
            cv::Point3f world_pnt(n_x[i],n_y[i],n_z[i]);
            
            float depth = worldToRange(world_pnt);
            
            if(depth==depth)
            {
                cv::Point image_pnt = ccc.worldToCylindricalImage(world_pnt);
                
                int idx =  image_pnt.y * ccc.width +image_pnt.x;
                

                if(image_roi.contains(image_pnt))
                {
                    cv::Point3f prev_point(x[idx], y[idx], z[idx]);
                    
                    float prev_depth = worldToRange(prev_point);
                    
                    if(!(prev_depth >= depth)) //overwrite || 
                    {
                        x[idx] = world_pnt.x;
                        y[idx] = world_pnt.y;
                        z[idx] = world_pnt.z;
                    }
                }
                else
                {
                    //ROS_DEBUG_STREAM("Outside of image!: (" << world_pnt << " => " << image_pnt);
                }
            }
        }
    
        
        
    }
    


    
    inline
    void remapDepthImage(const cv::Mat& depth_image, const CylindricalCoordsConverter& ccc, const image_geometry::PinholeCameraModel& cam_model, cv::Mat& cylindrical_history)
    {
        ROS_DEBUG("Generating depth to cylindrical image mapping");
        
        const cv::Rect image_roi = ccc.getImageROI();
        const int width = ccc.width;
        const int height = ccc.height;
                
        depth_image.forEach<float>
        (
            [&](const float &depth, const int* position) -> void
            {
                int i = position[0];
                int j = position[1];
                
                cv::Point2d pt;
                pt.x = j;
                pt.y = i;
                
                if(depth==depth)
                {
                    cv::Point3f world_pnt = cam_model.projectPixelTo3dRay(pt)*depth;
                    cv::Point image_pnt = ccc.worldToCylindricalImage(world_pnt);
                    
                    if(image_roi.contains(image_pnt))
                    {
                        cylindrical_history.at<float>(0,image_pnt.y*width + image_pnt.x) = world_pnt.x;
                        cylindrical_history.at<float>(1,image_pnt.y*width + image_pnt.x) = world_pnt.y;
                        cylindrical_history.at<float>(2,image_pnt.y*width + image_pnt.x) = world_pnt.z;
                        
                    }
                }
                
            }
        );
        
    }
    
    
    inline
    void remapDepthImage(const sensor_msgs::Image::ConstPtr& image_msg, const CylindricalCoordsConverter& ccc, const image_geometry::PinholeCameraModel& cam_model, cv::Mat& cylindrical_history)
    {
        const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
        remapDepthImage(image,ccc, cam_model, cylindrical_history);
    }
    
    sensor_msgs::ImagePtr getRawRangeImageMsg(const cv::Mat& cylindrical_history, const CylindricalCoordsConverter& ccc);
    
    
    void transformPoints(cv::Mat& points, const geometry_msgs::TransformStamped& trans);
    
   
}

}

#endif
