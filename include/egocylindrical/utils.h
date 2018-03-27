#ifndef EGOCYLINDRICAL_UTILS_H
#define EGOCYLINDRICAL_UTILS_H

#include <egocylindrical/ecwrapper.h>

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
#include <sensor_msgs/PointCloud2.h>

#include <egocylindrical/EgoCylinderPoints.h>

namespace egocylindrical
{

namespace utils
{

    struct CylindricalCoordsConverter
    {
       int width, height;
       float hfov, vfov;
       float h_scale, v_scale;
       float h_offset=0, v_offset=0;
       
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
        
        /*
        inline
        cv::Point3f getWorldPoint(const cv::Mat& image, const cv::Point& image_pnt) const
        {
            
        }
        */
        
        /* TODO: check ROI and perform worldToCylindricalImage call in here
         * Goal: abstract away the underlying data representation from the rest of my code
         */
        
        inline
        bool setWorldPoint(cv::Mat& image, const cv::Point3f& world_pnt, const cv::Point& image_pnt)
        {
            image.at<float>(0,image_pnt.y*width + image_pnt.x) = world_pnt.x;
            image.at<float>(1,image_pnt.y*width + image_pnt.x) = world_pnt.y;
            image.at<float>(2,image_pnt.y*width + image_pnt.x) = world_pnt.z;
            
            return true;
        }
        
        
        
    };
    
    
    inline
    void addPoints(utils::ECWrapper& cylindrical_history, const utils::ECWrapper& new_points, const CylindricalCoordsConverter& ccc, bool overwrite)
    {
        cv::Rect image_roi = ccc.getImageROI();
        
        float* x = cylindrical_history.getX();
        float* y = cylindrical_history.getY();
        float* z = cylindrical_history.getZ();
        
        const float* n_x = new_points.getX();
        const float* n_y = new_points.getY();
        const float* n_z = new_points.getZ();
        

        ROS_DEBUG("Relocated the propagated image");
        //#pragma omp parallel for
        for(int i = 0; i < new_points.getCols(); ++i)
        {
            
            cv::Point3f world_pnt(n_x[i],n_y[i],n_z[i]);
            
            float depth = worldToRange(world_pnt);
            
            if(depth==depth)
            {
                // The following 3 steps could probably be moved to the point propagation step and performed in parallel
                // It will depend on whether the extra memory access for the steps cost more or less than the calculations
                cv::Point image_pnt = cylindrical_history.worldToCylindricalImage(world_pnt);
                
                int idx =  image_pnt.y * cylindrical_history.getWidth() +image_pnt.x;
                

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
    void addDepthImage(utils::ECWrapper& cylindrical_history, const cv::Mat& depth_image, const image_geometry::PinholeCameraModel& cam_model)
    {
        ROS_DEBUG("Generating depth to cylindrical image mapping");
        
        const cv::Rect image_roi = cylindrical_history.getImageRoi();
        const int width = cylindrical_history.getWidth();
        //const int height = ccc.height;
        
        float* x = cylindrical_history.getX();
        float* y = cylindrical_history.getY();
        float* z = cylindrical_history.getZ();
                
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
                    cv::Point image_pnt = cylindrical_history.worldToCylindricalImage(world_pnt);
                    
                    if(image_roi.contains(image_pnt))
                    {
                        x[image_pnt.y*width + image_pnt.x] = world_pnt.x;
                        y[image_pnt.y*width + image_pnt.x] = world_pnt.y;
                        z[image_pnt.y*width + image_pnt.x] = world_pnt.z;
                        
                    }
                }
                
            }
        );
        
    }
    
    
    inline
    void addDepthImage(utils::ECWrapper& cylindrical_history, const sensor_msgs::Image::ConstPtr& image_msg, const image_geometry::PinholeCameraModel& cam_model)
    {
        const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
        addDepthImage(cylindrical_history, image, cam_model);
    }
    
    
    
    // Functions defined in separate compilation units:
    sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history);
    
    void transformPoints(utils::ECWrapper& points, const geometry_msgs::TransformStamped& trans);
    
    //sensor_msgs::PointCloud2 generate_point_cloud(const utils::ECWrapper& points);
    
}

}

#endif
