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
        return cv::sqrt(cv::pow(point.x, 2) + cv::pow(point.z, 2));
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
    
    
    /* Inplace transform
     * 
     */
    inline
    void transform_impl(cv::Mat& points, const float* __restrict__ const R, const float* __restrict__ const T)
    {
        float* __restrict__ x = points.ptr<float>(0,0);
        float* __restrict__ y = points.ptr<float>(1,0);
        float* __restrict__ z = points.ptr<float>(2,0);
        
        float* point_ptr[] = {x,y,z};
        
        #pragma omp simd
        for(size_t p = 0; p < points.cols; ++p)
        {
            for(int row=0; row < 3; ++row)
            {
                float temp = 0;
                for(int col=0; col < 3; ++col)
                {
                    temp += R[row*3+col] * point_ptr[col][p]; // points.at<float>(col,p);
                }
                point_ptr[row][p] = temp + T[row];
            }
        }
    }
    
    
    inline
    void transform_impl2(const cv::Mat& points, cv::Mat& points_t, float* __restrict__ R, float* __restrict__ T)
    {
        const float* __restrict__ x = points.ptr<float>(0,0);
        const float* __restrict__ y = points.ptr<float>(1,0);
        const float* __restrict__ z = points.ptr<float>(2,0);
        
        const float* __restrict__ point_ptr[] = {x,y,z};
        
        
        float* __restrict__ n_x = points_t.ptr<float>(0,0);
        float* __restrict__ n_y = points_t.ptr<float>(1,0);
        float* __restrict__ n_z = points_t.ptr<float>(2,0);
        
        float* __restrict__ new_point_ptr[] = {n_x,n_y,n_z};
        
        #pragma omp simd  //aligned(variable[:alignment] [,variable[:alignment]])
        for(size_t p = 0; p < points.cols; ++p)
        {
            for(int row=0; row < 3; ++row)
            {
                float temp = 0;
                for(int col=0; col < 3; ++col)
                {
                    temp += R[row*3+col] * point_ptr[col][p]; // points.at<float>(col,p);
                }
                new_point_ptr[row][p] = temp + T[row];
            }
        }
    }
    
    inline
    void transformPoints(cv::Mat& points, const geometry_msgs::TransformStamped& trans)
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
        
        std::cout << "temp.dims = " << points.dims << "temp.size = [";
        for(int i = 0; i < points.dims; ++i) {
            if(i) std::cout << " X ";
            std::cout << points.size[i];
        }
        std::cout << "] temp.channels = " << points.channels() << std::endl;
        
        float translationArray[3];
        translationArray[0] = trans.transform.translation.x;
        translationArray[1] = trans.transform.translation.y;
        translationArray[2] = trans.transform.translation.z;
        
        cv::Mat transformed_pts(points.rows, points.cols, points.type());
        
        ros::WallTime start = ros::WallTime::now();
        transform_impl(points, rotationArray, translationArray);
        ROS_INFO_STREAM("Transform function took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        start = ros::WallTime::now();
        transform_impl2(points, transformed_pts, rotationArray, translationArray);
        ROS_INFO_STREAM("Transform2 function took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        
        
        cv::swap(transformed_pts,points);

        ROS_DEBUG("Getting the final matrix");

    }
    

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
        cv::Rect image_roi(cv::Point(), cylindrical_history.size());
        
        float* x = cylindrical_history.ptr<float>(0,0);
        float* y = cylindrical_history.ptr<float>(1,0);
        float* z = cylindrical_history.ptr<float>(2,0);
        
        const float* n_x = new_points.ptr<float>(0,0);
        const float* n_y = new_points.ptr<float>(1,0);
        const float* n_z = new_points.ptr<float>(2,0);
        

        ROS_DEBUG("Relocated the propagated image");
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
                    
                    if(overwrite || !(prev_depth >= depth))
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
    
    
    inline
    cv::Mat getRawRangeImage(const cv::Mat& cylindrical_history)
    {
        cv::Rect image_roi(cv::Point(), cylindrical_history.size());
        
        cv::Mat range_image(1, cylindrical_history.size().width, CV_32FC1);
            
        ROS_DEBUG("Generating image of cylindrical memory");
        
        std::cout << "temp.dims = " << cylindrical_history.dims << "temp.size = [";
        for(int i = 0; i < cylindrical_history.dims; ++i) {
            if(i) std::cout << " X ";
            std::cout << cylindrical_history.size[i];
        }
        std::cout << "] temp.channels = " << cylindrical_history.channels() << std::endl;
        
        
        float* r = range_image.ptr<float>(0);
        const float* x = cylindrical_history.ptr<float>(0,0);
        const float* y = cylindrical_history.ptr<float>(1,0);
        const float* z = cylindrical_history.ptr<float>(2,0);
        
        for(int j = 0; j < cylindrical_history.cols; ++j)
        {
            cv::Point3f world_pnt(x[j],y[j],z[j]);
            float depth = worldToRange(world_pnt);
            
            bool a;
            if(depth == depth)
            {
                a = true;
            }
            
            r[j] = depth;
        }
        
        return range_image;
        
    }
    
    
}

}

#endif
