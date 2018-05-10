#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_CORE_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_CORE_H


#include <egocylindrical/ecwrapper.h>
#include <image_geometry/pinhole_camera_model.h>

#include <egocylindrical/utils.h>

namespace egocylindrical
{
    namespace utils
    {

        // TODO: use coordinate converter instead, since we don't need the actual data here
        template <typename U, typename S>
        inline
        void initializeDepthMapping(const utils::ECWrapper& cylindrical_history, const image_geometry::PinholeCameraModel& cam_model, U* inds, S* x, S* y, S* z)
        {
            
            cv::Size image_size = cam_model.reducedResolution();
            int image_width = image_size.width;
            int image_height = image_size.height;
            
            int num_pixels = image_width * image_height;
            
            for(int i = 0; i < image_height; ++i)
            {
                for(int j = 0; j < image_width; ++j)
                {   
                    int image_idx = i*image_width + j;
                    
                    cv::Point2d pt;
                    pt.x = j;
                    pt.y = i;
                    
                    cv::Point3f world_pnt = cam_model.projectPixelTo3dRay(pt);
                    
                    int cyl_idx = cylindrical_history.worldToCylindricalIdx(world_pnt.x,world_pnt.y,world_pnt.z);
                    
                    inds[image_idx] = cyl_idx;
                    
                    x[image_idx] = world_pnt.x;
                    y[image_idx] = world_pnt.y;
                    z[image_idx] = world_pnt.z;

                }
            }
            
        }


        
        template <typename T, typename U, typename S>
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const T* depths, const U* inds, const S* n_x, const S* n_y, const S* n_z, int num_pixels)
        {

            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            
            
            for(int i = 0; i < num_pixels; ++i)
            {
                
                T depth = depths[i];
                
                if(depth == depth)
                {
                    // NOTE: Currently, no check that index is in bounds. As long as the camera's fov fits inside the egocylindrical fov, this is safe
                    U idx = inds[i];
                    
                    S z_val =  n_z[i]*depth;
                    if(!(z[idx] <= z_val))
                    {
                        x[idx] = n_x[i]*depth;
                        y[idx] = n_y[i]*depth;
                        z[idx] = z_val;
                    }
                }
            }         
            
        }
        
        template <typename U, typename S>
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const cv::Mat& image, const U* inds, const S* n_x, const S* n_y, const S* n_z, int num_pixels)
        {
            if(image.depth() == CV_32FC1)
            {
                remapDepthImage(cylindrical_points, (const float*)image.data, inds, n_x, n_y, n_z, num_pixels);
            }
            else if (image.depth() == CV_16UC1)
            {
                remapDepthImage(cylindrical_points, (const uint16_t*)image.data, inds, n_x, n_y, n_z, num_pixels);
            }
        }
        
        template <typename U, typename S>
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const U* inds, const S* n_x, const S* n_y, const S* n_z, int num_pixels)
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
            remapDepthImage(cylindrical_points, image, inds, n_x, n_y, n_z, num_pixels);
        }
        
        
        
        
        class CleanCameraModel : public image_geometry::PinholeCameraModel
        {
        public:
            void init()
            {
                PinholeCameraModel::initRectificationMaps();
            }
        };
        
        // TODO: Template based on data types and select smallest appropriate type that can represent necessary range of indicies?
        class DepthImageRemapper
        {
        private:
            AlignedVector<long int> inds_;
            AlignedVector<float> x_;
            AlignedVector<float> y_;
            AlignedVector<float> z_;

            int num_pixels_;
          
            CleanCameraModel cam_model_;
            
        public:
            // TODO: also trigger update if ECWrapper's parameters change. However, that will be part of a larger restructuring
            inline
            void updateMapping(const ECWrapper& cylindrical_points, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
            {
                if( cam_model_.fromCameraInfo(cam_info) )
                {
                    cam_model_.init();
                    
                    cv::Size image_size = cam_model_.reducedResolution();
                    int image_width = image_size.width;
                    int image_height = image_size.height;
                    
                    num_pixels_ = image_width * image_height;
                    inds_.resize(num_pixels_);
                    x_.resize(num_pixels_);
                    y_.resize(num_pixels_);
                    z_.resize(num_pixels_);
                    
                    ROS_INFO("Generating depth to cylindrical image mapping");
                    initializeDepthMapping(cylindrical_points, cam_model_, inds_.data(), x_.data(), y_.data(), z_.data());
                }
            }
            
            inline
            void remapDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg)
            {
                utils::remapDepthImage(cylindrical_points, image_msg, inds_.data(), x_.data(), y_.data(), z_.data(), num_pixels_);
            }
            
            void update( ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
            {
                ROS_INFO("Updating cylindrical points with depth image");
                
                ros::WallTime start = ros::WallTime::now();
                updateMapping( cylindrical_points, cam_info);
                ros::WallTime mid = ros::WallTime::now();
                
                remapDepthImage( cylindrical_points, image_msg);
                ros::WallTime end = ros::WallTime::now();
                
                ROS_INFO_STREAM("Updating camera model took " <<  (mid - start).toSec() * 1e3 << "ms");
                ROS_INFO_STREAM("Remapping depth image took " <<  (end - mid).toSec() * 1e3 << "ms");
                
            }

        };
      
      
    }

}


#endif //EGOCYLINDRICAL_DEPTH_IMAGE_CORE_H
