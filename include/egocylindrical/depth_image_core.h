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
        inline
        void initializeDepthMapping(const utils::ECWrapper& cylindrical_history, const image_geometry::PinholeCameraModel& cam_model, AlignedVector<long int>& inds, AlignedVector<float>& x, AlignedVector<float>& y, AlignedVector<float>& z )
        {
            
            cv::Size image_size = cam_model.reducedResolution();
            int image_width = image_size.width;
            int image_height = image_size.height;
            
            int num_pixels = image_width * image_height;
            
            inds.resize(num_pixels);
            x.resize(num_pixels);
            y.resize(num_pixels);
            z.resize(num_pixels);
            
            
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
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const T* depths, const U* inds, const S* n_x, const S* n_y, const S* n_z, S* t_x, S* t_y, S* t_z, int num_pixels)
        {
            //decltype (inds)::value_type inds_t;
            //typename decltype (*inds) inds_t;
            
            
            //typedef long int inds_t;
            //typename std::remove_reference<decltype(inds)>::type::value_type inds_t;
            
            //const T* depths = (const T*) depth_image.data;
            
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            ros::WallTime start = ros::WallTime::now();
            
            /*
            
            //#pragma omp simd aligned(n_x,n_y,n_z,t_x,t_y,t_z: __BIGGEST_ALIGNMENT__) aligned(depths: 16)
            #pragma GCC ivdep
            for(int i = 0; i < num_pixels; ++i)
            {
                T depth = depths[i];
            
                t_x[i] = n_x[i]*depth;
                t_y[i] = n_y[i]*depth;
                t_z[i] = n_z[i]*depth;
            }
            
            ros::WallTime mid = ros::WallTime::now();
            
            float max_diff = 0;
            
            for(int i = 0; i < num_pixels; ++i)
            {
                U idx = inds[i];
                
                if( t_z[i]==t_z[i] && !(z[idx] <= t_z[i]) )
                {
                    if(z[idx] == z[idx])
                    {
                       max_diff = std::max((z[idx] - t_z[i]), max_diff);
                    }
                    
                    x[idx] = t_x[i];
                    y[idx] = t_y[i];
                    z[idx] = t_z[i];
                }
            }
            
            ros::WallTime end = ros::WallTime::now();
            
            ROS_INFO_STREAM("Generating depth image points took " <<  (mid - start).toSec() * 1e3 << "ms");
            
            ROS_INFO_STREAM("Inserting depth image points took " <<  (end - mid).toSec() * 1e3 << "ms");
            
            ROS_INFO_STREAM("Max single error: " << max_diff);
            
            
            */
            
            for(int i = 0; i < num_pixels; ++i)
            {
                
                T depth = depths[i];
                
                if(depth == depth)
                {
                    U idx = inds[i];
                    
                    if(!(z[idx] <= n_z[i]*depth))
                    {
                        x[idx] = n_x[i]*depth;
                        y[idx] = n_y[i]*depth;
                        z[idx] = n_z[i]*depth;
                    }
                }
            }
            
            ros::WallTime end = ros::WallTime::now();
            
            ROS_INFO_STREAM("Remapping depth image took " <<  (end - start).toSec() * 1e3 << "ms");
            
            
            
        }
        
        template <typename U, typename S>
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const cv::Mat& image, const U* inds, const S* n_x, const S* n_y, const S* n_z, S* t_x, S* t_y, S* t_z, int num_pixels)
        {
            if(image.depth() == CV_32FC1)
            {
                remapDepthImage(cylindrical_points, (const float*)image.data, inds, n_x, n_y, n_z, t_x, t_y, t_z, num_pixels);
            }
            else if (image.depth() == CV_16UC1)
            {
                remapDepthImage(cylindrical_points, (const uint16_t*)image.data, inds, n_x, n_y, n_z, t_x, t_y, t_z, num_pixels);
            }
        }
        
        template <typename U, typename S>
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const U* inds, const S* n_x, const S* n_y, const S* n_z, S* t_x, S* t_y, S* t_z, int num_pixels)
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
            remapDepthImage(cylindrical_points, image, inds, n_x, n_y, n_z, t_x, t_y, t_z, num_pixels);
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
            
            AlignedVector<float> t_x_;
            AlignedVector<float> t_y_;
            AlignedVector<float> t_z_;
            
            int num_pixels_;
          
            CleanCameraModel model_t;
            
        public:
            // TODO: also trigger update if ECWrapper's parameters change. However, that will be part of a larger restructuring
            inline
            void updateMapping(const ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
            {
                if( model_t.fromCameraInfo(cam_info) )
                {
                    model_t.init();
                    
                    ROS_INFO("Generating depth to cylindrical image mapping");
                    initializeDepthMapping(cylindrical_points, model_t, inds_, x_, y_, z_);
                    //const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
                    
                    num_pixels_ = x_.size();
                    
                    t_x_.resize(num_pixels_);
                    t_y_.resize(num_pixels_);
                    t_z_.resize(num_pixels_);
                }

            }
            
            inline
            void remapDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg)
            {
                utils::remapDepthImage(cylindrical_points, image_msg, inds_.data(), x_.data(), y_.data(), z_.data(), t_x_.data(), t_y_.data(), t_z_.data(), num_pixels_);
            }
            
            void update( ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
            {
                ROS_INFO("Updating cylindrical points with depth image");
                
                updateMapping( cylindrical_points, image_msg, cam_info);
                remapDepthImage( cylindrical_points, image_msg);
                
                //utils::addDepthImage(cylindrical_points, image_msg, model_t);
                
            }

        };
      
      
    }

}


#endif //EGOCYLINDRICAL_DEPTH_IMAGE_CORE_H
