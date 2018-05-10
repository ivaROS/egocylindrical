#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_CORE_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_CORE_H


#include <egocylindrical/ecwrapper.h>
#include <image_geometry/pinhole_camera_model.h>


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


        
        template <typename T>
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const cv::Mat& depth_image, const AlignedVector<long int>& inds, const AlignedVector<float>& n_x, const AlignedVector<float>& n_y, const AlignedVector<float>& n_z )
        {
            //decltype (inds)::value_type inds_t;
            
            typedef long int inds_t;
            //typename std::remove_reference<decltype(inds)>::type::value_type inds_t;
            
            const T* depths = (const T*) depth_image.data;
            
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            const int num_pixels = inds.size();
            
            
            for(int i = 0; i < num_pixels; ++i)
            {
                
                T depth = depths[i];
                
                if(depth == depth)
                {
                    inds_t idx = inds[i];
                    x[idx] = n_x[i]*depth;
                    y[idx] = n_y[i]*depth;
                    z[idx] = n_z[i]*depth;
                }
            }
            
        }
        
        
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const cv::Mat& image, const AlignedVector<long int>& inds, const AlignedVector<float>& n_x, const AlignedVector<float>& n_y, const AlignedVector<float>& n_z )
        {
            if(image.depth() == CV_32FC1)
            {
                remapDepthImage<float>(cylindrical_points, image, inds, n_x, n_y, n_z);
            }
            else if (image.depth() == CV_16UC1)
            {
                remapDepthImage<uint16_t>(cylindrical_points, image, inds, n_x, n_y, n_z);
            }
        }
        
        inline
        void remapDepthImage(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const AlignedVector<long int>& inds, const AlignedVector<float>& n_x, const AlignedVector<float>& n_y, const AlignedVector<float>& n_z )
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
            remapDepthImage(cylindrical_points, image, inds, n_x, n_y, n_z);
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
                    
                }

            }
            
            inline
            void remapDepthImage2(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg)
            {
                remapDepthImage(cylindrical_points, image_msg, inds_, x_, y_, z_);
            }
            
            void update( ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
            {
                ROS_INFO("Updating cylindrical points with depth image");
                
                updateMapping( cylindrical_points, image, cam_info);
                remapDepthImage2( cylindrical_points, image);
            }

        };
      
      
    }

}


#endif //EGOCYLINDRICAL_DEPTH_IMAGE_CORE_H
