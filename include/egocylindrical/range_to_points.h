#include <egocylindrical/ecwrapper.h>

#include <ros/ros.h>
#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf/LinearMath/Matrix3x3.h>
#include <omp.h>
//#include <pcl_ros/point_cloud.h>
//#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>


namespace egocylindrical
{
    
    namespace utils
    {


        template <uint scale, typename S, typename T, typename U>
        inline
        void range_to_points(const utils::ECConverter& info, U* m_ranges, T* x, T* y, T* z)
        {
            S* ranges = (S*) m_ranges;
            
            int image_width = info.getWidth();
            int image_height = info.getHeight();
            
            int image_idx = 0;
                      
            #pragma GCC ivdep
            for(int i = 0; i < image_height; ++i)
            {
                for(int j = 0; j < image_width; ++j)
                {   
                    cv::Point2d pt;
                    pt.x = j;
                    pt.y = i;
                    
                    float range = ranges[image_idx];

                    cv::Point3f world_pnt = info.projectPixelTo3dRay(pt)*range;

                    x[image_idx] = world_pnt.x/scale;
                    y[image_idx] = world_pnt.y/scale;
                    z[image_idx] = world_pnt.z/scale;

                    image_idx++;
                }
            }
            
        }
        
        
        
        ECWrapperPtr range_image_to_wrapper(const ECWrapper& info, const sensor_msgs::Image::ConstPtr image)
        {
          ECWrapperPtr wrapper = getECWrapper(info);
          ECConverter converter;
          converter.fromParams(info.getParams());
          
          if(image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
          {
            range_to_points<1000, uint16_t>(converter, image->data.data(), wrapper->getX(), wrapper->getY(), wrapper->getZ());
          }
          else if(image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
          {
            range_to_points<1, float>(converter, image->data.data(), wrapper->getX(), wrapper->getY(), wrapper->getZ());
          }
          
          wrapper->setHeader(info.getHeader());
          
          return wrapper;
          
        }

    }   
}
