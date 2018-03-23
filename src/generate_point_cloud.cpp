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
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>


namespace egocylindrical
{
    
    namespace utils
    {
        
        sensor_msgs::PointCloud2 generate_point_cloud(const cv::Mat& points)
        {
            const int num_cols = points.cols;
            
            pcl::PointCloud<pcl::PointXYZ> pcloud;
            pcloud.points.resize(num_cols);
            pcloud.width = num_cols;
            pcloud.height = 1;
            

            const float* x = points.ptr<float>(0,0);
            const float* y = points.ptr<float>(1,0);
            const float* z = points.ptr<float>(2,0);

            for(int j = 0; j < num_cols; ++j)
            {   pcl::PointXYZ point(x[j],y[j],z[j]);
                pcloud.at(j) = point;
            }

            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(pcloud, msg);
            
            return msg;
        
        }
    }   
}