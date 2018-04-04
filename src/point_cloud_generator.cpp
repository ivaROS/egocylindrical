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
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>


namespace egocylindrical
{
    
    namespace utils
    {
        /* TODO: ensure that the right size 'ints' are used everywhere. On my current system, the size of int = minimum size of long int, so 32 bit system might fail */
        sensor_msgs::PointCloud2::ConstPtr generate_point_cloud(const utils::ECWrapper& points)
        {
            const int num_cols = points.getCols();
            
            pcl::PointCloud<pcl::PointXYZ> pcloud;
            
            
            sensor_msgs::PointCloud2::Ptr pcloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
            
            pcl::toROSMsg(pcloud, *pcloud_msg);
            
            pcloud_msg->data.resize(sizeof(pcl::PointXYZ) * num_cols);
            
            pcloud_msg->width = num_cols;
            pcloud_msg->height = 1;
            
            
            
            /*
            
            pcloud.points.resize(num_cols);
            pcloud.width = num_cols;
            pcloud.height = 1;
            
*/
            const float* x = points.getX();
            const float* y = points.getY();
            const float* z = points.getZ();
            
            float* data = (float*) pcloud_msg->data.data();

            /*
            #pragma omp parallel for num_threads(4)
            for(int j = 0; j < num_cols; ++j)
            {   pcl::PointXYZ point(x[j],y[j],z[j]);
                pcloud.at(j) = point;
            }
            */
            
            
            #pragma omp parallel for num_threads(4)
            for(int j = 0; j < num_cols; ++j)
            {   
                data[4*j] = x[j];
                data[4*j+1] = y[j];
                data[4*j+2] = z[j];
            }
            
            
            
            
            

            //ros::WallTime start = ros::WallTime::now();
            //sensor_msgs::PointCloud2 msg;
            //pcl::toROSMsg(pcloud, msg);
            
            //ROS_INFO_STREAM("pointcloud conversion took " << (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
            
            pcloud_msg->header = points.getHeader();
            
            return pcloud_msg;
        
        }
        
        /*
        toPCLPointCloud2 (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)
        {
            // Ease the user's burden on specifying width/height for unorganized datasets
            if (cloud.width == 0 && cloud.height == 0)
            {
                msg.width  = static_cast<uint32_t>(cloud.points.size ());
                msg.height = 1;
            }
            else
            {
                assert (cloud.points.size () == cloud.width * cloud.height);
                msg.height = cloud.height;
                msg.width  = cloud.width;
            }
            
            // Fill point cloud binary data (padding and all)
            size_t data_size = sizeof (PointT) * cloud.points.size ();
            msg.data.resize (data_size);
            if (data_size)
            {
                memcpy(&msg.data[0], &cloud.points[0], data_size);
            }
            
            // Fill fields metadata
            msg.fields.clear ();
            for_each_type<typename traits::fieldList<PointT>::type> (detail::FieldAdder<PointT>(msg.fields));
            
            msg.header     = cloud.header;
            msg.point_step = sizeof (PointT);
            msg.row_step   = static_cast<uint32_t> (sizeof (PointT) * msg.width);
            msg.is_dense   = cloud.is_dense;
        }
        
        fromPCL(pcl_pc2.fields, pc2.fields);
        pc2.is_bigendian = pcl_pc2.is_bigendian;
        pc2.point_step = pcl_pc2.point_step;
        pc2.row_step = pcl_pc2.row_step;
        pc2.is_dense = pcl_pc2.is_dense;
        
        */
    }   
}