#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_INSERTER_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_INSERTER_H

#include <egocylindrical/point_transformer_object.h>
#include <egocylindrical/ecwrapper.h>
#include <image_geometry/pinhole_camera_model.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <egocylindrical/utils.h>

#include <egocylindrical/depth_image_core.h>  //TODO: Move general purpose components into separate header

namespace egocylindrical
{
    namespace utils
    {
      
//         template <typename T>
//         struct PointRef
//         {
//           T& x, y, z;
//           
//           PointRef(T& x, T& y, T& z):
//             x(x),
//             y(y),
//             z(z)
//             {}
//         };
// 
//         template <typename T>
//         class PointSource
//         {
//         public:
//             
//             struct Iterator 
//             {
//                 using iterator_category = std::forward_iterator_tag;
//                 using difference_type   = std::ptrdiff_t;
//                 using value_type        = PointRef<T>;
//                 using pointer           = PointRef<T>*;  // or also value_type*
//                 using reference         = PointRef<T>&;  // or also value_type&
//                 
//                 Iterator(PointSource* source):
//                     source_(source)
//                 {
//                   
//                 }
//                 
//             private:
//                 PointSource* source_;
//             };
//             
//             virtual Iterator::begin()=0;
//             virtual Iterator::end()=0;
//         };


//         class DepthImagePointSource : public PointSource
//         {
//             
// 
//         };

        template <typename T>
        void insertPoints(utils::ECWrapper& cylindrical_points, const cv::Mat image, const image_geometry::PinholeCameraModel& cam_model, const geometry_msgs::TransformStamped transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            int image_width = image_size.width;
            int image_height = image_size.height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
                                    
            for(int i = 0; i < image_height; ++i)
            {
                for(int j = 0; j < image_width; ++j)
                {                       
                    cv::Point2d pt;
                    pt.x = j;
                    pt.y = i;
                    
                    T depth = image.at<T>(i,j);
                    
                    if(depth==depth)  //Only insert actual points
                    {
                    
                        cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                        cv::Point3f world_pnt = ray * (float)(depth/scale);
                        cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                        
                        int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                        
                        if(cyl_idx >= 0  && cyl_idx < max_ind)
                        {
                            float depth = worldToRangeSquared(transformed_pnt);
                    
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_depth = worldToRangeSquared(prev_point);
                            
                            if(!(prev_depth <= depth)) //overwrite || 
                            {   
                                x[cyl_idx] = transformed_pnt.x;
                                y[cyl_idx] = transformed_pnt.y;
                                z[cyl_idx] = transformed_pnt.z;
                            }
                        }
                        else
                        {
                            cyl_idx = cylindrical_points.worldToCanIdx(transformed_pnt);
                        
                            if(cyl_idx >=0 && cyl_idx < cylindrical_points.getNumPts())
                            {
                                float depth = worldToCanDepth(transformed_pnt);
                                
                                cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                                
                                float prev_depth = worldToCanDepth(prev_point);
                                
                                if(!(prev_depth <= depth)) //overwrite || 
                                {   
                                    
                                    x[cyl_idx] = transformed_pnt.x;
                                    y[cyl_idx] = transformed_pnt.y;
                                    z[cyl_idx] = transformed_pnt.z;
                                }
                            }
                        }
                    }
                    
                }
            }
        }



        inline
        void insertPoints(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const image_geometry::PinholeCameraModel& cam_model, const geometry_msgs::TransformStamped transform)
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;

            if(image.depth() == CV_32FC1)
            {
                insertPoints<float>(cylindrical_points, image, cam_model, transform);
            }
            else if (image.depth() == CV_16UC1)
            {
                insertPoints<uint16_t>(cylindrical_points, image, cam_model, transform);
            }
        }



        class DepthImageInserter
        {
            tf2_ros::Buffer& buffer_;
            std::string fixed_frame_id_;
            ros::NodeHandle pnh_;
            CleanCameraModel cam_model_;

          
        public:
            DepthImageInserter(tf2_ros::Buffer& buffer, ros::NodeHandle pnh):
                buffer_(buffer),
                pnh_(pnh)
            {}
            
            bool init()
            {
                //TODO: use pips::param_utils
                bool status = pnh_.getParam("fixed_frame_id", fixed_frame_id_);

                return status;
            }
            
            bool insert(ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
            {
                //Get transform
                geometry_msgs::TransformStamped transform;
                try
                {
                    const std_msgs::Header& target_header = cylindrical_points.getHeader();
                    const std_msgs::Header& source_header = image_msg->header;
                    
                    transform = buffer_.lookupTransform(target_header.frame_id, target_header.stamp, source_header.frame_id, source_header.stamp, fixed_frame_id_);
                }
                catch (tf2::TransformException &ex) 
                {
                    ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
                    return false;
                }
                
                insertPoints(cylindrical_points, image_msg, cam_model_, transform);
            }


        };
        
    } //end namespace utils
} //end namespace egocylindrical
#endif //EGOCYLINDRICAL_DEPTH_IMAGE_INSERTER_H
