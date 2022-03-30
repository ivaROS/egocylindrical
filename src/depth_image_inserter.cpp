#include <egocylindrical/depth_image_inserter.h>

#include <egocylindrical/point_transformer_object.h>
#include <egocylindrical/depth_image_common.h>
#include <egocylindrical/ecwrapper.h>

#include <ros/node_handle.h>
#include <tf2_ros/buffer.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cstdlib>

namespace egocylindrical
{
    namespace utils
    {
        //whole image vectorization w/ inds
        template <typename T>
        void insertPoints4(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            const int image_width = image_size.width;
            const int image_height = image_size.height;
            const int num_pixels = image_width * image_height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
            
            AlignedVector<float> ranges(num_pixels, dNaN), nx(num_pixels, dNaN), ny(num_pixels, dNaN), nz(num_pixels, dNaN);
            AlignedVector<int32_t> inds(num_pixels);
            
            const T* const imgptr = (T* const) image.data;
            
            const float row_factor = ((float) 1)/image_width;
            
            for(int i = 0; i < num_pixels; ++i)
            {
                float raw_row = i*row_factor;
                float row = std::floor(raw_row);
                float decimal = raw_row - row;
                float col = decimal * image_width;
                //auto res = std::div(i, image_width); int row = res.quot; int col = res.rem;
                //int row = i / image_width;
                //int col = i % image_width;

                
                T depth = imgptr[i];
                
                cv::Point2d pt(col, row);

                //if(depth>0)  //Only insert actual points (works for both float and uint16)
                {                        
                    cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                    cv::Point3f world_pnt = ray * (((float) depth)/scale);
                    cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                    
                    nx[i] = transformed_pnt.x;
                    ny[i] = transformed_pnt.y;
                    nz[i] = transformed_pnt.z;
                    
                    int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                    
                    float range_sq = worldToRangeSquared(transformed_pnt);
                    
                    //Only insert actual points (works for both float and uint16)
                    ranges[i] = (depth>0) ? range_sq : dNaN;    
                    inds[i] = cyl_idx;
                }                
            }
            
            for(int i = 0; i < num_pixels; ++i)
            {
                if(ranges[i]>0)
                {
                    cv::Point3f transformed_pnt(nx[i], ny[i], nz[i]);
                    
                    int cyl_idx = inds[i];
                            
                    if(cyl_idx >= 0  && cyl_idx < max_ind)
                    {
                        float range_sq = ranges[i];
                
                        cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                        
                        float prev_range_sq = worldToRangeSquared(prev_point);
                        
                        if(!(prev_range_sq <= range_sq)) //overwrite || 
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
                            float can_depth = worldToCanDepth(transformed_pnt);
                            
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_can_depth = worldToCanDepth(prev_point);
                            
                            if(!(prev_can_depth <= can_depth)) //overwrite || 
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
      
        //whole image vectorization
        template <typename T>
        void insertPoints3(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            const int image_width = image_size.width;
            const int image_height = image_size.height;
            const int num_pixels = image_width * image_height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
            
            AlignedVector<float> ranges(num_pixels, dNaN), nx(num_pixels, dNaN), ny(num_pixels, dNaN), nz(num_pixels, dNaN);
            AlignedVector<uint32_t> inds(num_pixels);
            
            const T* const imgptr = (T* const) image.data;
            
            const float row_factor = ((float) 1)/image_width;
            
            for(int i = 0; i < num_pixels; ++i)
            {
                float raw_row = i*row_factor;
                float row = std::floor(raw_row);
                float decimal = raw_row - row;
                float col = decimal * image_width;
                //auto res = std::div(i, image_width); int row = res.quot; int col = res.rem;
                //int row = i / image_width;
                //int col = i % image_width;

                
                T depth = imgptr[i];
                
                cv::Point2d pt(col, row);

                //if(depth>0)  //Only insert actual points (works for both float and uint16)
                {                        
                    cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                    cv::Point3f world_pnt = ray * (((float) depth)/scale);
                    cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                    
                    nx[i] = transformed_pnt.x;
                    ny[i] = transformed_pnt.y;
                    nz[i] = transformed_pnt.z;
                    
                    //int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                    
                    float range_sq = worldToRangeSquared(transformed_pnt);
                    
                    //Only insert actual points (works for both float and uint16)
                    ranges[i] = (depth>0) ? range_sq : dNaN;
                }                
            }
            
            for(int i = 0; i < num_pixels; ++i)
            {
                if(ranges[i]>0)
                {
                    cv::Point3f transformed_pnt(nx[i], ny[i], nz[i]);
                    
                    int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                            
                    if(cyl_idx >= 0  && cyl_idx < max_ind)
                    {
                        float range_sq = ranges[i];
                
                        cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                        
                        float prev_range_sq = worldToRangeSquared(prev_point);
                        
                        if(!(prev_range_sq <= range_sq)) //overwrite || 
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
                            float can_depth = worldToCanDepth(transformed_pnt);
                            
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_can_depth = worldToCanDepth(prev_point);
                            
                            if(!(prev_can_depth <= can_depth)) //overwrite || 
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
        
        //row-wise vectorization
        template <typename T>
        void insertPoints2(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform)
        {
            cv::Size image_size = cam_model.reducedResolution();
            const int image_width = image_size.width;
            const int image_height = image_size.height;
            const int num_pixels = image_width * image_height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();
            
            AlignedVector<float> ranges(num_pixels, dNaN), nx(num_pixels, dNaN), ny(num_pixels, dNaN), nz(num_pixels, dNaN);
            
            const T* const imgptr = (T* const) image.data;
            
            //for(int i = 0; i < num_pixels; ++i)
            //{
            int i = 0;
            for(int row = 0; row < image_height; ++row)
            {
                for(int col = 0; col < image_width; ++col)
                {                       
                    //auto res = std::div(i, image_width); int row = res.quot; int col = res.rem;
                    //int row = i / image_width;
                    //int col = i % image_width;
                    
                    //int row = i+10;
                    //int col = i + 30;
                    
                    T depth = imgptr[i];
                    
                    cv::Point2d pt(col, row);

                    {                        
                        cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                        cv::Point3f world_pnt = ray * (((float) depth)/scale);
                        cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                        
                        nx[i] = transformed_pnt.x;
                        ny[i] = transformed_pnt.y;
                        nz[i] = transformed_pnt.z;
                        
                        //int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                        
                        float range_sq = worldToRangeSquared(transformed_pnt);
                        
                        //Only insert actual points (works for both float and uint16)
                        ranges[i] = (depth>0) ? range_sq : dNaN;
                    }
                    i++;
                }
            }
            
            for(int i = 0; i < num_pixels; ++i)
            {
                if(ranges[i]>0)
                {
                    cv::Point3f transformed_pnt(nx[i], ny[i], nz[i]);
                    
                    int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                            
                    if(cyl_idx >= 0  && cyl_idx < max_ind)
                    {
                        float range_sq = ranges[i];
                
                        cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                        
                        float prev_range_sq = worldToRangeSquared(prev_point);
                        
                        if(!(prev_range_sq <= range_sq)) //overwrite || 
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
                            float can_depth = worldToCanDepth(transformed_pnt);
                            
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_can_depth = worldToCanDepth(prev_point);
                            
                            if(!(prev_can_depth <= can_depth)) //overwrite || 
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
      
        //immediate insert
        template <typename T>
        void insertPoints(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform)
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
                    
                    
                    if(depth>0)  //Only insert actual points (works for both float and uint16)
                    {                        
                        cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                        cv::Point3f world_pnt = ray * (((float) depth)/scale);
                        cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                        
                        int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                        
                        if(cyl_idx >= 0  && cyl_idx < max_ind)
                        {
                            float range_sq = worldToRangeSquared(transformed_pnt);
                    
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_range_sq = worldToRangeSquared(prev_point);
                            
                            if(!(prev_range_sq <= range_sq)) //overwrite || 
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
                                float can_depth = worldToCanDepth(transformed_pnt);
                                
                                cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                                
                                float prev_can_depth = worldToCanDepth(prev_point);
                                
                                if(!(prev_can_depth <= can_depth)) //overwrite || 
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
        void insertPoints(utils::ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform)
        {
            const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;

            if(image.depth() == CV_32FC1)
            {
                insertPoints4<float>(cylindrical_points, image, cam_model, transform);
            }
            else if (image.depth() == CV_16UC1)
            {
                insertPoints4<uint16_t>(cylindrical_points, image, cam_model, transform);
            }
        }

        
        
        DepthImageInserter::DepthImageInserter(tf2_ros::Buffer& buffer, ros::NodeHandle pnh):
            buffer_(buffer),
            pnh_(pnh)
        {}
        
        bool DepthImageInserter::init()
        {
            //TODO: use pips::param_utils
            bool status = pnh_.getParam("fixed_frame_id", fixed_frame_id_);

            return status;
        }
        
        bool DepthImageInserter::insert(ECWrapper& cylindrical_points, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
        {
            const std_msgs::Header& target_header = cylindrical_points.getHeader();
            const std_msgs::Header& source_header = image_msg->header;
            
            if(target_header == source_header)
            {
                ROS_INFO("Target and source headers match, using remapping approach");
                depth_remapper_.update(cylindrical_points, image_msg, cam_info);
                return true;
            }
            
            if( cam_model_.fromCameraInfo(cam_info) )
            {
                ROS_DEBUG("Camera info has changed!");
                //If camera info changed, update any precomputed values
                //cam_model_.init();
            }
            else
            {
                ROS_DEBUG("Camera info has not changed");
            }
            
            //Get transform
            geometry_msgs::TransformStamped transform;
            try
            {
                transform = buffer_.lookupTransform(target_header.frame_id, target_header.stamp, source_header.frame_id, source_header.stamp, fixed_frame_id_);
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
                return false;
            }
            
            insertPoints(cylindrical_points, image_msg, cam_model_, transform);

            return true;
        }


    } //end namespace utils
} //end namespace egocylindrical
