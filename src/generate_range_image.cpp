#include <egocylindrical/ecwrapper.h>

#include <egocylindrical/utils.h>

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
    
        void generateRawRangeImage(const utils::ECWrapper& cylindrical_history, cv::Mat& range_image)
        {
            //cv::Rect image_roi = cylindrical_history.getImageRoi();
            
                
            ROS_DEBUG("Generating image of cylindrical memory");
            
            float* const r = (float *)__builtin_assume_aligned(range_image.data, 16);
            
            const float* const cyl_ptr = (float *)__builtin_assume_aligned(cylindrical_history.getPoints(), 16);
            int num_cols = cylindrical_history.getCols();

            #pragma GCC ivdep
            for(int j = 0; j < num_cols; ++j)
            {
                float temp = dNaN;
                
                if(cyl_ptr[j]==cyl_ptr[j])
                {
                    temp = std::sqrt(cyl_ptr[j]*cyl_ptr[j] + cyl_ptr[num_cols*2 + j]*cyl_ptr[num_cols*2 + j]);
                }
                r[j] = temp;
                
            }
                    
        }
        
        
        cv::Mat getRawRangeImage(const utils::ECWrapper& cylindrical_history)
        {
            cv::Mat range_image(cylindrical_history.getHeight(), cylindrical_history.getWidth(), CV_32FC1);   
            generateRawRangeImage(cylindrical_history, range_image);
            return range_image;
        }

        cv::Mat getImageMat(const sensor_msgs::Image& image)
        {
            cv::Mat im = cv::Mat(image.height, image.width, CV_32FC1, const_cast<uchar*>(&image.data[0]), image.step);
            return im;
        }
        
        cv::Mat getImageMat(const sensor_msgs::ImagePtr& image)
        {
            if(image != nullptr)
            {
                return getImageMat(*image);
            }
            else
            {
                ROS_ERROR("ImagePtr is null!");
                return cv::Mat();
            }
            
        }
        
        sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history, const CylindricalCoordsConverter& ccc)
        {
            sensor_msgs::ImagePtr new_msg_ptr = boost::make_shared<sensor_msgs::Image>();
            
            sensor_msgs::Image &new_msg = *new_msg_ptr;
            new_msg.height = ccc.height;
            new_msg.width = ccc.width;
            new_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            new_msg.is_bigendian = false; //image->is_bigendian;
            new_msg.step = ccc.width * sizeof(float); // cylindrical_history.elemSize(); // Ideally, replace this with some other way of getting size
            size_t size = new_msg.step * ccc.height;
            new_msg.data.resize(size);
            //cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::TYPE_32FC1, new_im_).toImageMsg();
            
            cv::Mat range_image = getImageMat(new_msg);
            
            generateRawRangeImage(cylindrical_history, range_image);
            range_image = range_image.reshape(1, ccc.height);
            
            return new_msg_ptr;
        }
        
    }
}