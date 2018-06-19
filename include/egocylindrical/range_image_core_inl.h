#ifndef RANGE_IMAGE_CORE_INL_H
#define RANGE_IMAGE_CORE_INL_H

#include <egocylindrical/ecwrapper.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <stdint.h>


namespace egocylindrical
{
    
    namespace utils
    {
        template <typename T,uint scale>
        void generateRangeImage(const utils::ECWrapper& cylindrical_history, T* r, const T unknown_val, int num_threads)
        {
            //cv::Rect image_roi = cylindrical_history.getImageRoi();
            
                
            ROS_DEBUG("Generating image of cylindrical memory");
            
            
            const float* const cyl_ptr = (float *)__builtin_assume_aligned(cylindrical_history.getPoints(), 16);
            int num_cols = cylindrical_history.getCols();

            #pragma GCC ivdep
            for(int j = 0; j < num_cols; ++j)
            {
                T temp = unknown_val;
                
                if(cyl_ptr[j]==cyl_ptr[j])
                {
                    temp = std::sqrt(cyl_ptr[j]*cyl_ptr[j] + cyl_ptr[num_cols*2 + j]*cyl_ptr[num_cols*2 + j]) * scale;
                }
                r[j] = temp;
                
            }
                    
        }
        
        
        template <typename T, uint scale>
        sensor_msgs::ImagePtr generateRangeImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, const T unknown_val, int num_threads)
        {
            sensor_msgs::ImagePtr new_msg_ptr = boost::make_shared<sensor_msgs::Image>();
            
            sensor_msgs::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = cylindrical_history.getHeight();
            new_msg.width = cylindrical_history.getWidth();
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false; //image->is_bigendian;
            new_msg.step = cylindrical_history.getWidth() * sizeof(T); //sensor_msgs::image_encodings::bitDepth(encoding); // cylindrical_history.elemSize(); // Ideally, replace this with some other way of getting size
            size_t size = new_msg.step * cylindrical_history.getHeight();
            new_msg.data.resize(size);
            //cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::TYPE_32FC1, new_im_).toImageMsg();
            
            T* data = (T*)new_msg.data.data();
            
            generateRangeImage<T, scale>(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        
        template <typename T>
        sensor_msgs::ImagePtr generateRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads);

        
        template <> 
        sensor_msgs::ImagePtr generateRangeImageMsg<float>(const utils::ECWrapper& cylindrical_history, int num_threads)
        {
            return generateRangeImageMsg<float, 1>(cylindrical_history, sensor_msgs::image_encodings::TYPE_32FC1, dNaN, num_threads);
        }
        
        template <>
        sensor_msgs::ImagePtr generateRangeImageMsg<uint16_t>(const utils::ECWrapper& cylindrical_history, int num_threads)
        {
            return generateRangeImageMsg<uint16_t, 1000>(cylindrical_history, sensor_msgs::image_encodings::TYPE_16UC1, 0, num_threads);
        }

        
    }
}

#endif //RANGE_IMAGE_CORE_INL_H
