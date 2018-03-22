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

/*
    inline
    float getDistance(const float* const cyl_ptr, const int j, const int num_cols ) 
    {
        float temp = std::sqrt(cyl_ptr[j]*cyl_ptr[j] + cyl_ptr[num_cols*2 + j]*cyl_ptr[num_cols*2 + j]);
        return temp;
    }
    
    inline
    void  generateRawRangeImage(const cv::Mat& cylindrical_history, cv::Mat& range_image)
    {
        cv::Rect image_roi(cv::Point(), cylindrical_history.size());
        
            
        ROS_DEBUG("Generating image of cylindrical memory");
        
        //float* r = range_image.ptr<float>(0);
        float* const r = (float *)range_image.data;
        
        const float* const cyl_ptr = (float *)cylindrical_history.data;
        int num_cols = cylindrical_history.cols;

        #pragma GCC ivdep
        for(int j = 0; j < num_cols; ++j)
        {
            float temp = 0;
            
            if(cyl_ptr[j]==cyl_ptr[j])
            {
                temp = std::sqrt(cyl_ptr[j]*cyl_ptr[j] + cyl_ptr[num_cols*2 + j]*cyl_ptr[num_cols*2 + j]);
                //temp = (cyl_ptr[j]*cyl_ptr[j] + cyl_ptr[num_cols*2 + j]*cyl_ptr[num_cols*2 + j]);
                //temp = getDistance(cyl_ptr, j, num_cols);
            }
            r[j] = temp;
            
        }
                
    }
    
   
int main(int argc, char** argv)
{
    cv::Mat temp = cv::Mat(3,2048*320,CV_32FC1, std::numeric_limits<float>::quiet_NaN());
    temp.colRange(0,500).setTo(0);
    
    
    cv::Mat im = cv::Mat(1,320*2048,CV_32FC1,std::numeric_limits<float>::quiet_NaN());
    
    generateRawRangeImage(temp,im);
    
    im = im.reshape(320,2048);
    
    cv::imshow("hi", im);
    cv::waitKey(0);
}
*/

int main(int argc, char** argv)
{
    float bad = std::numeric_limits<float>::quiet_NaN();
    float good = .45;
    
    
    
}