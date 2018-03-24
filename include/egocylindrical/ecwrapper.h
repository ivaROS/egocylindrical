#ifndef EGOCYLINDRICAL_ECWRAPPER_H
#define EGOCYLINDRICAL_ECWRAPPER_H



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
#include <sensor_msgs/PointCloud2.h>

#include <egocylindrical/EgoCylinderPoints.h>



namespace egocylindrical
{
    
    namespace utils
    {

        constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
        
        
        /*
         * This class is intended to act as an abstraction of the egocylindrical representation
         * to enable other functions to operate on it without requiring knowledge of the implementation.
         * Functionality will be moved incrementally.
         * It is hoped that it will be extended to simplify other egocylindrical versions, ex. stixel
         */
        
        class ECWrapper
        {
            cv::Mat points_;
            int height_, width_;
            float vfov_;
            
            std_msgs::Header header_;
            EgoCylinderPoints::Ptr msg_; // The idea would be to store everything in the message's allocated storage to prevent copies
            
            // For now, just get things working using this.
            // Note: It may be preferable to allocate x,y,z separately to ensure they are aligned (unless the width is chosen such that they will be anyway...)
            // Another idea: possibly template this class by height/width, potentially enabling compile time optimizations
            
            
        public:
            
            ECWrapper(int height, int width, float vfov):
            height_(height),
            width_(width),
            vfov_(vfov)
            {
                points_ = cv::Mat(3, height * width, CV_32FC1, utils::dNaN);    //TODO: Allocate space in the msg_ to avoid later copy
            }
            
            inline
            float* getPoints()
            {
                float* point_ptr = (float*)__builtin_assume_aligned(points_.data, 16);   //This can be updated for improved alignments, etc
                return point_ptr;
            }
            
            inline
            const float* getPoints() const
            {
                return getPoints();
            }
            
            inline
            float* getX()
            {
                return getPoints();
            }
            
            inline
            const float* getX() const
            {
                return getX();
            }
            
            inline
            float* getY()
            {
                return getPoints() + (height_ * width_);
            }
            
            inline
            const float* getY() const
            {
                return getY();
            }
            
            inline
            float* getZ()
            {
                return getPoints() + 2*(height_ * width_);
            }
            
            inline
            const float* getZ() const
            {
                return getZ();
            }
            
            inline
            void setHeader(std_msgs::Header header)
            {
                header_ = header;
            }
            
            inline
            int getCols() const
            {
                return points_.cols;
            }
            
            inline
            int getWidth() const
            {
                return width_;
            }
            
            inline
            int getHeight() const
            {
                return height_;
            }
            
            inline
            std_msgs::Header getHeader() const
            {
                return header_; 
            }
            
            inline
            cv::Rect getImageRoi() const
            {
                return cv::Rect(0, 0, width_, height_);
            }
            
            
            inline
            EgoCylinderPoints::ConstPtr getEgoCylinderPointsMsg()
            {
                /* This is a temporary solution until I can decide how to do this. Ideally, I would publish the final message directly
                 * so that other nodelets can do their thing. However, currently I transform points in place, overwriting the old values.
                 * The whole point of storing the data in a msgptr may be undermined by this. It will come down to whether it is faster to
                 * 1. Copy the message here before publishing
                 * 2. Publish the message, then make a copy for use in callback
                 * 3. Perform out of place point transformation
                 */
                EgoCylinderPoints::ConstPtr msg = boost::make_shared<EgoCylinderPoints>(*msg_);
                
                return (EgoCylinderPoints::ConstPtr) msg_;
            }
            
            // Function stubs to fill in and uncomment as needed
            /*
             *    inline
             *    float& x(int row, int col)
             *    {
             *        
        }
        
        inline
        float& y(int row, int col)
        {
        
        }
        
        inline
        float& z(int row, int col)
        {
        
        }
        
        inline
        const float& x(int row, int col) const
        {
        return (const float& ) x(row, col);
        }
        
        inline
        const float& y(int row, int col) const
        {
        return (const float& ) y(row, col);
        }
        
        inline
        const float& z(int row, int col) const
        {
        return (const float& ) z(row, col);
        }
        
        inline
        cv::Point3f at(int row, int col)
        {
        
        }
        
        */
            
            
        };
        
        typedef std::shared_ptr<ECWrapper> ECWrapperPtr;
        
        inline
        ECWrapperPtr getECWrapper(int height, int width, float vfov)
        {
            return std::make_shared<ECWrapper>(height,width,vfov);
        }
        
    }
}
    
#endif //EGOCYLINDRICAL_ECWRAPPER_H