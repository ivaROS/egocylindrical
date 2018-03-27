#ifndef EGOCYLINDRICAL_ECWRAPPER_H
#define EGOCYLINDRICAL_ECWRAPPER_H



#include <ros/ros.h>
#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf/LinearMath/Matrix3x3.h>
//#include <omp.h>
//#include <sensor_msgs/PointCloud2.h>

#include <egocylindrical/EgoCylinderPoints.h>

//#include <iomanip> // for debug printing


namespace egocylindrical
{
    
    namespace utils
    {

        constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
        
        // Source: https://gist.github.com/volkansalma/2972237
        inline
        float atan2_approximation1(float y, float x)
        {
            //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
            //Volkan SALMA
            
            constexpr float ONEQTR_PI = M_PI / 4.0;
            constexpr float THRQTR_PI = 3.0 * M_PI / 4.0;
            float r, angle;
            float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
            if ( x < 0.0f )
            {
                r = (x + abs_y) / (abs_y - x);
                angle = THRQTR_PI;
            }
            else
            {
                r = (x - abs_y) / (x + abs_y);
                angle = ONEQTR_PI;
            }
            angle += (0.1963f * r * r - 0.9817f) * r;
            if ( y < 0.0f )
                return( -angle );     // negate if in quad III or IV
                else
                    return( angle );
                
                
        }
        
        
        
        inline
        cv::Point3f projectWorldToCylinder(const cv::Point3f& point)
        {
            cv::Point3f Pcyl_t = point / std::sqrt(point.x * point.x + point.z * point.z);
            return Pcyl_t;
        }
        
        
        inline
        cv::Point worldToCylindricalImage(const cv::Point3f& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
        {
            
            cv::Point3f p_cyl = projectWorldToCylinder(point);
            
            float x = atan2_approximation1(p_cyl.x, p_cyl.z) * h_scale + cyl_width / 2;
            //float x = std::atan2(p_cyl.x, p_cyl.z) * h_scale + cyl_width / 2;
            float y = p_cyl.y * v_scale + cyl_height / 2;
            
            cv::Point im_pt(x,y);
            return im_pt;
        }
        
        inline
        float worldToRangeSquared(const cv::Point3f& point)
        {
            return point.x*point.x + point.z*point.z;
        }
        
        inline
        float worldToRange(const cv::Point3f& point)
        {
            return std::sqrt(worldToRangeSquared(point));
        }
        
        /*
         * This class is intended to act as an abstraction of the egocylindrical representation
         * to enable other functions to operate on it without requiring knowledge of the implementation.
         * Functionality will be moved incrementally.
         * It is hoped that it will be extended to simplify other egocylindrical versions, ex. stixel
         */
        
        class ECWrapper
        {
        private:
            
            cv::Mat points_;
            
            float* pointsx_;
            
            int height_, width_;
            float vfov_;
            float hscale_, vscale_;
            
            std_msgs::Header header_;
            EgoCylinderPoints::Ptr msg_; // The idea would be to store everything in the message's allocated storage to prevent copies
            
            EgoCylinderPoints::ConstPtr const_msg_;
            
            // For now, just get things working using this.
            // Note: It may be preferable to allocate x,y,z separately to ensure they are aligned (unless the width is chosen such that they will be anyway...)
            // Another idea: possibly template this class by height/width, potentially enabling compile time optimizations
            // Also: maybe should store x, then z, then y, since only x and z are needed for range image
           

           
        public:
            
            float* ranges_=nullptr;
            long int* inds_=nullptr; // Note: on 32/64 bit systems, int almost always has the same size as long, but just to be safe...
            
            ECWrapper(int height, int width, float vfov):
            height_(height),
            width_(width),
            vfov_(vfov)
            {
                msg_ = boost::make_shared<EgoCylinderPoints>();
                msg_->points.data.resize(3*height_*width_);  //Note: can pass 'utils::dNaN as 2nd argument to set all values
                
                
                ranges_ = new float[height_*width_];
                inds_ = new long int[height_*width_];
                
                msg_->fov_v = vfov_;
                
                hscale_ = width_/(2*M_PI);
                vscale_ = height_/vfov;
                
                std::vector<std_msgs::MultiArrayDimension>& dims = msg_->points.layout.dim;
                dims.resize(3);
                
                
                std_msgs::MultiArrayDimension dim0;
                dim0.label = "components";
                dim0.size = 3;
                dim0.stride = 3*height_*width_;
                dims[0] = dim0;
                
                
                std_msgs::MultiArrayDimension dim1;
                dim1.label = "rows";
                dim1.size = height_;
                dim1.stride = height_*width_;
                dims[1] = dim1;

                
                std_msgs::MultiArrayDimension dim2;
                dim2.label = "point";
                dim2.size = width_;
                dim2.stride = width_;             
                dims[2] = dim2;
                
                
                int step = dim1.stride * sizeof(float);
                
                points_ = cv::Mat(3, height_ * width_, CV_32FC1, const_cast<float*>(msg_->points.data.data()), step);
                
                
                //std::cout << "Address: " << std::hex  << msg_->points.data.data() << std::dec << ", height=" << height_ << ", width=" << width_ << ", step=" << step << std::endl; //std::setfill('0') << std::setw(2) << ar[i] << " ";
                

                points_.setTo(utils::dNaN);
            }
            
            ECWrapper(const egocylindrical::EgoCylinderPointsConstPtr& ec_points) 
            {
                const_msg_ = ec_points;
                header_ = const_msg_->header;
                vfov_ = const_msg_->fov_v;
                
                const std::vector<std_msgs::MultiArrayDimension>& dims = const_msg_->points.layout.dim;
                int components = dims[0].size;
                height_ = dims[1].size;
                width_ = dims[2].size;
                
                int step = dims[1].stride * sizeof(float);
                
                points_ = cv::Mat(components, height_ * width_, CV_32FC1, const_cast<float*>(const_msg_->points.data.data()), step);
                
                //std::cout << "Address: " << std::hex  << const_msg_->points.data.data() << std::dec << ", height=" << height_ << ", width=" << width_ << ", step=" << step << std::endl;
                
            }
            
            ~ECWrapper()
            {
                
                   if(inds_ != nullptr)
                   {
                       delete inds_;
                       delete ranges_;
                   }
            }
            
            
            
            
            inline float* getPoints()                   { return (float*)__builtin_assume_aligned(points_.data, 16); }
            inline const float* getPoints()     const   { return (const float*)__builtin_assume_aligned(points_.data, 16); }
            
            inline float* getX()                        { return getPoints(); }
            inline const float* getX()          const   { return (const float*) getPoints(); }
            
            inline float* getY()                        { return getPoints() + (height_ * width_); }
            inline const float* getY()          const   { return (const float*) getPoints() + (height_ * width_); }
            
            inline float* getZ()                        { return getPoints() + 2*(height_ * width_); }
            inline const float* getZ()          const   { return (const float*) getPoints() + 2*(height_ * width_); }
            

            inline
            void setHeader(std_msgs::Header header)
            {
                header_ = header;
                msg_->header = header;
            }
            
            inline
            int getCols() const
            {
                return height_*width_;
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
            cv::Point worldToCylindricalImage(cv::Point3f point) const
            {
                return utils::worldToCylindricalImage(point, width_, height_, hscale_, vscale_, 0, 0);
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