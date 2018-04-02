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

//#include <eigen_stl_containers/eigen_stl_containers.h>
#include <boost/align/aligned_allocator.hpp>

#include <cstddef>
#include <cstdalign>

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
        float worldToRangeSquared(const float x, const float z)
        {
            return x*x + z*z;
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
        
        
        
        //typedef ::egocylindrical::EgoCylinderPoints_<Eigen::aligned_allocator<void, 32> > AlignedEgoCylinderPoints;
        typedef ::egocylindrical::EgoCylinderPoints_<boost::alignment::aligned_allocator<void, 32> > AlignedEgoCylinderPoints;
        
        
        //typedef AlignedEgoCylinderPoints ECMsg;
        typedef EgoCylinderPoints ECMsg;
        typedef boost::shared_ptr<ECMsg> ECMsgPtr;
        typedef boost::shared_ptr<ECMsg const> ECMsgConstPtr;
        
        
        /*
         * This class is intended to act as an abstraction of the egocylindrical representation
         * to enable other functions to operate on it without requiring knowledge of the implementation.
         * Functionality will be moved incrementally.
         * It is hoped that it will be extended to simplify other egocylindrical versions, ex. stixel
         */
        
        class ECWrapper
        {
        private:
            
            //cv::Mat points_;
            
            float* points_;
            
            float* ranges_=nullptr;
            long int* inds_=nullptr; // Note: on 32/64 bit systems, int almost always has the same size as long, but just to be safe...
            
            float* aligned_ranges_ = nullptr;
            long int* aligned_inds_ = nullptr;
            
            int height_, width_;
            float vfov_;
            float hscale_, vscale_;
            
            std_msgs::Header header_;
            ECMsgPtr msg_; // The idea would be to store everything in the message's allocated storage to prevent copies
            
            ECMsgConstPtr const_msg_;
            
            // For now, just get things working using this.
            // Note: It may be preferable to allocate x,y,z separately to ensure they are aligned (unless the width is chosen such that they will be anyway...)
            // Another idea: possibly template this class by height/width, potentially enabling compile time optimizations
            // Also: maybe should store x, then z, then y, since only x and z are needed for range image
           

           
        public:
            
            ECWrapper(int height, int width, float vfov, bool allocate_arrays = false):
            height_(height),
            width_(width),
            vfov_(vfov)
            {
                msg_ = boost::make_shared<ECMsg>();
                
                //Eigen::aligned_allocator<EgoCylinderPoints> Alloc;
                //msg_ = boost::allocate_shared<EgoCylinderPoints>(Alloc);
                
                int max_alignment = alignof(std::max_align_t);
                
                int biggest_alignment = __BIGGEST_ALIGNMENT__;
                
                size_t object_size = sizeof(float);
                
                size_t object_alignment = alignof(float);
                
                size_t buffer_size = biggest_alignment - object_size;
                size_t buffer_objects = buffer_size / object_size;
                
                // NOTE: width x height must be divisible by 8 for this approach to work. Otherwise, additional information will be necessary in order to properly place the x,y,z pointers
                
                ROS_INFO_STREAM("max_alignment: " << max_alignment << ", biggest_alignment: " << biggest_alignment << ", object_size: " << object_size << ", object_alignment: " << object_alignment << ", number buffer objects: " << buffer_objects);

                msg_->points.data.resize(3*height_*width_ + buffer_objects, dNaN);  //Note: can pass 'utils::dNaN as 2nd argument to set all values
                
                
                points_ = msg_->points.data.data();
                
                // TODO: create templated function to get aligned pointers. Something similar here: http://en.cppreference.com/w/cpp/memory/align
                /*
                {
                    void* temp_points = (void*) msg_->points.data.data();
                    
                    size_t space_before = height_*width_*3*sizeof(float);
                    size_t space_after = space_before;
                    
                    std::align(biggest_alignment, sizeof(float), temp_points, space_after);
                    pointsx_ = (float*) temp_points;
                    
                    msg_->points.layout.data_offset = (space_before - space_after);
                    
                    
                    ROS_INFO_STREAM("Aligned pointsx_, adjusted pointer by " << (space_before - space_after) << " bytes");
                }
                */
                
                
                if(allocate_arrays)
                {
                    {
                        ranges_ = new float[height_*width_ + buffer_objects];
                        aligned_ranges_ = ranges_;
                        
                        /*
                        void* temp_range = (void*) ranges_;
                        //aligned_ranges_ = ranges_;
                        
                        size_t space_before = height_*width_*sizeof(float);
                        size_t space_after = space_before;
                        
                        std::align(biggest_alignment, sizeof(float), temp_range, space_after);
                        aligned_ranges_ = (float*) temp_range;
                        
                        ROS_INFO_STREAM("Aligned ranges_, adjusted pointer by " << (space_before - space_after) << " bytes");
                        */
                    }
                    
                    {
                        inds_ = new long int[height_*width_ + (biggest_alignment / sizeof(long int)) - 1];
                        aligned_inds_ = inds_;
                        
                        /*
                        void* temp_inds = (void*) inds_;
                        
                        size_t space_before = height_*width_*sizeof(long int);
                        size_t space_after = space_before;
                        
                        std::align(biggest_alignment, sizeof(long int), temp_inds, space_after);
                        aligned_inds_ = (long int*) temp_inds;
                        
                        ROS_INFO_STREAM("Aligned inds_, adjusted pointer by " << (space_before - space_after) << " bytes");
                        */
                    }
                }
                
                msg_->fov_v = vfov_;
                
                hscale_ = width_/(2*M_PI);
                vscale_ = height_/vfov;
                
                
                std::vector<std_msgs::MultiArrayDimension>& dims = msg_->points.layout.dim;
                dims.resize(3);
                
                
                std_msgs::MultiArrayDimension& dim0 = dims[0];
                dim0.label = "components";
                dim0.size = 3;
                dim0.stride = 3*height_*width_;
                //dims[0] = dim0;
                
                
                std_msgs::MultiArrayDimension& dim1 = dims[1];
                dim1.label = "rows";
                dim1.size = height_;
                dim1.stride = height_*width_;
                //dims[1] = dim1;

                
                std_msgs::MultiArrayDimension& dim2 = dims[2];
                dim2.label = "point";
                dim2.size = width_;
                dim2.stride = width_;             
                //dims[2] = dim2;
                
                
                int step = dim1.stride * sizeof(float);
                
                //points_ = cv::Mat(3, height_ * width_, CV_32FC1, const_cast<float*>(msg_->points.data.data()), step);
                
                
                //std::cout << "Address: " << std::hex  << msg_->points.data.data() << std::dec << ", height=" << height_ << ", width=" << width_ << ", step=" << step << std::endl; //std::setfill('0') << std::setw(2) << ar[i] << " ";
                

                //points_.setTo(utils::dNaN);
            }
            
            ECWrapper(const ECMsgConstPtr& ec_points) 
            {
                const_msg_ = ec_points;
                header_ = const_msg_->header;
                vfov_ = const_msg_->fov_v;
                
                const std::vector<std_msgs::MultiArrayDimension>& dims = const_msg_->points.layout.dim;
                int components = dims[0].size;
                height_ = dims[1].size;
                width_ = dims[2].size;
                
                int step = dims[1].stride * sizeof(float);
                
                //points_ = cv::Mat(components, height_ * width_, CV_32FC1, const_cast<float*>(const_msg_->points.data.data()), step);
                
                points_ = (float*) const_msg_->points.data.data();// + (const_msg_->points.layout.data_offset) / sizeof(float);
                
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
            
                        
            inline float* getPoints()                   { return (float*) points_; }
            inline const float* getPoints()     const   { return (const float*) points_; }
            
            inline float* getX()                        { return getPoints(); }
            inline const float* getX()          const   { return (const float*) getPoints(); }
            
            inline float* getY()                        { return getPoints() + (height_ * width_); }
            inline const float* getY()          const   { return (const float*) getPoints() + (height_ * width_); }
            
            inline float* getZ()                        { return getPoints() + 2*(height_ * width_); }
            inline const float* getZ()          const   { return (const float*) getPoints() + 2*(height_ * width_); }
            
            inline float* getRanges()                   { return aligned_ranges_; }
            inline const float* getRanges()     const   { return (const float*) aligned_ranges_; }
            
            inline long int* getInds()                  { return aligned_inds_; }
            inline const long int* getInds()    const   { return (const long int*) aligned_inds_; }
            

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
            int getNumPts() const
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
            cv::Point worldToCylindricalImage(const cv::Point3f& point) const
            {
                return utils::worldToCylindricalImage(point, width_, height_, hscale_, vscale_, 0, 0);
            }
            
            
            
            inline
            int worldToCylindricalIdx(float x, float y, float z) const
            {
                cv::Point image_pnt = worldToCylindricalImage(cv::Point3f(x,y,z));
                
                int tidx = image_pnt.y * getWidth() +image_pnt.x;
                
                return tidx;
            }
            
            
            inline
            ECMsgConstPtr getEgoCylinderPointsMsg()
            {
                /* This is a temporary solution until I can decide how to do this. Ideally, I would publish the final message directly
                 * so that other nodelets can do their thing. However, currently I transform points in place, overwriting the old values.
                 * The whole point of storing the data in a msgptr may be undermined by this. It will come down to whether it is faster to
                 * 1. Copy the message here before publishing
                 * 2. Publish the message, then make a copy for use in callback
                 * 3. Perform out of place point transformation
                 */
                ECMsgConstPtr msg = boost::make_shared<EgoCylinderPoints>(*msg_);
                
                return (ECMsgConstPtr) msg;
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
        ECWrapperPtr getECWrapper(int height, int width, float vfov, bool allocate_arrays=false)
        {
            return std::make_shared<ECWrapper>(height,width,vfov,allocate_arrays);
        }
        
    }
}
    
#endif //EGOCYLINDRICAL_ECWRAPPER_H