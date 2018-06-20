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

#include <boost/align/aligned_alloc.hpp>
#include <boost/align/aligned_allocator.hpp>

#include <cstddef>
#include <cstdalign>

//#include <iomanip> // for debug printing


namespace egocylindrical
{
    
    namespace utils
    {

        constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
        
        
        //Source: https://stackoverflow.com/a/39714493/2906021
        inline
        float inverse_approximation(float x)
        {
            union {
                float dbl;
                unsigned uint;
            } u;
            u.dbl = x;
            u.uint = ( 0xbe6eb3beU - u.uint ) >> (unsigned char)1;
            // pow( x, -0.5 )
            u.dbl *= u.dbl;                 // pow( pow(x,-0.5), 2 ) = pow( x, -1 ) = 1.0 / x
            return u.dbl;
        }
        
        // TODO: Quantify max error and verify that it doesn't affect anything
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
        
        //https://github.com/andrepuschmann/math-neon/blob/master/src/math_sqrtf.c
        inline
        float inv_sqrt_approximation(float x)
        {
            float b, c;
            
            union {
                float 	f;
                int 	i;
            } a;
            
            //fast invsqrt approx
            a.f = x;
            a.i = 0x5F3759DF - (a.i >> 1);		//VRSQRTE
            c = x * a.f;
            b = (3.0f - c * a.f) * 0.5;		//VRSQRTS
            a.f = a.f * b;		
            c = x * a.f;
            b = (3.0f - c * a.f) * 0.5;
            a.f = a.f * b;
            
            return a.f;
        }
        
        
        /* Note: functions using 'std::sqrt' must be compiled with '-fno-math-errno' in order to be vectorized.
         * See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=51890 for explanation.
         */
        template <typename T>
        inline
        cv::Point3_<T> projectWorldToCylinder(const cv::Point3_<T>& point)
        {
          cv::Point3_<T> Pcyl_t = point / std::sqrt(point.x * point.x + point.z * point.z);
          return Pcyl_t;
        }
        
        /*
        inline
        cv::Point3f projectWorldToCylinder(const cv::Point3f& point)
        {
            cv::Point3f Pcyl_t = point / std::sqrt(point.x * point.x + point.z * point.z);
            return Pcyl_t;
        }
        */
        
        template <typename T>
        inline
        cv::Point_<T> worldToCylindricalImage(const cv::Point3_<T>& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
        {
            
            cv::Point3_<T> p_cyl = projectWorldToCylinder(point);
            
            T x = std::atan2(p_cyl.x, p_cyl.z) * h_scale + cyl_width / 2;
            T y = p_cyl.y * v_scale + cyl_height / 2;
            
            cv::Point_<T> im_pt(x,y);
            return im_pt;
        }
        
        template <typename T>
        inline
        cv::Point worldToCylindricalImageFast(const cv::Point3_<T>& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
        {
          
          cv::Point3_<T> p_cyl = projectWorldToCylinder(point);
          
          T x = atan2_approximation1(p_cyl.x, p_cyl.z) * h_scale + cyl_width / 2;

          T y = p_cyl.y * v_scale + cyl_height / 2;
          
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
            return worldToRangeSquared(point.x, point.z);
        }
        
        inline
        float worldToRange(const cv::Point3f& point)
        {
            return std::sqrt(worldToRangeSquared(point));
        }
        
        

        
        
        //typedef ::egocylindrical::EgoCylinderPoints_<Eigen::aligned_allocator<void, 32> > AlignedEgoCylinderPoints;
        typedef ::egocylindrical::EgoCylinderPoints_<boost::alignment::aligned_allocator<void, 32> > AlignedEgoCylinderPoints;
        
        // NOTE: I'm not sure that using this typedef renamed version was such a good idea after all...
        //typedef AlignedEgoCylinderPoints ECMsg;
        typedef EgoCylinderPoints ECMsg;
        typedef boost::shared_ptr<ECMsg> ECMsgPtr;
        typedef boost::shared_ptr<ECMsg const> ECMsgConstPtr;
        
        template <typename T>
        using AlignedVector = std::vector<T, boost::alignment::aligned_allocator<T, __BIGGEST_ALIGNMENT__> >;
        
        
        
        
        
        
        struct ECParams
        {
          int height, width;
          float vfov;
          
          bool operator==(const ECParams &other) const 
          {
            return (height==other.height && width==other.width && vfov==other.vfov);
          }
        };
        
        
        
        
        
        // TODO: pull the relevant egocylindrical 'camera' info into separate message and use this class as the interpreter of it
        // ECWrapper would also use this class
        class ECConverter
        {
        private:
          int height_, width_;
          float vfov_;
          float hscale_, vscale_;
          
        public:
          ECConverter()
          {
          }
          
          
          inline
          void update()
          {
            hscale_ = width_/(2*M_PI);
            vscale_ = height_/vfov_;
          }
          
          inline
          void fromCameraInfo(const ECMsgConstPtr& msg)
          {
            
            //header_ = msg->header;
            vfov_ = msg->fov_v;
            
            const std::vector<std_msgs::MultiArrayDimension>& dims = msg->points.layout.dim;
            height_ = dims[1].size;
            width_ = dims[2].size;
            
            update();
          }
          
          inline
          void fromParams(const ECParams& params)
          {
            vfov_ = params.vfov;
            height_ = params.height;
            width_ = params.width;
            
            update();
          }
          
          inline
          cv::Point worldToCylindricalImage(const cv::Point3f& point) const
          {
            return utils::worldToCylindricalImage(point, width_, height_, hscale_, vscale_, 0, 0);
          }
          
          inline
          int getHeight() const
          {
            return height_;
          }
          
          inline
          int getWidth() const
          {
            return width_;
          }
          
          inline
          int getNumPts() const
          {
            return height_*width_;
          }

          template <typename S, typename T>
          inline 
          void project3dToPixel(const cv::Point3_<S> point, cv::Point_<T>& pixel) const
          {
            utils::worldToCylindricalImage(point, pixel, width_, height_, hscale_, vscale_, 0, 0);
          }
          
          
          template <typename T>
          inline 
          cv::Point_<T> project3dToPixel(const cv::Point3_<T> point) const
          {
            return utils::worldToCylindricalImage(point, width_, height_, hscale_, vscale_, 0, 0);
          }
          
          
          inline
          cv::Point3d projectPixelTo3dRay(const cv::Point2d& point) const
          {
            cv::Point3d ray;
            double theta = (point.x - (width_/2))/hscale_;
            
            ray.x = sin(theta);
            ray.z = cos(theta);
            
            ray.y = (point.y - (height_/2))/vscale_;
            
            ray /= (ray.x*ray.x + ray.z*ray.z);
            return ray;
          }
          
        };
        
        
        

        
        /*
         * This class is intended to act as an abstraction of the egocylindrical representation
         * to enable other functions to operate on it without requiring knowledge of the implementation.
         * Functionality will be moved incrementally.
         * It is hoped that it will be extended to simplify other egocylindrical versions, ex. stixel
         */
        
        // TODO: create an abstract class to serve as interface so that different storage mechanisms can be used on the backend
        class ECWrapper
        {
        private:
            
            //cv::Mat points_;
            
            float* points_;
            
            //void* ranges_=nullptr;
            AlignedVector<float> ranges_;
            
            //long int* inds_=nullptr; // Note: on 32/64 bit systems, int almost always has the same size as long, but just to be safe...
            
            AlignedVector<long int> inds_;

            
            float* aligned_ranges_ = nullptr;
            long int* aligned_inds_ = nullptr;
            
            int height_, width_;
            float vfov_;
            float hscale_, vscale_;
            
            bool allocate_arrays_;
            
            std_msgs::Header header_;
            ECMsgPtr msg_; // The idea would be to store everything in the message's allocated storage to prevent copies
            
            ECMsgConstPtr const_msg_;
            
            bool msg_locked_;
            
            // For now, just get things working using this.
            // Note: It may be preferable to allocate x,y,z separately to ensure they are aligned (unless the width is chosen such that they will be anyway...)
            // Another idea: possibly template this class by height/width, potentially enabling compile time optimizations
            // Also: maybe should store x, then z, then y, since only x and z are needed for range image
           

           
        public:
            
            ECWrapper(int height, int width, float vfov, bool allocate_arrays = false):
                height_(height),
                width_(width),
                vfov_(vfov),
                allocate_arrays_(allocate_arrays)
            {
                msg_ = boost::make_shared<ECMsg>();
                
                msg_locked_ = false;
                
                init();
            }
            
            ECWrapper(const ECMsgConstPtr& ec_points) 
            {
                const_msg_ = ec_points;
                header_ = const_msg_->header;
                vfov_ = const_msg_->fov_v;
                
                const std::vector<std_msgs::MultiArrayDimension>& dims = const_msg_->points.layout.dim;
                height_ = dims[1].size;
                width_ = dims[2].size;
                
                points_ = (float*) const_msg_->points.data.data() + (const_msg_->points.layout.data_offset) / sizeof(float);
                                
                msg_locked_ = true;
                
            }
            
            ~ECWrapper()
            {
                /*
                if(ranges_ != nullptr)
                   {
                       //delete inds_;
                       //delete ranges_;
                       boost::alignment::aligned_free(ranges_);
                       
                   }
                   */
            }

            inline float* getPoints()                   { return (float*) points_; }
            inline const float* getPoints()     const   { return (const float*) points_; }
            
            inline float* getX()                        { return getPoints(); }
            inline const float* getX()          const   { return (const float*) getPoints(); }
            
            inline float* getY()                        { return getPoints() + (height_ * width_); }
            inline const float* getY()          const   { return (const float*) getPoints() + (height_ * width_); }
            
            inline float* getZ()                        { return getPoints() + 2*(height_ * width_); }
            inline const float* getZ()          const   { return (const float*) getPoints() + 2*(height_ * width_); }
            
            inline float* getRanges()                   { return (float*) ranges_.data(); }
            inline const float* getRanges()     const   { return (const float*) ranges_.data(); }
            
            inline long int* getInds()                  { return inds_.data(); }
            inline const long int* getInds()    const   { return (const long int*) inds_.data(); }
            
            inline bool isLocked()              const   { return msg_locked_; }

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
                cv::Point3f point(x,y,z);
                cv::Point image_pnt = utils::worldToCylindricalImageFast(point, width_, height_, hscale_, vscale_, 0, 0);
                
                int tidx = image_pnt.y * getWidth() +image_pnt.x;
                
                return tidx;
            }
            
            inline
            int worldToCylindricalXIdx(float x, float z) const
            {
                int xind = atan2_approximation1(x,z)*hscale_ + width_/2;
                return xind;
            }
            
            inline
            int worldToCylindricalYIdx(float y, float range_squared) const
            {
                int yind = y * inv_sqrt_approximation(range_squared)*vscale_ + hscale_/2;
                return yind;
            }
            
            
            inline
            ECMsgConstPtr getEgoCylinderPointsMsg()
            {
                msg_locked_ = true;
                return (ECMsgConstPtr) msg_;
            }
            
            inline
            ECParams getParams() const
            {
                ECParams params;
                params.height=height_;
                params.width=width_;
                params.vfov=vfov_;
                
                return params;
            }
            
            inline
            void init()
            {
                int max_alignment = alignof(std::max_align_t);
                
                int biggest_alignment = __BIGGEST_ALIGNMENT__;
                
                size_t object_size = sizeof(float);
                
                size_t object_alignment = alignof(float);
                
                size_t buffer_size = biggest_alignment - object_size;
                size_t buffer_objects = buffer_size / object_size;
                
                // NOTE: width x height must be divisible by 8 for this approach to work. Otherwise, additional information will be necessary in order to properly place the x,y,z pointers
                
                ROS_DEBUG_STREAM("max_alignment: " << max_alignment << ", biggest_alignment: " << biggest_alignment << ", object_size: " << object_size << ", object_alignment: " << object_alignment << ", number buffer objects: " << buffer_objects);
                
                msg_->points.data.resize(3*height_*width_ + buffer_objects, dNaN);
                
                
                //Align data pointer
                {
                    void* temp_points = (void*) msg_->points.data.data();
                    
                    size_t space_before = height_*width_*3*sizeof(float);
                    size_t space_after = space_before;
                    
                    std::align(biggest_alignment, sizeof(float), temp_points, space_after);
                    points_ = (float*) temp_points;
                    
                    msg_->points.layout.data_offset = (space_before - space_after);
                    
                    ROS_DEBUG_STREAM("Aligned points_, adjusted pointer by " << (space_before - space_after) << " bytes");
                }
                
                if(allocate_arrays_)
                {
                    ranges_.resize(height_*width_);
                    inds_.resize(height_*width_); 
                }
                
                msg_->fov_v = vfov_;
                
                hscale_ = width_/(2*M_PI);
                vscale_ = height_/vfov_;
                
                
                std::vector<std_msgs::MultiArrayDimension>& dims = msg_->points.layout.dim;
                dims.resize(3);
                
                std_msgs::MultiArrayDimension& dim0 = dims[0];
                dim0.label = "components";
                dim0.size = 3;
                dim0.stride = 3*height_*width_;                
                
                std_msgs::MultiArrayDimension& dim1 = dims[1];
                dim1.label = "rows";
                dim1.size = height_;
                dim1.stride = height_*width_;                
                
                std_msgs::MultiArrayDimension& dim2 = dims[2];
                dim2.label = "point";
                dim2.size = width_;
                dim2.stride = width_;   
            }
            
            bool init(const ECWrapper& other)
            {
                return init(other.height_, other.width_, other.vfov_);
            }
            
            inline
            bool init(int height, int width, float vfov, bool clear=false)
            {
                if(!msg_locked_)
                {
                    if(height!=height_ || width!=width_ || vfov!=vfov_) //Update this to use the 'getParams functions
                    {
                        height_ = height;
                        width_ = width;
                        vfov_ = vfov;
                        
                        if(clear)
                        {
                            msg_->points.data.clear();
                        }
                            
                        init();
                    }
                    return true;
                }
                return false;
            }
            
            inline
            // TODO: ensure that storage is correct size, etc
            // NOTE: Currently, this is only used by 'transformed_points', and in a way in which that doesn't matter
            void useStorageFrom(const ECWrapper& other)
            {
                msg_ = other.msg_;
                points_ = other.points_;
            }
            
        };
      
        typedef std::shared_ptr<ECWrapper> ECWrapperPtr;
        
        inline
        ECWrapperPtr getECWrapper(int height, int width, float vfov, bool allocate_arrays=false)
        {
          return std::make_shared<ECWrapper>(height,width,vfov,allocate_arrays);
        }
        
        inline
        ECWrapperPtr getECWrapper(const ECParams& params)
        {
          return getECWrapper(params.height,params.width,params.vfov);
        }
        
        inline
        ECWrapperPtr getECWrapper(const ECWrapper& wrapper)
        {
          return getECWrapper(wrapper.getParams());
        }
        

        
    }
    
}
    
#endif //EGOCYLINDRICAL_ECWRAPPER_H
