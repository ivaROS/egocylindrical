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
//#include <egocylindrical/EgoCylinderInfo.h>

//#include <eigen_stl_containers/eigen_stl_containers.h>

#include <boost/align/aligned_alloc.hpp>
#include <boost/align/aligned_allocator.hpp>

#include <cstddef>
#include <cstdalign>
#include <cstdint>

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
        
        template <typename T>
        inline
        T worldToRangeSquared(const T x, const T z)
        {
          return x*x + z*z;
        }
        
        template <typename T>
        inline
        T worldToRangeSquared(const cv::Point3_<T>& point)
        {
          return worldToRangeSquared(point.x, point.z);
        }
        
        template <typename T>
        inline
        T worldToRange(const cv::Point3_<T>& point)
        {
          return std::sqrt(worldToRangeSquared(point));
        }
        
        template <typename T>
        inline
        T worldToCanDepth(const cv::Point3_<T>& point)
        {
          return std::fabs(point.y);
        }
        
        /* Note: functions using 'std::sqrt' must be compiled with '-fno-math-errno' in order to be vectorized.
         * See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=51890 for explanation.
         */
        template <typename T>
        inline
        cv::Point3_<T> projectWorldToCylinder(const cv::Point3_<T>& point)
        {
          cv::Point3_<T> Pcyl_t = point / worldToRange(point);
          return Pcyl_t;
        }
        
        template <typename T>
        inline
        cv::Point3_<T> projectWorldToCan(const cv::Point3_<T>& point, float vfov)
        {
          cv::Point3_<T> Pcyl_t = point * (vfov/2) / worldToCanDepth(point);
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
        
        /*
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
          int yind = y * inv_sqrt_approximation(range_squared)*vscale_ + height_/2;
          return yind;
        }
        */
        
        
//         template <typename T>
//         inline
//         T worldToCylindricalXIdx(const cv::Point3_<T>& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
//         {
//           return std::atan2(point.x, point.z) * h_scale + cyl_width / 2;
//         }
//         
//         template <typename T>
//         inline
//         T worldToCylindricalXIdxFast(const cv::Point3_<T>& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
//         {
//           return atan2_approximation1(point.x, point.z) * h_scale + cyl_width / 2;
//         }
// 
//         template <typename T>
//         inline
//         T worldToCylindricalYIdx(const cv::Point3_<T>& point, T range, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
//         {
//           return point.y * v_scale /range + cyl_height / 2 + v_offset;
//         }
//         
//         inline
//         int pixToIdx(int xind, int yind, int width)
//         {
//           int ind = yind*width + xind;
//           
//           return ind;
//         }
//         
//         inline
//         int pixToIdx(cv::Point pix, int width)
//         {
//           return pixToIdx(pix.x, pix.y, width);
//         }
//         
//         template <typename T>
//         inline
//         T worldToCanXIdx(const cv::Point3_<T>& point, int can_width, float scale)
//         {
//           T absy = worldToCanDepth(point);
//           T xind = point.x/absy * scale + can_width/2;
//           return xind;
//         }
//         
//         template <typename T>
//         inline
//         T worldToCanZIdx(const cv::Point3_<T>& point, int can_width, float scale)
//         {
//           T absy = worldToCanDepth(point);
//           T zind = point.z/absy * scale + can_width/2;
//           return zind;
//         }
//         
//         template <typename T>
//         inline
//         int pixToCanIdx(int xind, int zind, int can_width, T y)
//         {
//           int rel_ind = zind*can_width + xind;
//           
//           int ind = (y > 0) ? rel_ind + can_width*can_width : rel_ind;
//           return ind;
//         }
//         
//         template <typename T>
//         inline
//         int worldToCanIdx(const cv::Point3_<T>& point, int can_width, float scale)
//         {
//           int xind = worldToCanXIdx(point, can_width, scale);
//           int zind = worldToCanZIdx(point, can_width, scale);
//           return pixToCanIdx(xind, zind, can_width, point.y);
// /*          
//           int rel_ind = zind*can_width + xind;
//           
//           int ind = (point.y > 0) ? rel_ind + can_width*can_width : rel_ind;
//           return ind;*/
//         }
//         
//         template <typename T>
//         inline
//         T worldToCylindricalYIdx(const cv::Point3_<T>& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
//         {
//           return worldToCylindricalYIdx(point, worldToRange(point), cyl_width, cyl_height, h_scale, v_scale, h_offset, v_offset);
//         }
//         
//         template <typename T>
//         inline
//         T worldToCylindricalYIdxFast(const cv::Point3_<T>& point, T range_squared, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
//         {
//           return point.y * v_scale * inv_sqrt_approximation(range_squared) + v_offset;
//         }
//         
//         template <typename T>
//         inline
//         cv::Point_<T> worldToCylindricalImage(const cv::Point3_<T>& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
//         {
//           T x = worldToCylindricalXIdx(point, cyl_width, cyl_height, h_scale, v_scale, h_offset, v_offset);
//           T y = worldToCylindricalYIdx(point, cyl_width, cyl_height, h_scale, v_scale, h_offset, v_offset);
//             
//           cv::Point_<T> im_pt(x,y);
//           return im_pt;
//         }
//         
//         template <typename T>
//         inline
//         cv::Point worldToCylindricalImageFast(const cv::Point3_<T>& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
//         {
//           
//           T x = worldToCylindricalXIdxFast(point, cyl_width, cyl_height, h_scale, v_scale, h_offset, v_offset);
//           T y = worldToCylindricalYIdx(point, cyl_width, cyl_height, h_scale, v_scale, h_offset, v_offset);
//           
//           cv::Point im_pt(x,y);
//           return im_pt;
//         }
//         
// 
//         template <typename T>
//         inline
//         int worldToCylindricalIdx(const cv::Point3_<T>& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
//         {
//           cv::Point image_pnt = utils::worldToCylindricalImageFast(point, cyl_width, cyl_height, h_scale, v_scale, h_offset, v_offset);
//           
//           int tidx = pixToIdx(image_pnt, cyl_width); //image_pnt.y * cyl_width +image_pnt.x;
//           
//           return tidx;
//         }

        
        
        //typedef ::egocylindrical::EgoCylinderPoints_<Eigen::aligned_allocator<void, 32> > AlignedEgoCylinderPoints;
        typedef ::egocylindrical::EgoCylinderPoints_<boost::alignment::aligned_allocator<void, 32> > AlignedEgoCylinderPoints;
        
        // NOTE: I'm not sure that using this typedef renamed version was such a good idea after all...
        //typedef AlignedEgoCylinderPoints ECMsg;
        typedef EgoCylinderPoints ECMsg;
        typedef boost::shared_ptr<ECMsg> ECMsgPtr;
        typedef boost::shared_ptr<ECMsg const> ECMsgConstPtr;
        
        template <typename T>
        using AlignedVector = std::vector<T, boost::alignment::aligned_allocator<T, __BIGGEST_ALIGNMENT__> >;
        
        
        
        //Based on https://stackoverflow.com/a/41661212
        union typeconverter{
          float    float_val;
          uint32_t int_val;
        };
        
        inline
        uint32_t toUint(float val)
        {
          typeconverter t = {.float_val=val};
          return t.int_val;
        }
        
        inline
        float toFloat(uint32_t val)
        {
          typeconverter t = {.int_val=val};
          return t.float_val;
        }
        
        struct ECParams
        {
          int height=0, width=0, can_width=0;
          float vfov=0, v_offset=0, cyl_radius=1;
          
          bool operator==(const ECParams &other) const 
          {
            return (height==other.height && width==other.width && vfov==other.vfov && can_width==other.can_width && v_offset==other.v_offset && cyl_radius==other.cyl_radius);
          }
          
          bool operator!=(const ECParams& other) const
          {
            return !operator==(other);
          }
          
          void fromCameraInfo(const ECMsg& msg)
          {
            vfov = msg.fov_v;
            
            //header_ = msg->header;
            //vfov_ = msg->fov_v;
            
            const std::vector<std_msgs::MultiArrayDimension>& dims = msg.points.layout.dim;
            height = dims[1].size;
            width = dims[2].size;
            can_width = dims[3].size;
            if(dims.size()>4)
            {
              v_offset = toFloat(dims[4].size);
              
              if(dims.size()>5)
              {
                cyl_radius = toFloat(dims[5].size);
              }
              else
              {
                cyl_radius = 1;
              }
            }
            else
            {
              v_offset = 0;
            }
          }
          
          inline
          int getNumPts() const
          {
            return getCols() + 2*can_width*can_width;
          }
          
          inline
          int getCols() const
          {
            return height*width;
          }
        };
        
        //NOTE: This should probably be moved out of the header file
        inline
        std::ostream& operator<< (std::ostream& stream, const ECParams& params)
        {
          stream << "height: " << params.height << ", width: " << params.width << ", can_width: " << params.can_width << ", v_offset: " << params.v_offset << ", vfov: " << params.vfov;
          return stream;
        }
        
        struct DerivedECParams : public ECParams
        {
          int v_center;
          float hscale, vscale, canscale;
          
          inline
          bool update(const ECParams& params)
          {
            //TODO: Add conditions to verify that valid parameters were selected
            //static_cast<ECParams>(*this) = params;
            *((ECParams*)this)=params;
            hscale = width/(2*M_PI);
            vscale = height/vfov;  //NOTE: In the paper, vscale=hscale. If keeping them separate does not prove useful, they should be merged
            canscale = can_width*vfov/4;
            v_center = v_offset * height / vfov;
            
            return true;
          }
          
        };
        
        //NOTE: This should probably be moved out of the header file
        inline
        std::ostream& operator<< (std::ostream& stream, const DerivedECParams& params)
        {
          stream << ((ECParams)params) << ", v_center: " << params.v_center << ", hscale: " << params.hscale << ", vscale: " << params.vscale << ", canscale: " << params.canscale;
          return stream;
        }
        
        
        class ECConverter
        {

        protected:
          DerivedECParams params_;
          
        public:
          ECConverter()
          {
          }
          
          
          inline
          bool update(const ECParams& params)
          {
            return params_.update(params);
          }
          
          inline
          bool fromCameraInfo(const ECMsgConstPtr& msg)
          {
            ECParams params;
            params.fromCameraInfo(*msg);

            return update(params);
          }
          
          inline
          bool fromParams(const ECParams& params)
          {
            return update(params);
          }
          
          inline
          ECParams getParams() const
          {
              return params_;
          }
          
          void fillMsgInfo(ECMsg& msg) const
          {
            msg.fov_v = params_.vfov;
            
            std::vector<std_msgs::MultiArrayDimension>& dims = msg.points.layout.dim;
            dims.resize(6);
            
            std_msgs::MultiArrayDimension& dim0 = dims[0];
            dim0.label = "components";
            dim0.size = 3;
            dim0.stride = 3*params_.getCols(); 
            
            std_msgs::MultiArrayDimension& dim1 = dims[1];
            dim1.label = "rows";
            dim1.size = params_.height;
            dim1.stride = params_.getCols();                
            
            std_msgs::MultiArrayDimension& dim2 = dims[2];
            dim2.label = "point";
            dim2.size = params_.width;
            dim2.stride = params_.width;   
            
            std_msgs::MultiArrayDimension& dim3 = dims[3];
            dim3.label = "can";
            dim3.size = params_.can_width;
            dim3.stride = params_.can_width; 
            
            std_msgs::MultiArrayDimension& dim4 = dims[4];
            dim4.label = "v_offset";
            dim4.size = toUint(params_.v_offset);
            dim4.stride = 0;  //Not used
            
            std_msgs::MultiArrayDimension& dim5 = dims[5];
            dim5.label = "cyl_radius";
            dim5.size = toUint(params_.cyl_radius);
            dim5.stride = 0;  //Not used
          }
          
          inline
          int getHeight() const
          {
            return params_.height;
          }
          
          inline
          int getWidth() const
          {
            return params_.width;
          }
          
          inline
          int getCanWidth() const
          {
            return params_.can_width;
          }
          
          float getHScale() const
          {
            return params_.hscale;
          }
          
          float getVScale() const
          {
            return params_.vscale;
          }
          
          float getCanScale() const
          {
            return params_.canscale;
          }
          
          inline
          int getNumPts() const
          {
            return params_.getNumPts();
          }
          
          inline
          int getCols() const
          {
            return params_.getCols();
          }
          
          inline
          cv::Rect getImageRoi() const
          {
            return cv::Rect(0, 0, params_.width, params_.height);
          }
          
          
          template <typename S, typename T>
          inline
          void worldToCylindricalXIdx(const cv::Point3_<S>& point, T& x_idx, bool fast=false) const
          {
            x_idx = std::atan2(point.x, point.z) * params_.hscale + params_.width / 2;
          }
          
          template <typename S, typename T>
          inline
          void worldToCylindricalXIdxFast(const cv::Point3_<S>& point, T& x_idx) const
          {
            x_idx = atan2_approximation1(point.x, point.z) * params_.hscale + params_.width / 2;
          }
          

          template <typename S, typename T>
          inline
          void worldToCylindricalYIdx(const cv::Point3_<S>& point, S range, T& y_idx) const
          {
            y_idx = point.y * params_.vscale /range + params_.height / 2 + params_.v_center;
          }
          
          template <typename S, typename T>
          inline
          void worldToCylindricalYIdxFast(const cv::Point3_<S>& point, S range_squared, T& y_idx) const
          {
            y_idx = point.y * params_.vscale * inv_sqrt_approximation(range_squared) + params_.v_center;
          }
          
          template <typename S, typename T>
          inline
          void worldToCylindricalYIdx(const cv::Point3_<S>& point, T& y_idx) const
          {
            worldToCylindricalYIdx(point, worldToRange(point), y_idx);
          }
          
          template <typename S, typename T>
          inline
          void worldToCylindricalImage(const cv::Point3_<S>& point, cv::Point_<T>& pix) const
          {
            worldToCylindricalXIdxFast(point, pix.x);
            worldToCylindricalYIdx(point, pix.y);
          }
          
          template <typename T>
          inline
          cv::Point worldToCylindricalImage(const cv::Point3_<T>& point) const
          {
            cv::Point pix;
            worldToCylindricalImage(point, pix);
            return pix;
          }

          //Redundant, perhaps, but convenient
          template <typename S, typename T>
          inline 
          void project3dToPixel(const cv::Point3_<S> point, cv::Point_<T>& pix) const
          {
            worldToCylindricalImage(point, pix);
          }
          
          template <typename T>
          inline 
          cv::Point project3dToPixel(const cv::Point3_<T> point) const
          {
            cv::Point pix;
            project3dToPixel(point, pix);
            return pix;
          }
          
          template <typename S, typename T, typename U>
          inline 
          void project3dToPixelRange(const cv::Point3_<S> point, cv::Point_<T>& pix, U& range) const
          {
            project3dToPixel(point, pix);
            range = utils::worldToRange(point);
          }
          
//           inline
//           int pixToIdx(int xind, int yind) const
//           {
//             return utils::pixToIdx(xind, yind, getWidth());
//           }
//           
          inline
          int pixToIdx(const cv::Point& pix) const
          {
            int ind = pix.y*params_.width + pix.x;
            return ind;
          }
          
          inline
          int pixToIdx(int x, int y) const
          {
            return pixToIdx(cv::Point(x,y));
          }
          
          template<typename T>
          inline
          int worldToCylindricalIdx(const cv::Point3_<T>& point) const
          {
            cv::Point pix;
            worldToCylindricalImage(point, pix);
            int idx = pixToIdx(pix);
            return idx;
          }
          
          template<typename T>
          inline
          int worldToCylindricalIdx(T x, T y, T z) const
          {
            cv::Point3_<T> point(x,y,z);
            return worldToCylindricalIdx(point);
          }
          
          template <typename S, typename T>
          inline
          void worldToCanXIdx(const cv::Point3_<S>& point, T& x_idx) const
          {
            if(point.y >=0 )
            {
              S absy = point.y;
              S h_b = params_.vfov/2-params_.v_offset;
              x_idx = point.x*h_b/absy * (params_.can_width/2) + params_.can_width/2;
            }
            else
            {
              S absy = -point.y;
              S h_t = params_.vfov/2+params_.v_offset;
              x_idx = point.x*h_t/absy * (params_.can_width/2) + params_.can_width/2;
            }
//             T absy = worldToCanDepth(point);
//             T xind = point.x/absy * getCanScale() + getCanWidth()/2;
//             return xind;
          }
          
          template <typename S, typename T>
          inline
          void worldToCanXIdx(S x, S y, S z, T& x_idx) const
          {
            worldToCanXIdx(cv::Point3_<S>(x,y,z), x_idx);
          }
          
          template <typename T>
          inline
          int worldToCanXIdx(const cv::Point3_<T> point) const
          {
            int x_idx;
            worldToCanXIdx(point, x_idx);
            return x_idx;
          }
          
          template <typename S, typename T>
          inline
          void worldToCanZIdx(const cv::Point3_<S>& point, T& z_idx) const
          {
            if(point.y >=0 )
            {
              S absy = point.y;
              S h_b = params_.vfov/2-params_.v_offset;
              z_idx = point.z*h_b/absy * (params_.can_width/2) + params_.can_width/2;
            }
            else
            {
              S absy = -point.y;
              S h_t = params_.vfov/2+params_.v_offset;
              z_idx = point.z*h_t/absy * (params_.can_width/2) + params_.can_width/2;
            }
//             T absy = worldToCanDepth(point);
//             T zind = point.z/absy * getCanScale() + getCanWidth()/2;
//             return zind;
          }
          
          template <typename T>
          inline
          int worldToCanZIdx(const cv::Point3_<T>& point) const
          {
            int z_idx;
            worldToCanZIdx(point, z_idx);
            return z_idx;
          }
          
          template <typename S, typename T>
          inline
          void worldToCanZIdx(S x, S y, S z, T& z_idx) const
          {
            return worldToCanZIdx(cv::Point3_<S>(x,y,z), z_idx);
          }
          
          template <typename T>
          inline
          int pixToCanIdx(int xind, int zind, T y) const
          {
            int rel_ind = zind*params_.can_width + xind;
            int ind = (y > 0) ? rel_ind + params_.can_width*params_.can_width : rel_ind;
            return ind;
          }
          
          template <typename T>
          inline
          int pixToCanIdx(const cv::Point3i& pt, T y) const
          {
            return pixToCanIdx(pt.x, pt.z, y);
          }

          
          template <typename T>
          inline
          int worldToCanIdx(const cv::Point3_<T>& point) const
          {
            cv::Point3i pix;
            worldToCanXIdx(point, pix.x);
            worldToCanZIdx(point, pix.z);
            int can_idx = pixToCanIdx(pix, point.y) + params_.getCols();
            return can_idx;
          }
          
          template <typename T>
          inline
          int worldToCanIdx(T x, T y, T z) const
          {
            return worldToCanIdx(cv::Point3_<T>(x,y,z));
          }
          
          template <typename T>
          inline
          int worldToIdx(const cv::Point3_<T>& point) const
          {
            int idx = worldToCylindricalIdx(point);
            return idx;

            if(idx<0 || idx >= params_.getCols())
            {
              idx = worldToCanIdx(point);// + params_.getCols();
            }
            return idx;
          }
          
         
          template <typename S, typename T>
          inline
          void projectPixelTo3dRay(const cv::Point_<S>& point, cv::Point3_<T>& ray) const
          {
            T theta = (point.x - (params_.width/2))/params_.hscale;
            
            ray.x = sin(theta);
            ray.z = cos(theta);
            
            ray.y = (point.y - (params_.height/2 + params_.v_offset))/params_.vscale;
          }
          
         
          template <typename T>
          inline
          cv::Point3_<float> projectPixelTo3dRay(const cv::Point_<T>& point) const
          {
            cv::Point3_<float> ray;
            projectPixelTo3dRay(point, ray);
            return ray;
          }
          
          template <typename T>
          inline
          cv::Point3d projectCanPixelTo3dRay(const cv::Point_<T>& point) const
          {
            cv::Point3_<T> ray;
            ray.y = -1;
            
            T y = point.y;
            if(point.y >= params_.can_width)
            {
              y -= getCanWidth();
              ray.y = 1;
            }
                        
            ray.x = (point.x - (params_.can_width/2))/params_.canscale;
            ray.z = (y - (params_.can_width/2))/params_.canscale;
            
            return ray;
          }
          
          template <typename T>
          inline
          cv::Point3_<T> projectWorldToCylinder(const cv::Point3_<T>& point) const
          {
            cv::Point3_<T> Pcyl_t = point / worldToRange(point) * params_.cyl_radius;
            return Pcyl_t;
          }
          
          template <typename T>
          inline
          cv::Point3_<T> projectWorldToCan(const cv::Point3_<T>& point) const
          {
            cv::Point3_<T> Pcyl_t;
            if(point.y >=0 )
            {
              T absy = point.y;
              T h_b = (params_.vfov/2-params_.v_offset)*params_.cyl_radius;
              Pcyl_t = point * h_b / absy;
            }
            else
            {
              T absy = -point.y;
              T h_t = (params_.vfov/2+params_.v_offset)*params_.cyl_radius;
              Pcyl_t = point * h_t / absy;
            }
            
            //cv::Point3_<T> Pcyl_t = point * (params_.vfov/2) / worldToCanDepth(point);
            return Pcyl_t;
          }
          
          template <typename T>
          inline
          T worldToCanDepth(const cv::Point3_<T>& point)
          {
            return std::fabs(point.y);
          }

        };
        
        
        // TODO: create an abstract class to serve as interface so that different storage mechanisms can be used on the backend
        class ECWrapper : public ECConverter
        {
        private:
        public:         
            float* points_;
            
            AlignedVector<float> ranges_;
            AlignedVector<int32_t> inds_;

            bool allocate_arrays_;
            
            std_msgs::Header header_;
            ECMsgPtr msg_; // The idea is to store everything in the message's allocated storage to prevent copies
            
            ECMsgConstPtr const_msg_;
            
            bool msg_locked_;
            
            // For now, just get things working using this.
            // Note: It may be preferable to allocate x,y,z separately to ensure they are aligned (unless the width is chosen such that they will be anyway...)
            // Another idea: possibly template this class by height/width, potentially enabling compile time optimizations
            // Also: maybe should store x, then z, then y, since only x and z are needed for range image
           

           
        public:
            
            ECWrapper(const ECParams& params, bool allocate_arrays = false):
                allocate_arrays_(allocate_arrays)
            {
                msg_ = boost::make_shared<ECMsg>();
                
                msg_locked_ = false;
                
                init(params);
            }
            
            ECWrapper(const ECMsgConstPtr& ec_points) 
            {
                fromCameraInfo(ec_points);
                const_msg_ = ec_points;
                header_ = const_msg_->header;

                points_ = (float*) const_msg_->points.data.data() + (const_msg_->points.layout.data_offset) / sizeof(float);
                                
                msg_locked_ = true;
            }
            
            ~ECWrapper()
            {

            }

            
            inline float* getPoints()                   { return (float*) points_; } //__builtin_assume_aligned(points_, __BIGGEST_ALIGNMENT__)
            inline const float* getPoints() const       { return (const float*) points_; } //__builtin_assume_aligned(points_, __BIGGEST_ALIGNMENT__)
            
            inline float* getX()                        { return getPoints(); }
            inline const float* getX()          const   { return (const float*) getPoints(); }
            
            inline float* getY()                        { return getPoints() + getNumPts(); }
            inline const float* getY()          const   { return (const float*) getPoints() + getNumPts(); }
            
            inline float* getZ()                        { return getPoints() + 2*getNumPts(); }
            inline const float* getZ()          const   { return (const float*) getPoints() + 2*getNumPts(); }
            
            inline float* getRanges()                   { return (float*) __builtin_assume_aligned(ranges_.data(), __BIGGEST_ALIGNMENT__); }
            inline const float* getRanges()     const   { return (const float*) __builtin_assume_aligned(ranges_.data(), __BIGGEST_ALIGNMENT__); }
            
            inline int32_t* getInds()                  { return (int32_t*) __builtin_assume_aligned(inds_.data(), __BIGGEST_ALIGNMENT__); }
            inline const int32_t* getInds()    const   { return (const int32_t*) __builtin_assume_aligned(inds_.data(), __BIGGEST_ALIGNMENT__); }
            
            inline bool isLocked()              const   { return msg_locked_; }

            inline
            void setHeader(std_msgs::Header header)
            {
                header_ = header;
                msg_->header = header;
            }
            
            inline
            std_msgs::Header getHeader() const
            {
                return header_; 
            }
            
            inline
            ECMsgConstPtr getEgoCylinderPointsMsg()
            {
                msg_locked_ = true;
                return (ECMsgConstPtr) msg_;
            }
            
            inline
            ECMsgConstPtr getEgoCylinderInfoMsg() const
            {
              ECMsgPtr info = boost::make_shared<ECMsg>();
              fillMsgInfo(*info);
              info->header = msg_->header;
              return (ECMsgConstPtr) info;
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
                
                ROS_DEBUG_STREAM("Allocating space for " << getNumPts() << " points.");
                msg_->points.data.resize(3*getNumPts() + buffer_objects, dNaN);
                
                
                //Align data pointer
                {
                    void* temp_points = (void*) msg_->points.data.data();
                    
                    size_t space_before = getNumPts()*3*sizeof(float);
                    size_t space_after = space_before;
                    
                    std::align(biggest_alignment, sizeof(float), temp_points, space_after);
                    points_ = (float*) temp_points;
                    
                    msg_->points.layout.data_offset = (space_before - space_after);
                    
                    ROS_DEBUG_STREAM("Aligned points_, adjusted pointer by " << (space_before - space_after) << " bytes");
                }
                
                if(allocate_arrays_)
                {
                    ranges_.resize(getNumPts());
                    inds_.resize(getNumPts()); 
                }

                fillMsgInfo(*msg_);
            }
            
            bool init(const ECWrapper& other)
            {
                return init(other.getParams());
            }
            
            inline
            bool init(const ECParams& params, bool clear=false)
            {
                ROS_DEBUG_STREAM("Current parameters: [" << params_ << "]; New parameters: [" << params << "]");
                if(msg_->points.data.size()==0)
                {
                  ROS_DEBUG_STREAM("No space for points!");
                }
                if(!msg_locked_)
                {
                    if(params != (const ECParams)params_)
                    {
                        ROS_DEBUG_STREAM("Params have changed, update!");
                        fromParams(params);
                        if(clear)
                        {
                            msg_->points.data.clear();
                        }
                            
                        init();
                    }
                    else
                    {
                      ROS_DEBUG_STREAM("Params have not changed!");
                      if(clear)
                      {
                        std::fill(msg_->points.data.begin(), msg_->points.data.end(), dNaN);
                      }
                    }
                    if(msg_->points.data.size()==0)
                    {
                      ROS_WARN_STREAM("Still no space for points!");
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
            
            using Ptr=std::shared_ptr<ECWrapper>;
        };
      
        typedef std::shared_ptr<ECWrapper> ECWrapperPtr;
        
        inline
        ECWrapperPtr getECWrapper(const ECParams& params, bool allocate_arrays=false)
        {
          return std::make_shared<ECWrapper>(params, allocate_arrays);
        }
        
        inline
        ECWrapperPtr getECWrapper(int height, int width, float vfov, int can_width, bool allocate_arrays=false)
        {
          ECParams params;
          params.height=height;
          params.width=width;
          params.vfov = vfov;
          params.can_width=can_width;
          return getECWrapper(params, allocate_arrays);
        }
        
        inline
        ECWrapperPtr getECWrapper(const ECWrapper& wrapper, bool allocate_arrays=false)
        {
          return getECWrapper(wrapper.getParams(), allocate_arrays);
        }

    }
    
}
    
#endif //EGOCYLINDRICAL_ECWRAPPER_H
