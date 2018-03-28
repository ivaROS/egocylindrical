#include <egocylindrical/ecwrapper.h>

#include <ros/ros.h>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <omp.h>

#include <geometry_msgs/TransformStamped.h>


namespace egocylindrical
{
    
    namespace utils
    {

        /* Inplace transform
        * 
        */
        inline
        void transform_impl(utils::ECWrapper& points, const float*  const _R, const float*  const _T)
        {
            cv::Rect image_roi = points.getImageRoi();
            
            const float r0 = _R[0];
            const float r1 = _R[1];
            const float r2 = _R[2];
            const float r3 = _R[3];
            const float r4 = _R[4];
            const float r5 = _R[5];
            const float r6 = _R[6];
            const float r7 = _R[7];
            const float r8 = _R[8];
            
            const float t0 = _T[0];
            const float t1 = _T[1];
            const float t2 = _T[2];
            
            float* point_ptr = (float*)__builtin_assume_aligned(points.getPoints(), 16);            
            
            const float* const R = (float*)__builtin_assume_aligned(_R, __BIGGEST_ALIGNMENT__);
            const float* const T = (float*)__builtin_assume_aligned(_T, __BIGGEST_ALIGNMENT__);

            const int num_cols = points.getCols();
            const int width = points.getWidth();
            const int height = points.getHeight();
            
            float* x = points.getX();
            float* y = points.getY();
            float* z = points.getZ();
            
            
            #pragma GCC ivdep  //https://gcc.gnu.org/onlinedocs/gcc/Loop-Specific-Pragmas.html
            //#pragma omp simd
            for(long int p = 0; p < num_cols; ++p)
            {
                /*
                float temp[3];
                for(int row=0; row < 3; ++row)
                {
                    temp[row] = 0;
                    for(int col=0; col < 3; ++col)
                    {
                        temp[row] += R[row*3+col] * point_ptr[num_cols * col + p]; // points.at<float>(col,p);
                    }
                }
                
                for(int row=0; row < 3; ++row)
                {
                    point_ptr[num_cols * row + p] = temp[row] + T[row];
                }
                */
                
                
                float x_p = x[p];
                float y_p = y[p];
                float z_p = z[p];
                
                x[p] = r0 * x[p] + r1 * y[p] + r2 * z[p] + t0;
                y[p] = r3 * x[p] + r4 * y[p] + r5 * z[p] + t1;
                z[p] = r6 * x[p] + r7 * y[p] + r8 * z[p] + t2;
                

                
                
                float depth=dNaN;
                
                int idx = -1;
      
                cv::Point3f world_pnt(x[p],y[p],z[p]);
                
                depth= worldToRangeSquared(world_pnt);
                
                cv::Point image_pnt = points.worldToCylindricalImage(world_pnt);
                
                int tidx = image_pnt.y * width +image_pnt.x;
                
                if(tidx < num_cols)
                    idx = tidx;
   
                points.inds_[p] = idx;
                points.ranges_[p] = depth;
                
            }
                    
        }
        
        
        inline
        void transform_impl(const utils::ECWrapper& points, utils::ECWrapper& transformed_points, const float*  const _R, const float*  const _T)
        {
            cv::Rect image_roi = points.getImageRoi();
            
            const float r0 = _R[0];
            const float r1 = _R[1];
            const float r2 = _R[2];
            const float r3 = _R[3];
            const float r4 = _R[4];
            const float r5 = _R[5];
            const float r6 = _R[6];
            const float r7 = _R[7];
            const float r8 = _R[8];
            
            const float t0 = _T[0];
            const float t1 = _T[1];
            const float t2 = _T[2];
            
            float* point_ptr = (float*)__builtin_assume_aligned(points.getPoints(), 16);            
            
            const float* const R = (float*)__builtin_assume_aligned(_R, __BIGGEST_ALIGNMENT__);
            const float* const T = (float*)__builtin_assume_aligned(_T, __BIGGEST_ALIGNMENT__);
            
            const int num_cols = points.getCols();
            const int width = points.getWidth();
            const int height = points.getHeight();
            
            const float* x = points.getX();
            const float* y = points.getY();
            const float* z = points.getZ();
            
            float* x_n = transformed_points.getX();
            float* y_n = transformed_points.getY();
            float* z_n = transformed_points.getZ();
            
            
            #pragma GCC ivdep  //https://gcc.gnu.org/onlinedocs/gcc/Loop-Specific-Pragmas.html
            //#pragma omp simd
            for(long int p = 0; p < num_cols; ++p)
            {
                /*
                 *       float temp[3];
                 *       for(int row=0; row < 3; ++row)
                 *       {
                 *           temp[row] = 0;
                 *           for(int col=0; col < 3; ++col)
                 *           {
                 *               temp[row] += R[row*3+col] * point_ptr[num_cols * col + p]; // points.at<float>(col,p);
            }
            }
            
            for(int row=0; row < 3; ++row)
            {
            point_ptr[num_cols * row + p] = temp[row] + T[row];
            }
            */
                
                
                float x_p = x[p];
                float y_p = y[p];
                float z_p = z[p];
                
                x_n[p] = r0 * x[p] + r1 * y[p] + r2 * z[p] + t0;
                y_n[p] = r3 * x[p] + r4 * y[p] + r5 * z[p] + t1;
                z_n[p] = r6 * x[p] + r7 * y[p] + r8 * z[p] + t2;
                
                
                
                
                float depth=dNaN;
                
                int idx = -1;
                
                cv::Point3f world_pnt(x[p],y[p],z[p]);
                
                depth= worldToRangeSquared(world_pnt);
                
                cv::Point image_pnt = points.worldToCylindricalImage(world_pnt);
                
                int tidx = image_pnt.y * width +image_pnt.x;
                
                if(tidx < num_cols)
                    idx = tidx;
                
                transformed_points.inds_[p] = idx;
                transformed_points.ranges_[p] = depth;
                
            }
            
        }
        
        
        // TODO: This functionality could be moved into a tf2_ros implementation
        void transformPoints(utils::ECWrapper& points, const geometry_msgs::TransformStamped& trans)
        {
            
  
            tf::Quaternion rotationQuaternion = tf::Quaternion(trans.transform.rotation.x,
                                trans.transform.rotation.y,
                                trans.transform.rotation.z,
                                trans.transform.rotation.w);


            tf::Matrix3x3 tempRotationMatrix = tf::Matrix3x3(rotationQuaternion);


            ROS_DEBUG("Getting Rotation Matrix");
            float rotationArray[9]  __attribute__ ((aligned (__BIGGEST_ALIGNMENT__)));
            rotationArray[0] = (float) tempRotationMatrix[0].getX();
            rotationArray[1] = (float) tempRotationMatrix[0].getY();
            rotationArray[2] = (float) tempRotationMatrix[0].getZ();
            rotationArray[3] = (float) tempRotationMatrix[1].getX();
            rotationArray[4] = (float) tempRotationMatrix[1].getY();
            rotationArray[5] = (float) tempRotationMatrix[1].getZ();
            rotationArray[6] = (float) tempRotationMatrix[2].getX();
            rotationArray[7] = (float) tempRotationMatrix[2].getY();
            rotationArray[8] = (float) tempRotationMatrix[2].getZ();
            //cv::Mat rotationMatrix = cv::Mat(3, 3, CV_32FC1, &rotationArray[0]);
            
            float translationArray[3] __attribute__ ((aligned (__BIGGEST_ALIGNMENT__)));
            translationArray[0] = trans.transform.translation.x;
            translationArray[1] = trans.transform.translation.y;
            translationArray[2] = trans.transform.translation.z;
            
            transform_impl(points, rotationArray, translationArray);

        }
        
        // TODO: This functionality could be moved into a tf2_ros implementation
        void transformPoints(const utils::ECWrapper& points, utils::ECWrapper& transformed_points, const geometry_msgs::TransformStamped& trans)
        {
            
            
            tf::Quaternion rotationQuaternion = tf::Quaternion(trans.transform.rotation.x,
                                                               trans.transform.rotation.y,
                                                               trans.transform.rotation.z,
                                                               trans.transform.rotation.w);
            
            
            tf::Matrix3x3 tempRotationMatrix = tf::Matrix3x3(rotationQuaternion);
            
            
            ROS_DEBUG("Getting Rotation Matrix");
            float rotationArray[9]  __attribute__ ((aligned (__BIGGEST_ALIGNMENT__)));
            rotationArray[0] = (float) tempRotationMatrix[0].getX();
            rotationArray[1] = (float) tempRotationMatrix[0].getY();
            rotationArray[2] = (float) tempRotationMatrix[0].getZ();
            rotationArray[3] = (float) tempRotationMatrix[1].getX();
            rotationArray[4] = (float) tempRotationMatrix[1].getY();
            rotationArray[5] = (float) tempRotationMatrix[1].getZ();
            rotationArray[6] = (float) tempRotationMatrix[2].getX();
            rotationArray[7] = (float) tempRotationMatrix[2].getY();
            rotationArray[8] = (float) tempRotationMatrix[2].getZ();
            //cv::Mat rotationMatrix = cv::Mat(3, 3, CV_32FC1, &rotationArray[0]);
            
            float translationArray[3] __attribute__ ((aligned (__BIGGEST_ALIGNMENT__)));
            translationArray[0] = trans.transform.translation.x;
            translationArray[1] = trans.transform.translation.y;
            translationArray[2] = trans.transform.translation.z;
            
            transform_impl(points, transformed_points, rotationArray, translationArray);
            
        }
        
    }
    
}