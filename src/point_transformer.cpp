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
            
            float* ranges = points.getRanges();
            long int* inds = points.getInds();
            
            
            //#pragma GCC ivdep  //https://gcc.gnu.org/onlinedocs/gcc/Loop-Specific-Pragmas.html
            
            if (omp_get_dynamic())
                omp_set_dynamic(0);
            
            #pragma omp parallel
            {
                if(omp_in_parallel())
                {
                    ROS_INFO_STREAM("Parallel region with " << omp_get_num_threads() << " threads");
                }
                
                
                #pragma omp for simd schedule(static)
                for(long int p = 0; p < num_cols; ++p)
                {
                  /*
                    if(omp_in_parallel())
                    {
                        int thread_id = omp_get_thread_num();
                        
                        ROS_INFO_STREAM_NAMED("omp","OpenMP active! Thread # " << thread_id);
                        
                    }
                    ROS_INFO_STREAM("testing");
                    */
                    
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
                    
                    x[p] = r0 * x_p + r1 * y_p + r2 * z_p + t0;
                    y[p] = r3 * x_p + r4 * y_p + r5 * z_p + t1;
                    z[p] = r6 * x_p + r7 * y_p + r8 * z_p + t2;
                    

                    
                    
                    float depth=dNaN;
                    
                    int idx = -1;
        
                    cv::Point3f world_pnt(x[p],y[p],z[p]);
                    
                    depth= worldToRangeSquared(world_pnt);
                    
                    cv::Point image_pnt = points.worldToCylindricalImage(world_pnt);
                    
                    int tidx = image_pnt.y * width +image_pnt.x;
                    
                    if(tidx < num_cols)
                        idx = tidx;
    
                    inds[p] = idx;
                    ranges[p] = depth;
                    
                }
            
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
                        
            const float* const R = (float*)__builtin_assume_aligned(_R, __BIGGEST_ALIGNMENT__);
            const float* const T = (float*)__builtin_assume_aligned(_T, __BIGGEST_ALIGNMENT__);
            
            const int num_cols = points.getCols();
            const int width = points.getWidth();
            const int height = points.getHeight();
            
            
            const float* x = (const float*)__builtin_assume_aligned(points.getX(), __BIGGEST_ALIGNMENT__);
            const float* y = (const float*)__builtin_assume_aligned(points.getY(), __BIGGEST_ALIGNMENT__);
            const float* z = (const float*)__builtin_assume_aligned(points.getZ(), __BIGGEST_ALIGNMENT__);
            
            float* x_n = (float*)__builtin_assume_aligned(transformed_points.getX(), __BIGGEST_ALIGNMENT__);
            float* y_n = (float*)__builtin_assume_aligned(transformed_points.getY(), __BIGGEST_ALIGNMENT__);
            float* z_n = (float*)__builtin_assume_aligned(transformed_points.getZ(), __BIGGEST_ALIGNMENT__);
            
            //float* ranges = transformed_points.getRanges();
            //long int* inds = transformed_points.getInds();
            
            float* ranges = (float*)__builtin_assume_aligned(transformed_points.getRanges(), __BIGGEST_ALIGNMENT__);
            long int* inds = (long int*)__builtin_assume_aligned(transformed_points.getInds(), __BIGGEST_ALIGNMENT__);
            
            
            
                        
            if (omp_get_dynamic())
                omp_set_dynamic(0);
            omp_set_nested(1);
            int omp_p = omp_get_max_threads();
            
            omp_p = std::min(omp_p-1, 4);
            
            
            #pragma omp parallel num_threads(omp_p)
            {
                
                #pragma omp single nowait
                {
                    if(omp_in_parallel())
                    {
                        ROS_INFO_STREAM("Parallel region with " << omp_get_num_threads() << " threads");
                    }
                }
                
                
                //#pragma GCC ivdep  //https://gcc.gnu.org/onlinedocs/gcc/Loop-Specific-Pragmas.html
                #pragma omp for simd schedule(static)
                for(long int p = 0; p < num_cols; ++p)
                {

                    float x_p = x[p];
                    float y_p = y[p];
                    float z_p = z[p];
                    
                    x_n[p] = r0 * x[p] + r1 * y[p] + r2 * z[p] + t0;
                    y_n[p] = r3 * x[p] + r4 * y[p] + r5 * z[p] + t1;
                    z_n[p] = r6 * x[p] + r7 * y[p] + r8 * z[p] + t2;
                     
                    float depth=dNaN;
                    
                    int idx = -1;
                                    
                    depth= worldToRangeSquared(x_n[p],z_n[p]);
                
                    int tidx = points.worldToCylindricalIdx(x_n[p],y_n[p],z_n[p]);
                    
                    if(tidx < num_cols)
                        idx = tidx;
                    
                    inds[p] = idx;
                    ranges[p] = depth;
                    
                }
                
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