#include <egocylindrical/point_transformer.h>
#include <egocylindrical/point_transformer_object.h>
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

        //
        inline
        void transform_impl(const utils::ECWrapper& points, utils::ECWrapper& transformed_points, const utils::ECWrapper& new_points, const PointTransformerObject& pto, int num_threads)
        {            
            const int num_cols = points.getCols();
            const int max_ind = new_points.getCols();
            const int num_pts = points.getNumPts();
            
            const float* x = (const float*)__builtin_assume_aligned(points.getX(), __BIGGEST_ALIGNMENT__);
            const float* y = (const float*)points.getY();
            const float* z = (const float*)points.getZ();
            
            float* x_n = (float*)__builtin_assume_aligned(transformed_points.getX(), __BIGGEST_ALIGNMENT__);
            float* y_n = (float*)transformed_points.getY();
            float* z_n = (float*)transformed_points.getZ();
            
            float* ranges = transformed_points.getRanges();
            
            
            //#ifndef PIPS_ON_ARM
            
            int32_t* inds = (int32_t*)__builtin_assume_aligned(transformed_points.getInds(), __BIGGEST_ALIGNMENT__);
            //#endif

            
            if (omp_get_dynamic())
                omp_set_dynamic(0);
            omp_set_nested(1);
            int omp_p = omp_get_max_threads();
            
            omp_p = std::min(omp_p-1, num_threads);
            
            
            #pragma omp parallel num_threads(omp_p)
            {
                
                #pragma omp single nowait
                {
                    if(omp_in_parallel())
                    {
                        ROS_DEBUG_STREAM("Parallel region with " << omp_get_num_threads() << " threads");
                    }
                }
                
                
                //#pragma GCC ivdep  //https://gcc.gnu.org/onlinedocs/gcc/Loop-Specific-Pragmas.html
                #pragma omp for simd schedule(static) aligned(x:__BIGGEST_ALIGNMENT__) aligned(x_n:__BIGGEST_ALIGNMENT__) aligned(ranges:__BIGGEST_ALIGNMENT__) aligned(inds:__BIGGEST_ALIGNMENT__)
                for(int p = 0; p < num_pts; ++p)
                {
                    float x_p = x[p];
                    float y_p = y[p];
                    float z_p = z[p];
                    
                    pto.transform(x_p, y_p, z_p, x_n[p], y_n[p], z_n[p]);           
                                    
                    float range_squared= worldToRangeSquared(x_n[p],z_n[p]);
                
                  
                    int idx = -1;
                    {
                      int tidx = new_points.worldToCylindricalIdx(x_n[p],y_n[p],z_n[p]);
                      
                      if(tidx < max_ind)
                      {
                        idx = tidx;
                      }
                    }

                    inds[p] = idx;


                    ranges[p] = range_squared;
                    
                }
                
            }
        }
        
        
        
        // TODO: This functionality could be moved into a tf2_ros implementation
        void transformPoints(const utils::ECWrapper& points, utils::ECWrapper& transformed_points, const utils::ECWrapper& new_points, const geometry_msgs::TransformStamped& trans, int num_threads)
        {
  
//             tf::Quaternion rotationQuaternion = tf::Quaternion(trans.transform.rotation.x,
//                                                                trans.transform.rotation.y,
//                                                                trans.transform.rotation.z,
//                                                                trans.transform.rotation.w);
//             
//             
//             tf::Matrix3x3 tempRotationMatrix = tf::Matrix3x3(rotationQuaternion);
//             
//             
//             ROS_DEBUG("Getting Rotation Matrix");
//             float rotationArray[9]  __attribute__ ((aligned (__BIGGEST_ALIGNMENT__)));
//             rotationArray[0] = (float) tempRotationMatrix[0].getX();
//             rotationArray[1] = (float) tempRotationMatrix[0].getY();
//             rotationArray[2] = (float) tempRotationMatrix[0].getZ();
//             rotationArray[3] = (float) tempRotationMatrix[1].getX();
//             rotationArray[4] = (float) tempRotationMatrix[1].getY();
//             rotationArray[5] = (float) tempRotationMatrix[1].getZ();
//             rotationArray[6] = (float) tempRotationMatrix[2].getX();
//             rotationArray[7] = (float) tempRotationMatrix[2].getY();
//             rotationArray[8] = (float) tempRotationMatrix[2].getZ();
//             //cv::Mat rotationMatrix = cv::Mat(3, 3, CV_32FC1, &rotationArray[0]);
//             
//             float translationArray[3] __attribute__ ((aligned (__BIGGEST_ALIGNMENT__)));
//             translationArray[0] = trans.transform.translation.x;
//             translationArray[1] = trans.transform.translation.y;
//             translationArray[2] = trans.transform.translation.z;
              PointTransformerObject pto(trans);
            
          //  if(points.isLocked())
          //  {
                ROS_DEBUG("Init transformed ECWrapper");
                transformed_points.init(points);    //This ensures that 'transformed_points' is big enough
                transform_impl(points, transformed_points, new_points, pto, num_threads);
        /*
            }
            else
            {
                ROS_INFO("In place");
                transformed_points.useStorageFrom(points);
                transform_impl(points, transformed_points, new_points, rotationArray, translationArray);
            }
            */
            
        }
        
    }
    
}
