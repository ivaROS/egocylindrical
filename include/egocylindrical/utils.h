#ifndef EGOCYLINDRICAL_UTILS_H
#define EGOCYLINDRICAL_UTILS_H

#include <egocylindrical/ecwrapper.h>

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

    
    inline
    void addPoints(utils::ECWrapper& cylindrical_history, const utils::ECWrapper& new_points, bool overwrite)
    {
        
        float* x = cylindrical_history.getX();
        float* y = cylindrical_history.getY();
        float* z = cylindrical_history.getZ();
        
        const float* n_x = new_points.getX();
        const float* n_y = new_points.getY();
        const float* n_z = new_points.getZ();
        
        const float* ranges = new_points.getRanges();
        const int32_t* inds = new_points.getInds();
        

        ROS_DEBUG("Relocated the propagated image");
        //#pragma omp parallel for
        for(int i = 0; i < new_points.getCols(); ++i)
        {
            
            int idx=-1;
            
            #ifndef PIPS_ON_ARM
              idx= inds[i];
            #else
              int yidx = inds[i];
              if(yidx >= 0)
              {
                int xidx = new_points.worldToCylindricalXIdx(n_x[i],n_z[i]);
                idx = yidx * new_points.getWidth() + xidx;
              }
            #endif
            
            if(idx >=0)
            {
                float depth = ranges[i];
                
                cv::Point3f prev_point(x[idx], y[idx], z[idx]);
                
                float prev_depth = worldToRangeSquared(prev_point);
                
                if(!(prev_depth <= depth)) //overwrite || 
                {   
                    /*TODO: Check if this gets compiled out or not. If not, remove this object, 
                     * or perhaps use basic templated custom point class to combine benefits of
                     * readability and efficiency
                     */
                    cv::Point3f world_pnt(n_x[i],n_y[i],n_z[i]);
                    
                    x[idx] = world_pnt.x;
                    y[idx] = world_pnt.y;
                    z[idx] = world_pnt.z;
                    
                }
                
            }
            
            
        }
    
        
        
    }
    
    
}

}

#endif
