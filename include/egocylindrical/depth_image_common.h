#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_COMMON_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_COMMON_H

#include <image_geometry/pinhole_camera_model.h>




namespace egocylindrical
{
    namespace utils
    {

        template<typename T> struct DepthScale {};
        
        template<>
        struct DepthScale<uint16_t>
        {
          static inline uint scale() { return 1000;}
        };
        
        template<>
        struct DepthScale<float>
        {
          static inline uint scale() { return 1;}
        };
        
        
        //Inherits from PinholeCamerModel in order to access protected member function initRectificationMaps
        class CleanCameraModel : public image_geometry::PinholeCameraModel
        {
        public:
            void init()
            {
                //Some of the camera model's functions don't work unless this has been called first
                PinholeCameraModel::initRectificationMaps();
            }
        };
        
    } //end namespace utils
} //end namespace egocylindrical

#endif  //EGOCYLINDRICAL_DEPTH_IMAGE_COMMON_H
