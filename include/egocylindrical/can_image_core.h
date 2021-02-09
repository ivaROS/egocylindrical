#ifndef CAN_IMAGE_CORE_H
#define CAN_IMAGE_CORE_H

#include <egocylindrical/ecwrapper.h>

#include <sensor_msgs/Image.h>


namespace egocylindrical
{
    
    namespace utils
    {
        sensor_msgs::ImagePtr getRawCanImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg=nullptr);

        sensor_msgs::ImagePtr getCanImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg=nullptr);
        
    }
}

#endif //CAN_IMAGE_CORE_H
