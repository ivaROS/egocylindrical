#ifndef RANGE_IMAGE_CORE_H
#define RANGE_IMAGE_CORE_H

#include <egocylindrical/ecwrapper.h>

#include <sensor_msgs/Image.h>


namespace egocylindrical
{
    
    namespace utils
    {
        sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg=nullptr);

        sensor_msgs::ImagePtr getRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg=nullptr);
        
    }
}

#endif //RANGE_IMAGE_CORE_H
