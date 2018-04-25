#ifndef RANGE_IMAGE_CORE_H
#define RANGE_IMAGE_CORE_H

#include <egocylindrical/ecwrapper.h>

#include <sensor_msgs/Image.h>


namespace egocylindrical
{
    
    namespace utils
    {
        sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history);

        sensor_msgs::ImagePtr getRangeImageMsg(const utils::ECWrapper& cylindrical_history);
        
    }
}

#endif //RANGE_IMAGE_CORE_H
