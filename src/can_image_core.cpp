#include <egocylindrical/ecwrapper.h>

#include <egocylindrical/can_image_core.h>
#include <egocylindrical/can_image_core_inl.h>


namespace egocylindrical
{
    
    namespace utils
    {
        
        sensor_msgs::ImagePtr getRawCanImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateCanImageMsg<uint16_t>(cylindrical_history, num_threads, preallocated_msg);
        }
        
        sensor_msgs::ImagePtr getCanImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateCanImageMsg<float>(cylindrical_history, num_threads, preallocated_msg);
        }
        
    }
}
