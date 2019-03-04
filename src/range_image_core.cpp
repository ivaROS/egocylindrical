#include <egocylindrical/ecwrapper.h>

#include <egocylindrical/range_image_core.h>
#include <egocylindrical/range_image_core_inl.h>


namespace egocylindrical
{
    
    namespace utils
    {
        
        sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateRangeImageMsg<uint16_t>(cylindrical_history, num_threads, preallocated_msg);
        }
        
        sensor_msgs::ImagePtr getRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateRangeImageMsg<float>(cylindrical_history, num_threads, preallocated_msg);
        }
        
    }
}
