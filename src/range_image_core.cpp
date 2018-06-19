#include <egocylindrical/ecwrapper.h>

#include <egocylindrical/range_image_core.h>
#include <egocylindrical/range_image_core_inl.h>


namespace egocylindrical
{
    
    namespace utils
    {
        
        sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads)
        {
            return generateRangeImageMsg<uint16_t>(cylindrical_history, num_threads);
        }
        
        sensor_msgs::ImagePtr getRangeImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads)
        {
            return generateRangeImageMsg<float>(cylindrical_history, num_threads);
        }
        
    }
}
