#include <egocylindrical/ecwrapper.h>

#include <egocylindrical/range_image_core.h>
#include <egocylindrical/range_image_core_inl.h>


namespace egocylindrical
{
    
    namespace utils
    {
        
        sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history)
        {
            return generateRangeImageMsg<uint16_t>(cylindrical_history);
        }
        
        sensor_msgs::ImagePtr getRangeImageMsg(const utils::ECWrapper& cylindrical_history)
        {
            return generateRangeImageMsg<float>(cylindrical_history);
        }
        
    }
}
