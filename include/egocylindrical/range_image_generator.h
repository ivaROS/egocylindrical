#ifndef EGOCYLINDRICAL_RANGE_IMAGE_GENERATOR_H
#define EGOCYLINDRICAL_RANGE_IMAGE_GENERATOR_H


//#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

namespace egocylindrical
{


    class EgoCylinderRangeImageGenerator
    {
        ros::NodeHandle nh_, pnh_;
        image_transport::ImageTransport it_;
        image_transport::Publisher im_pub_;
        ros::Subscriber ec_sub_;
        bool use_raw_;

    public:

        EgoCylinderRangeImageGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        
        bool init();
        
        void ssCB();



    private:
        
        void ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg);

    };

} // ns egocylindrical



#endif //EGOCYLINDRICAL_RANGE_IMAGE_GENERATOR_H