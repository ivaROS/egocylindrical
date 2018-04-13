
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

namespace egocylindrical
{
    //Forward declare functions from other compilation units
    namespace utils
    {
        sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history);
    }

    class EgoCylinderRangeImageGenerator
    {
        ros::NodeHandle nh_, pnh_;
        image_transport::ImageTransport it_;
        image_transport::Publisher im_pub_;
        ros::Subscriber ec_sub_;
        

        
        
    public:

      EgoCylinderRangeImageGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
            nh_(nh),
            pnh_(pnh),
            it_(nh_)
        {

        }
        
        bool init()
        {
            im_pub_ = it_.advertise("range_image", 2);
          
            ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderRangeImageGenerator::ecPointsCB, this);
            
            return true;
        }


    private:
        
        void ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
        {
            ROS_INFO("Received EgoCylinderPoints msg");
            
            if(im_pub_.getNumSubscribers() > 0)
            {

              ros::WallTime start = ros::WallTime::now();
              
              utils::ECWrapper ec_pts(ec_msg);
                    
              sensor_msgs::Image::ConstPtr image_ptr = utils::getRawRangeImageMsg(ec_pts);

              ROS_INFO_STREAM("Generating egocylindrical image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
              

              ROS_INFO("publish egocylindrical image");
              
              im_pub_.publish(image_ptr);
            }
            
        }

    };



    
    /**
    * @brief Nodelet-wrapper of the EgocylindricalRangeImageNodelet class
    */
    class EgocylindricalRangeImageNodelet : public nodelet::Nodelet
    {
    public:
      EgocylindricalRangeImageNodelet(){};
      ~EgocylindricalRangeImageNodelet(){}
      
      /**
      * @brief Initialise the nodelet
      *
      * This function is called, when the nodelet manager loads the nodelet.
      */
      virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTNodeHandle();
        ros::NodeHandle pnh = this->getMTPrivateNodeHandle();
        
        // resolve node(let) name
        std::string name = pnh.getUnresolvedNamespace();
        int pos = name.find_last_of('/');
        name = name.substr(pos + 1);
        
        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        propagator_ = std::make_shared<EgoCylinderRangeImageGenerator>(nh, pnh);
        
        // Initialises the controller
        if (propagator_->init())
        {
          NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
        }
        else
        {
          NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
        }
      }
    private:
      std::shared_ptr<EgoCylinderRangeImageGenerator> propagator_;
    };
    

PLUGINLIB_EXPORT_CLASS(egocylindrical::EgocylindricalRangeImageNodelet,
                       nodelet::Nodelet);
// %EndTag(FULLTEXT)%


}

