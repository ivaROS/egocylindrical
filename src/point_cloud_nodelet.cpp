//
// Created by root on 2/5/18.
//

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>


namespace egocylindrical
{

    namespace utils
    {
        sensor_msgs::PointCloud2::ConstPtr generate_point_cloud(const utils::ECWrapper& points);
    }

    class EgoCylinderPointCloudGenerator
    {
        ros::NodeHandle nh_, pnh_;
        ros::Publisher pc_pub_;
        ros::Subscriber ec_sub_;

        
        
    public:

        EgoCylinderPointCloudGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
                nh_(nh),
                pnh_(pnh)
        {

        }
        
        bool init()
        {
            pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cylindrical", 2);
            ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderPointCloudGenerator::ecPointsCB, this); 
            return true;
        }


    private:
        
        void ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
        {
            ROS_DEBUG("Received EgoCylinderPoints msg");

            if(pc_pub_.getNumSubscribers()>0)
            {
              ros::WallTime start = ros::WallTime::now();
              
              utils::ECWrapper ec_pts(ec_msg);
              
              sensor_msgs::PointCloud2::ConstPtr msg = utils::generate_point_cloud(ec_pts);
              
              ROS_INFO_STREAM("Generating point cloud took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
              

              ROS_DEBUG("publish egocylindrical pointcloud");
              
              pc_pub_.publish(msg);
            }
            
        }

    };

    
    
    
    /**
     * @brief Nodelet-wrapper of the EgocylindricalPointCloud class
     */
    class EgocylindricalPointCloudNodelet : public nodelet::Nodelet
    {
    public:
      EgocylindricalPointCloudNodelet(){};
      ~EgocylindricalPointCloudNodelet(){}
      
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
        propagator_ = std::make_shared<EgoCylinderPointCloudGenerator>(nh, pnh);
        
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
      std::shared_ptr<EgoCylinderPointCloudGenerator> propagator_;
    };
    
    
    PLUGINLIB_EXPORT_CLASS(egocylindrical::EgocylindricalPointCloudNodelet,
                           nodelet::Nodelet);
    // %EndTag(FULLTEXT)%
    
}
