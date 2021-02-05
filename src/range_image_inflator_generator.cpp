//
// Created by root on 2/5/18.
//

#include <egocylindrical/range_image_inflator_generator.h>
//#include <egocylindrical/range_image_inflator_core.h>
#include <algorithm>

// The below are redundant
#include <egocylindrical/EgoCylinderPoints.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <egocylindrical/ecwrapper.h>

namespace egocylindrical
{



    RangeImageInflatorGenerator::RangeImageInflatorGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        it_(nh_)
    {
        std::cout<<"Egocylindrical Range Image Node Initialized"<<std::endl;

        
        
    }
    
    bool RangeImageInflatorGenerator::init()
    {
        //use_raw_ = false;
        std::string image_topic = "inflated_image";
                
        pnh_.getParam("image_topic", image_topic );
        
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        reconfigure_server_->setCallback(boost::bind(&RangeImageInflatorGenerator::configCB, this, _1, _2));
        

        image_transport::SubscriberStatusCallback image_cb = boost::bind(&RangeImageInflatorGenerator::ssCB, this);
        im_pub_ = it_.advertise(image_topic, 2, image_cb, image_cb);
        
        // Synchronize Image and CameraInfo callbacks
        timeSynchronizer_ = boost::make_shared<synchronizer>(im_sub_, ec_sub_, 2);
        timeSynchronizer_->registerCallback(boost::bind(&RangeImageInflatorGenerator::imgCB, this, _1, _2));
        
        return true;
    }
    
    void RangeImageInflatorGenerator::configCB(const ConfigType &config, uint32_t level)
    {
      WriteLock lock(config_mutex_);
      
      ROS_INFO_STREAM("Updating Range Image Inflator config: num_threads=" << config.num_threads << ", inflation_radius=" << config.inflation_radius);
      config_ = config;
    }
    
    void RangeImageInflatorGenerator::ssCB()
    {
        
        //std::cout << (void*)ec_sub_ << ": " << im_pub_.getNumSubscribers() << std::endl;

        if(im_pub_.getNumSubscribers()>0)
        {
            if((void*)im_sub_.getSubscriber()) //if currently subscribed... no need to do anything
            {
              
            }
            else
            {
                im_sub_.subscribe(it_, "range_image", 2);
                ec_sub_.subscribe(nh_, "egocylindrical_points", 2);
                
                ROS_INFO("RangeImage Inflator Subscribing");
            }  
        }
        else
        {
            im_sub_.unsubscribe();
            ec_sub_.unsubscribe();
            
            ROS_INFO("RangeImage Inflator Unsubscribing");
        }
    }
    
    
    float getInflationAngle(float range, float inflation_radius)
    {
      return std::asin(inflation_radius/range);
    }
    
    int getNumInflationIndices(float range, float scale, float inflation_radius)
    {
      return getInflationAngle(range, inflation_radius) * scale;
    }
    
    template<typename T>
    void inflateRowRegion(T range, int start_ind, int end_ind, T* inflated)
    {
      for(int ind = start_ind; ind < end_ind; ind++)
      {
        if(inflated[ind]< range)
        {
        }
        else
        {
          inflated[ind] = range;
        }
      }
    }
    
    template<typename T>
    void inflateRow(const T* ranges, int width, float scale, float inflation_radius, T* inflated)
    {
      for(int i = 0; i < width; i++)
      {
        T range = ranges[i];

        if(range!=range)
        {
          continue;
        }
        int inflation_size = getNumInflationIndices(range, scale, inflation_radius);
        
        int start_ind = i - inflation_size;
        int end_ind = i + inflation_size + 1;
        
        T modified_range = range - inflation_radius;
        
        if(start_ind < 0)
        {
          inflateRowRegion(modified_range, start_ind+width, width, inflated);
          inflateRowRegion(modified_range, 0, end_ind, inflated);
        }
        else if(end_ind >= width)
        {
          inflateRowRegion(modified_range, start_ind, width, inflated);
          inflateRowRegion(modified_range, 0, end_ind-width, inflated);
        }
        else
        {
          inflateRowRegion(modified_range, start_ind, end_ind, inflated);
        }
      }
    }
    
    template<typename T>
    void inflateHorizontally(const T* ranges, int height, int width, float scale, float inflation_radius, int num_threads, T* inflated)
    {
      for(int j = 0; j < height; j++)
      {
        inflateRow(ranges+j*width, width, scale, inflation_radius, inflated+j*width);
      }
    }
    
    template<typename T>
    void inflateColumn(int height, int width, float scale, float inflation_radius, T* inflated)
    {
      for(int j = 0; j < height; j++)
      {
        T range = inflated[j*width];
        
        if(range!=range)
        {
          continue;
        }
        int inflation_size = getNumInflationIndices(range, scale, inflation_radius);
        
        int start_ind = std::max(j - inflation_size, 0);
        int end_ind = std::min(j + inflation_size + 1, height);
                
        for(int k = start_ind; k < end_ind; k++)
        {
          if(inflated[k*width]< range)
          {
          }
          else
          {
            inflated[k*width] = range;
          }
        }
      }
    }
    
    template<typename T>
    void inflateVertically(int height, int width, float scale, float inflation_radius, int num_threads, T* inflated)
    {
      for(int i = 0; i < width; i++)
      {
        inflateColumn(height, width, scale, inflation_radius, inflated+i);
      }
    }
    
    template<typename T>
    void inflateRangeImage(const T* ranges, const utils::ECConverter& converter, float inflation_radius, int num_threads, T* inflated)
    {
      int height = converter.getHeight();
      int width = converter.getWidth();
      
      float hscale = converter.getHScale();
      float vscale = converter.getVScale();
      
      inflateHorizontally(ranges, height, width, hscale, inflation_radius, num_threads, inflated);
      inflateVertically(height, width, vscale, inflation_radius, num_threads, inflated);
    }
    
    sensor_msgs::Image::ConstPtr getInflatedRangeImageMsg(const EgoCylinderPoints::ConstPtr& ec_msg, const sensor_msgs::Image::ConstPtr& range_msg, float inflation_radius, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
    {
      sensor_msgs::ImagePtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : boost::make_shared<sensor_msgs::Image>();
      
      sensor_msgs::Image &new_msg = *new_msg_ptr;
      new_msg.header = range_msg->header;
      new_msg.height = range_msg->height;
      new_msg.width = range_msg->width;
      new_msg.encoding = range_msg->encoding;
      new_msg.is_bigendian = range_msg->is_bigendian;
      new_msg.step = range_msg->step;
      
      size_t size = new_msg.step * new_msg.height;
      
      //ros::WallTime start = ros::WallTime::now();
      
      new_msg.data.resize(size);
      //cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::TYPE_32FC1, new_im_).toImageMsg();
      
      
      // ROS_INFO_STREAM_NAMED("timing","Allocating image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
      
      
      utils::ECConverter converter;
      converter.fromCameraInfo(ec_msg);
      
      float* inflated_ranges = (float*)new_msg.data.data();
      std::fill(inflated_ranges, inflated_ranges+converter.getCols(), utils::dNaN);
      
      const float* ranges = (float*)range_msg->data.data();
      
      inflateRangeImage<float>(ranges, converter, inflation_radius, num_threads, inflated_ranges);
      
      return new_msg_ptr;
    }

    
    void RangeImageInflatorGenerator::imgCB(const sensor_msgs::Image::ConstPtr& range_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
    {
        ROS_DEBUG("Received EgoCylinderPoints msg");
        
        // This may be redundant now
        if(im_pub_.getNumSubscribers() > 0)
        {

          ros::WallTime start = ros::WallTime::now();
          
          utils::ECWrapper ec_pts(ec_msg);
                
          ConfigType config;
          {
            ReadLock lock(config_mutex_);
            config = config_;
          }
          sensor_msgs::Image::ConstPtr image_ptr = getInflatedRangeImageMsg(ec_msg, range_msg, config.inflation_radius, config.num_threads, preallocated_msg_);

          ROS_DEBUG_STREAM_NAMED("timing","Inflating range image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          ROS_DEBUG("publish egocylindrical image");
          
          im_pub_.publish(image_ptr);
          
          start = ros::WallTime::now();
          preallocated_msg_= boost::make_shared<sensor_msgs::Image>();
          preallocated_msg_->data.resize(image_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
          ROS_DEBUG_STREAM_NAMED("timing","Preallocating image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          
        }
        
    }




}
