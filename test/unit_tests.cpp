#include <egocylindrical/ecwrapper.h>


using namespace egocylindrical;
using namespace egocylindrical::utils;

class Tests
{
public:
   
  
    bool point_conversion(ECWrapper& points)
    {
        int num_pts = points.getCols();
        const float* x = (const float*)points.getX();
        const float* y = (const float*)points.getY();
        const float* z = (const float*)points.getZ();
        for(int i = 0; i < num_pts; ++i)
        {
            cv::Point3d p(x[i],y[i],z[i]);
            cv::Point2d pp;
            float range;
            points.project3dToPixelRange(p, pp, range);
            cv::Point3d reproj = range * points.projectPixelTo3dRay(pp);
            double error = cv::norm(p-reproj);
            ROS_INFO_STREAM("Error=" << error);
        }

      return true;
    }
    
    bool point_conversion2(ECWrapper& points)
    {
        int num_pts = points.getCols();
        double factor = 10.0/num_pts;
//         const float* x = (const float*)points.getX();
//         const float* y = (const float*)points.getY();
//         const float* z = (const float*)points.getZ();
        float range=factor;
        for(int i = 0; i < points.getHeight(); ++i)
        {
            for(int j = 0; j < points.getWidth(); ++j)
            {
                cv::Point2d pp(i,j);
//                 double range = points.pixToIdx(pp);
                cv::Point3d reproj = range * points.projectPixelTo3dRay(pp);
                //cv::Point3d p(x[i],y[i],z[i]);
                float proj_range;
                cv::Point2d pp2;
                points.project3dToPixelRange(reproj, pp2, proj_range);
                double range_error = range - proj_range;
                double pix_error = cv::norm(pp-pp2);
                ROS_INFO_STREAM("Pixel0=" << pp<< ", Range0=" << range << ", Point=" << reproj << ", Pixel=" << pp2 << ", Range=" << proj_range << ", Range Error=" << range_error << ", Pix Error=" << pix_error);
                range+=factor;
            }
        }

      return true;
    }
    
    
    ECWrapper::Ptr getPts()
    {
      ECParams params;
      params.height=64;
      params.width=256;
      params.vfov=2;
      params.can_width=128;
      
      auto points = getECWrapper(params);
      
      int num_pts = points->getCols();
      float* x = (float*)points->getX();
      float* y = (float*)points->getY();
      float* z = (float*)points->getZ();
      for(int i = 0; i < num_pts; ++i)
      {

      }

      
      
      return points;
    }
    
    
    bool runTests()
    {
      auto pts = getPts();
      return point_conversion2(*pts);
    }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tester");
    Tests tests;
    tests.runTests();
}


