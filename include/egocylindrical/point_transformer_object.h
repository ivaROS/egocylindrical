#ifndef EGOCYLINDRICAL_POINT_TRANSFORMER_OBJECT_H
#define EGOCYLINDRICAL_POINT_TRANSFORMER_OBJECT_H

#include <geometry_msgs/TransformStamped.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace egocylindrical
{
    namespace utils
    {

        class PointTransformerObject
        {
            float r0,r1,r2,r3,r4,r5,r6,r7,r8,t0,t1,t2;
            
            public:
            PointTransformerObject()
            {}
            
            PointTransformerObject(const geometry_msgs::TransformStamped& trans)
            {
                tf::Quaternion rotationQuaternion(trans.transform.rotation.x,
                                                  trans.transform.rotation.y,
                                                  trans.transform.rotation.z,
                                                  trans.transform.rotation.w);
                        
                        
                tf::Matrix3x3 tempRotationMatrix = tf::Matrix3x3(rotationQuaternion);
                
                r0 = (float) tempRotationMatrix[0].getX();
                r1 = (float) tempRotationMatrix[0].getY();
                r2 = (float) tempRotationMatrix[0].getZ();
                r3 = (float) tempRotationMatrix[1].getX();
                r4 = (float) tempRotationMatrix[1].getY();
                r5 = (float) tempRotationMatrix[1].getZ();
                r6 = (float) tempRotationMatrix[2].getX();
                r7 = (float) tempRotationMatrix[2].getY();
                r8 = (float) tempRotationMatrix[2].getZ();
                
                t0 = trans.transform.translation.x;
                t1 = trans.transform.translation.y;
                t2 = trans.transform.translation.z;
            }
            
            template <typename T>
            void transform(T in_x, T in_y, T in_z, T& out_x, T& out_y, T& out_z) const
            {
                out_x = r0 * in_x + r1 * in_y + r2 * in_z + t0;
                out_y = r3 * in_x + r4 * in_y + r5 * in_z + t1;
                out_z = r6 * in_x + r7 * in_y + r8 * in_z + t2;
            }
            
            template <typename P>
            void transform(const P& p_in, P& p_out) const
            {
                transform(p_in.x, p_in.y, p_in.z, p_out.x, p_out.y, p_out.z);
            }
            
            template <typename P>
            P transform(const P& p_in) const
            {
                P p_out;
                transform(p_in, p_out);
                return p_out;
            }

        };
      
    } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_POINT_TRANSFORMER_OBJECT_H
