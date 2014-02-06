#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <urdf/model.h>
#include <fcl/collision_object.h>

#include <dynamic-graph/linear-algebra.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_datatypes.h>

namespace dynamicgraph{

namespace FCL{

namespace Conversions{

inline void split(std::vector<std::string> &tokens, const std::string &text, char sep) {
    int start = 0, end = 0;
    while ((end = text.find(sep, start)) != std::string::npos) {
        tokens.push_back(text.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(text.substr(start));
}

inline fcl::Vec3f extractFCLPosition(dynamicgraph::Matrix m){

    fcl::Vec3f pos;
    pos[0] = m.elementAt(0,3);
    pos[1] = m.elementAt(1,3);
    pos[2] = m.elementAt(2,3);

    return pos;
}

inline fcl::Matrix3f extractFCLRotation(dynamicgraph::Matrix m){

    fcl::Matrix3f rot;
    rot(0,0) = m.elementAt(0,0);
    rot(0,1) = m.elementAt(0,1);
    rot(0,2) = m.elementAt(0,2);

    rot(1,0) = m.elementAt(1,0);
    rot(1,1) = m.elementAt(1,1);
    rot(1,2) = m.elementAt(1,2);

    rot(2,0) = m.elementAt(2,0);
    rot(2,1) = m.elementAt(2,1);
    rot(2,2) = m.elementAt(2,2);

    return rot;

}

inline dynamicgraph::Vector convertToDG(fcl::Vec3f fcl_vec){
    dynamicgraph::Vector dg_vec(3);
    dg_vec(0) = fcl_vec[0];
    dg_vec(1) = fcl_vec[1];
    dg_vec(2) = fcl_vec[2];

    return dg_vec;

}

inline boost::shared_ptr<fcl::Transform3f> convertToFCLTransform(const dynamicgraph::Matrix& matrix){
    boost::shared_ptr<fcl::Transform3f> fcl_transform = boost::shared_ptr<fcl::Transform3f>(
                new fcl::Transform3f());

    fcl::Vec3f trans;
    trans.setValue(matrix.elementAt(0,3),matrix.elementAt(1,3),matrix.elementAt(2,3) );

    fcl::Matrix3f rot;
    rot.setValue(
                matrix.elementAt(0,0),
                matrix.elementAt(0,1),
                matrix.elementAt(0,2),

                matrix.elementAt(1,0),
                matrix.elementAt(1,1),
                matrix.elementAt(1,2),

                matrix.elementAt(2,0),
                matrix.elementAt(2,1),
                matrix.elementAt(2,2)

                );

    fcl_transform->setTranslation(trans);
    fcl_transform->setRotation(rot);

    return fcl_transform;
}

inline boost::shared_ptr<fcl::Transform3f> convertToFCLTransform(const urdf::Pose& urdf_pose){

    boost::shared_ptr<fcl::Transform3f> fcl_transform = boost::shared_ptr<fcl::Transform3f>(
                new fcl::Transform3f());
    fcl::Vec3f trans;
    trans.setValue(urdf_pose.position.x,urdf_pose.position.y,urdf_pose.position.z);
    fcl_transform->setTranslation(trans);

    fcl::Matrix3f rot;
    rot.setIdentity();
    double r,p,y;
    urdf_pose.rotation.getRPY(r,p,y);
    rot.setEulerYPR(y,p,r);

    fcl_transform->setRotation(rot);

//    // check that the matrix results in the same
//    double fcl_w = fcl_transform->getQuatRotation().getW();
//    double fcl_x = fcl_transform->getQuatRotation().getX();
//    double fcl_y = fcl_transform->getQuatRotation().getY();
//    double fcl_z = fcl_transform->getQuatRotation().getZ();

//    double urdf_w, urdf_x, urdf_y, urdf_z;
//    urdf_pose.rotation.getQuaternion(urdf_x,urdf_y, urdf_z, urdf_w);
//    std::cerr << "fcl quat : " << fcl_x << " "<< fcl_y << " "<< fcl_z << " "<< fcl_w << " " << std::endl;
//    std::cerr << "urdf quat : " << urdf_x << " "<< urdf_y << " "<< urdf_z << " "<< urdf_w << " "<< std::endl;

    return fcl_transform;
}

/* The DG/SOT respective the JRL-Dynamics uses a convention of rotating each joint around
                X- AXIS

    This forces a de-transformation according to the URDF
    Mandatorily this has to be changed according the each URDF or Robot

    Get rid of JRL-Dynamics would make this obsolete!
*/
inline dynamicgraph::Matrix sot_rotation_fix(const dynamicgraph::Matrix in){
    dynamicgraph::Matrix sot_compensation(4,4);


//    sot_compensation(0,0) = 0;
//    sot_compensation(0,1) = 0;
//    sot_compensation(0,2) = -1;
//    sot_compensation(0,3) = 0;

//    sot_compensation(1,0) = 0;
//    sot_compensation(1,1) = 1;
//    sot_compensation(1,2) = 0;
//    sot_compensation(1,3) = 0;

//    sot_compensation(2,0) = 1;
//    sot_compensation(2,1) = 0;
//    sot_compensation(2,2) = 0;
//    sot_compensation(2,3) = 0;

//    sot_compensation(3,0) = 0;
//    sot_compensation(3,1) = 0;
//    sot_compensation(3,2) = 0;
//    sot_compensation(3,3) = 1;

    sot_compensation(0,0) = 0;
    sot_compensation(0,1) = 1;
    sot_compensation(0,2) = 0;
    sot_compensation(0,3) = 0;

    sot_compensation(1,0) = 0;
    sot_compensation(1,1) = 0;
    sot_compensation(1,2) = 1;
    sot_compensation(1,3) = 0;

    sot_compensation(2,0) = 1;
    sot_compensation(2,1) = 0;
    sot_compensation(2,2) = 0;
    sot_compensation(2,3) = 0;

    sot_compensation(3,0) = 0;
    sot_compensation(3,1) = 0;
    sot_compensation(3,2) = 0;
    sot_compensation(3,3) = 1;

    return in.multiply(sot_compensation.inverse());

}
inline fcl::Transform3f sot_rotation_fix(fcl::Transform3f in){
    fcl::Transform3f sot_compensation;
    fcl::Matrix3f sot_rot(0, 1, 0,
                          0, 0, 1,
                          1, 0, 0);
    fcl::Vec3f sot_trans(0, 0, 0);
    sot_compensation.setRotation(sot_rot);
    sot_compensation.setTranslation(sot_trans);

    return in*sot_compensation;
}

inline tf::Transform transformToTF(const dynamicgraph::Matrix& matrix){
    tf::Transform transform;

    tf::Matrix3x3 rot;
    rot.setValue(
                matrix.elementAt(0,0),
                matrix.elementAt(0,1),
                matrix.elementAt(0,2),

                matrix.elementAt(1,0),
                matrix.elementAt(1,1),
                matrix.elementAt(1,2),

                matrix.elementAt(2,0),
                matrix.elementAt(2,1),
                matrix.elementAt(2,2)

    );

    tf::Vector3 pos;
    pos.setValue(matrix.elementAt(0,3),matrix.elementAt(1,3),matrix.elementAt(2,3));

    tf::Quaternion quat;
    rot.getRotation(quat);


    // check that!!
    transform.setRotation(quat);
    transform.setOrigin(pos);

    return transform;
}

}


}

}
#endif
