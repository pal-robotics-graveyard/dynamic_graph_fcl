#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <urdf/model.h>
#include <fcl/collision_object.h>

#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-homogeneous.hh>

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

inline sot::MatrixHomogeneous convertToDG(fcl::Transform3f fcl_matrix){
    sot::MatrixHomogeneous dg_mat;
    dg_mat.setIdentity();

    fcl::Matrix3f r = fcl_matrix.getRotation();
    fcl::Vec3f t = fcl_matrix.getTranslation();

    for (int i = 0; i < 3; ++i) {
        dg_mat.elementAt(i,3) = t[i];
        for (int j = 0; j < 3; ++j) {
            dg_mat.elementAt(i,j) = r(i,j);
        }
    }
    return dg_mat;
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
//    rot.setIdentity();
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

inline tf::Transform transformToTF(const fcl::CollisionObject capsule){
    tf::Transform transform;

    fcl::Transform3f transform_fcl = capsule.getTransform();

    tf::Vector3 pos = tf::Vector3(transform_fcl.getTranslation()[0],transform_fcl.getTranslation()[1],transform_fcl.getTranslation()[2]);
    transform.setOrigin(pos);

    tf::Quaternion quat;
    quat.setW(capsule.getQuatRotation().getW());
    quat.setX(capsule.getQuatRotation().getX());
    quat.setY(capsule.getQuatRotation().getY());
    quat.setZ(capsule.getQuatRotation().getZ());

    transform.setRotation(quat);
    transform.setOrigin(pos);

    return transform;
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

inline tf::Transform transformToTF(const dynamicgraph::Vector& vector)
{
    tf::Transform transform;

    tf::Quaternion quat(0,0,0,1);

    tf::Vector3 pos;
    pos.setValue(vector.elementAt(0),vector.elementAt(1),vector.elementAt(2));

    transform.setRotation(quat);
    transform.setOrigin(pos);

    return transform;
}

inline geometry_msgs::Transform transformToGeometryMsg(const fcl::Transform3f fcl_transform){

    geometry_msgs::Transform transform;
    geometry_msgs::Quaternion quat;
    quat.w = fcl_transform.getQuatRotation().getW();
    quat.x = fcl_transform.getQuatRotation().getX();
    quat.y = fcl_transform.getQuatRotation().getY();
    quat.z = fcl_transform.getQuatRotation().getZ();

    geometry_msgs::Vector3 trans;
    trans.x = fcl_transform.getTranslation()[0];
    trans.y = fcl_transform.getTranslation()[1];
    trans.z = fcl_transform.getTranslation()[2];

    transform.rotation = quat;
    transform.translation = trans;
    return transform;
}

}


}

}
#endif
