#ifndef URDFPARSER_H
#define URDFPARSER_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <fcl_capsule/collision_object.h>

#include <fcl_capsule/BV/OBBRSS.h>
#include <fcl_capsule/BVH/BVH_model.h>
#include <fcl_capsule/shape/geometric_shapes.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

#include <capsule_msgs/Capsule.h>

//#include <dynamic_graph_fcl/Conversions.h>


namespace dynamicgraph{

namespace fcl{

class URDFParser
{
public:

    /*there is a naming difference between URDF and SoT
      SoT operates on joints in terms of OpPoints.
      URDF Collisionobjects however are links.

      (!) REEM_robots have the convention of not having set a link_origin tag.
      This allows to ignore this difference in convention as the origin of the link equals the origin of the sot-joint
       */
    URDFParser(const std::string& robot_description_param, std::vector<std::string> joint_names);
    ~URDFParser();

    std::map<std::string, boost::shared_ptr<fcl_capsule::CollisionObject> > getCollisionObjects();
    boost::shared_ptr<fcl_capsule::CollisionObject> getCollisionObject(std::string link_name) ;

    void updateLinkPosition(const std::string& link_name,const fcl_capsule::Matrix3f& rot, const fcl_capsule::Vec3f& pos);
    void updateLinkPosition(const std::string& link_name,const fcl_capsule::Transform3f& transform);

    fcl_capsule::Transform3f getOrigin(const std::string& link_name);
    void getClosestPoints(const std::string& link_name_1,const std::string& link_name_2, fcl_capsule::Vec3f& p1, fcl_capsule::Vec3f& p2 );

    bool isEndeffector(const std::string& link_name) const;

    void publishCapsuleMessage(const std::string& link_name);

private:
    ros::NodeHandle nh_;
    urdf::Model model_;

    std::vector<std::string> joint_names_;

    void parseCollisionObjects();
    boost::shared_ptr<fcl_capsule::Transform3f> parseCapsule(const boost::shared_ptr<const urdf::Link>& link, boost::shared_ptr<fcl_capsule::CollisionObject>& collision_object) const;
    boost::shared_ptr<const urdf::Link> getFastLink(const std::string& name) const;
    boost::shared_ptr<const urdf::Joint> getFastJoint(const std::string& name) const;

    std::map<std::string, boost::shared_ptr<fcl_capsule::CollisionObject> > collision_objects_;
    std::map<std::string, boost::shared_ptr<fcl_capsule::Transform3f> > collision_objects_origins_;


    ros::Publisher pub_capsule_;
};

}

}
#endif
