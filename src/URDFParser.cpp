
#include <dynamic_graph_fcl/URDFParser.h>

#include <fcl/distance.h>
#include <dynamic_graph_fcl/Conversions.h>

namespace dynamicgraph{
namespace FCL{

URDFParser::URDFParser(const std::string &robot_description_param, const std::vector<std::string> joint_names)
    : nh_(),
      model_(),
      joint_names_(joint_names)
{
    model_.initParam(robot_description_param);
    parseCollisionObjects();

    pub_capsule_ = nh_.advertise<capsule_msgs::Capsule>("dynamic_graph_fcl/capsules",100);

    std::cout << "amount of parsed collisionobjects: " << collision_objects_.size() <<std::endl;
}

URDFParser::~URDFParser(){}

void URDFParser::parseCollisionObjects()
{

    for (int var = 0; var < joint_names_.size(); ++var) {

        std::string link_name;
        if (joint_names_[var].find("base") != std::string::npos){
            link_name = "base_link";
        }else{
            link_name = model_.getJoint( joint_names_[var] )->child_link_name;
        }

        boost::shared_ptr<const urdf::Link> link = getFastLink(link_name);
        boost::shared_ptr<fcl::CollisionObject> collision_object;

        /* for collision detection purposes every cylinder gets parsed as an capsule */
        if (link->collision->geometry->type == urdf::Geometry::CYLINDER){
            collision_objects_origins_[joint_names_[var]] = parseCapsule(link, collision_object);
            std::cout << "capsule succesfully parsed" << std::endl;
        }
        else{
            std::cerr << "module not parsed, because no cyclinder/capsule: "
                      << joint_names_[var] << std::endl;
            return;
        }
        collision_objects_[joint_names_[var]] = collision_object;
    }
}


boost::shared_ptr<fcl::Transform3f> URDFParser::parseCapsule(
        const boost::shared_ptr<const urdf::Link>& link,
        boost::shared_ptr<fcl::CollisionObject>& collision_object) const
{

    boost::shared_ptr<urdf::Cylinder> collisionGeometry =
            boost::dynamic_pointer_cast<urdf::Cylinder>(
                link->collision->geometry);

    urdf::Pose origin_tmp = link->collision->origin;

    std::cerr << "collision origin z for "<< link->name
              << "=" << link->collision->origin.position.z << std::endl;

    //check that!
    // This is heavily URDF dependent,
    // as this depends on my capsule decomposition, check the according URDF

    // CHECK HERE AGAIN!!!
    int signz = 1;
    if (link->collision->origin.position.z < 0){
        signz = -1;
    }

    double r,p,y;
    if (link->name.find("arm_right_2") != std::string::npos){
        origin_tmp.rotation.getRPY(r,p,y);
        std::cerr << "arm_right_2 " << r << " " << p << " "<< y << std::endl;
    }else if(link->name.find("arm_right_3") != std::string::npos){
        origin_tmp.rotation.getRPY(r,p,y);
        std::cerr << "arm_right_3 " << r << " " << p << " "<< y << std::endl;
    }
    origin_tmp.position.z -= signz* (collisionGeometry->length/2);

    boost::shared_ptr<fcl::Capsule> capsule =
            boost::shared_ptr<fcl::Capsule>(
                new fcl::Capsule(collisionGeometry->radius, signz*collisionGeometry->length));

    std::cerr << "capsule data; r: " << collisionGeometry->radius
              << " l: " << collisionGeometry->length << std::endl;
    std::cerr << "with origin shift: " << origin_tmp.position.x << ","
              << origin_tmp.position.y << ","<< origin_tmp.position.z
              << std::endl;

    collision_object.reset(new fcl::CollisionObject(capsule));
    return Conversions::convertToFCLTransform(origin_tmp);
}

boost::shared_ptr<fcl::CollisionObject> URDFParser::getCollisionObject(std::string link_name)
{
    return collision_objects_[link_name];
}

std::map<std::string, boost::shared_ptr<fcl::CollisionObject> > URDFParser::getCollisionObjects()
{
    return collision_objects_;
}

boost::shared_ptr<const urdf::Link> URDFParser::getFastLink(const std::string& name) const
{
    std::map<std::string, boost::shared_ptr<urdf::Link> >::const_iterator it;
    it = model_.links_.find(name);
    if ( it == model_.links_.end())
        return boost::shared_ptr<const urdf::Link>();
    else
        return it->second;
}

boost::shared_ptr<const urdf::Joint> URDFParser::getFastJoint(const std::string& name) const
{
    std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator it;
    it = model_.joints_.find(name);
    if ( it == model_.joints_.end())
        return boost::shared_ptr<const urdf::Joint>();
    else
        return it->second;
}

bool URDFParser::isEndeffector(const std::string& joint_name) const
{
    boost::shared_ptr<const urdf::Joint> joint = getFastJoint(joint_name);
    if (joint->type == urdf::Joint::FIXED){
        return true;
    }
    return false;
}

void URDFParser::updateLinkPosition(const std::string &link_name,
                                    const fcl::Matrix3f& rot,
                                    const fcl::Vec3f& pos)
{
    fcl::Transform3f transform;
    transform.setTranslation(pos);
    transform.setRotation(rot);

    updateLinkPosition(link_name, transform);
}


void URDFParser::updateLinkPosition(const std::string& link_name,
                                    const fcl::Transform3f& transform)
{
    boost::shared_ptr<fcl::CollisionObject> collision_object
            = collision_objects_[link_name];
    boost::shared_ptr<fcl::Transform3f> collision_origin
            = collision_objects_origins_[link_name];

    collision_object->setTransform( transform   *(*collision_origin));
}

fcl::Transform3f URDFParser::getOrigin(const std::string& link_name)
{
    return *collision_objects_origins_[link_name];
}

void URDFParser::getClosestPoints(const std::string &link_name_1,
                                  const std::string &link_name_2,
                                  fcl::Vec3f& p1, fcl::Vec3f& p2)
{

    boost::shared_ptr<fcl::CollisionObject> collision_objects_1
            = collision_objects_[link_name_1];
    boost::shared_ptr<fcl::CollisionObject> collision_objects_2
            = collision_objects_[link_name_2];

    fcl::DistanceRequest request;
    request.gjk_solver_type = fcl::GST_INDEP;
    request.enable_nearest_points = true;

    // result will be returned via the collision result structure
    fcl::DistanceResult result;

    // perform distance test
    fcl::distance(collision_objects_1.get(),
                  collision_objects_2.get(),
                  request, result);
    p1 = result.nearest_points[0];
    p2 = result.nearest_points[1];

}


void URDFParser::publishCapsuleMessage(const std::string& link_name) {

    boost::shared_ptr<const fcl::CollisionObject> cobject = collision_objects_[link_name];

    const boost::shared_ptr<const fcl::Capsule> capsule
            = boost::dynamic_pointer_cast<const fcl::Capsule> (cobject->collisionGeometry());

    geometry_msgs::Transform origin = Conversions::transformToGeometryMsg(getOrigin(link_name));

    capsule_msgs::Capsule message;
    message.linkname = link_name;
    message.radius = (float)capsule->radius;
    message.length = (float)capsule->lz;
    message.origin = origin;

    pub_capsule_.publish(message);

}


}
}
