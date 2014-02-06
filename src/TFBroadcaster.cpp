#include <dynamic_graph_fcl/TFBroadcaster.h>
#include <tf/transform_datatypes.h>

namespace dynamicgraph{

namespace FCL{


TFBroadcaster::TFBroadcaster():
    br_()
{}

TFBroadcaster::~TFBroadcaster(){}


void TFBroadcaster::sendTransform(const std::string link_name, const std::string parent_link, const tf::Transform transform){

    // default for parent link should be "base_link"
    br_.sendTransform(
                tf::StampedTransform(transform, ros::Time::now(), parent_link,
                                     "sot_origin"+link_name));
}


}
}
