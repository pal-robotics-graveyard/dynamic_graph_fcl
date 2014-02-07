#include <dynamic_graph_fcl/TFBroadcaster.h>
#include <tf/transform_datatypes.h>

namespace dynamicgraph{

namespace FCL{


TFBroadcaster::TFBroadcaster():
    br_()
{}

TFBroadcaster::~TFBroadcaster(){}


void TFBroadcaster::sendTransform(const std::string link_name,
                                  const tf::Transform transform){

    // default for parent link should be "base_link"
    br_.sendTransform(
                tf::StampedTransform(transform, ros::Time::now(), "base_link",
                                     "entFCL_"+link_name));
}


}
}
