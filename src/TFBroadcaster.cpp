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
                                    link_name));
}

bool TFBroadcaster::replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

void TFBroadcaster::sendTransform(const std::string link_name,
                                  const tf::Transform transform, std::string parent){

    // stupid hack because DG deals with joints rather then link
    // for debugging purposes just replace "joint" with "link"

    // default for parent link should be "base_link"
    br_.sendTransform(
                tf::StampedTransform(transform, ros::Time::now(), parent,
                                     link_name));
}



}
}
