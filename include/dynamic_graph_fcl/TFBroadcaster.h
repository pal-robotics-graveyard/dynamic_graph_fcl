#include <tf/transform_broadcaster.h>

namespace dynamicgraph{

namespace FCL{

class TFBroadcaster{

public:
    TFBroadcaster();
    ~TFBroadcaster();


    void sendTransform(const std::string link_name, const tf::Transform transform);


private:

    tf::TransformBroadcaster br_;
};
}
}
