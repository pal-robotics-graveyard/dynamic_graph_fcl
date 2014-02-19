#include <tf/transform_broadcaster.h>

namespace dynamicgraph{

namespace FCL{

class TFBroadcaster{

public:
    TFBroadcaster();
    ~TFBroadcaster();

    bool replace(std::string& str, const std::string& from, const std::string& to);
    void sendTransform(const std::string link_name, const tf::Transform transform);
    void sendTransform(const std::string link_name, const tf::Transform transform, std::string parent);


private:

    tf::TransformBroadcaster br_;
};
}
}
