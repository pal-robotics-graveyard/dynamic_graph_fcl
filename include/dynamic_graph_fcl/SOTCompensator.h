#include <ros/ros.h>
#include <dynamic-graph/linear-algebra.h>


namespace dynamicgraph{

namespace FCL{


/* The DG/SOT respective the JRL-Dynamics uses a convention of rotating each joint around
                X- AXIS

    This forces a de-transformation according to the URDF
    Mandatorily this has to be changed according the each URDF or Robot

    Get rid of JRL-Dynamics would make this obsolete!
*/
class SOTCompensator{

public:
    SOTCompensator();
    ~SOTCompensator();


    void initVectorSize(const int size);
    boost::shared_ptr<dynamicgraph::Matrix> getSOTCompensation(const int& idx) const;
    void setSOTCompensation(const std::string& joint_name, const int& idx);
    dynamicgraph::Matrix applySOTCompensation(const dynamicgraph::Matrix& signal_in, int idx) const;


private:

    ros::NodeHandle nh_;
    std::vector<boost::shared_ptr<dynamicgraph::Matrix> > compensation_matrix_vec;

};
}
}
