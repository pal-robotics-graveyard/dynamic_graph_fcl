#include <dynamic_graph_fcl/SOTCompensator.h>
#include <tf/transform_datatypes.h>

namespace dynamicgraph{

namespace FCL{


SOTCompensator::SOTCompensator():
    nh_()
{

    if(nh_.ok()){
        std::cerr << "ros initialized" << std::endl;
    }

}

SOTCompensator::~SOTCompensator(){}

void SOTCompensator::initVectorSize(const int size)
{
    compensation_matrix_vec.resize(size);
}

void SOTCompensator::setSOTCompensation(const std::string& joint_name, const int& idx)
{
    double w,x,y,z;
    nh_.param<double>("/sot_controller/"+joint_name+"/quat/w", w, 1);
    nh_.param<double>("/sot_controller/"+joint_name+"/quat/y", x, 0);
    nh_.param<double>("/sot_controller/"+joint_name+"/quat/y", y, 0);
    nh_.param<double>("/sot_controller/"+joint_name+"/quat/z", z, 0);


    std::cerr << "param server was: " << w << ";;"<< x << ";;"<< y << ";;"<< z << ";;"<<std::endl;


    // converting first to TF then to dynamic graph!
    // this is for sure not cool but does the job for now
    // maybe even operate directly in the sot-controller/sot-device where those paramteres are getting created
    // FIX THAT!

    tf::Quaternion quat(x,y,z,w);
    tf::Matrix3x3 rot;
    rot.setRotation(quat);

    // check if tf rotation is the same as the compensation
    // MAYBE THE COPY IS WRONGLY ADDRESSED !!



    boost::shared_ptr<dynamicgraph::Matrix> compensation
            = boost::shared_ptr<dynamicgraph::Matrix>(new dynamicgraph::Matrix(4,4));

    std::cerr << "compensation for joint: " << joint_name << std::endl;
    compensation->setZero();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            compensation->elementAt(i,j) = rot[i][j];
            std::cerr << "sot compensation at: " << i << " " <<j << " = " << compensation->elementAt(i,j) << std::endl;
        }
    }

    // make homogenous
//    compensation->elementAt(0,3) = 0;
//    compensation->elementAt(1,3) = 0;
//    compensation->elementAt(2,3) = 0;
//    compensation->elementAt(3,0) = 0;
//    compensation->elementAt(3,1) = 0;
//    compensation->elementAt(3,2) = 0;
    compensation->elementAt(3,3) = 1;



    compensation_matrix_vec[idx] = compensation;
}


boost::shared_ptr<dynamicgraph::Matrix> SOTCompensator::getSOTCompensation(const int& idx) const
{
    return compensation_matrix_vec[idx];
}

dynamicgraph::Matrix SOTCompensator::applySOTCompensation(const dynamicgraph::Matrix& signal_in, int idx) const
{

    boost::shared_ptr<dynamicgraph::Matrix> compensation = getSOTCompensation(idx);
    return signal_in.multiply(compensation->inverse());
}



}
}
