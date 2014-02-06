#ifndef SIGNALHELPER_H
#define SIGNALHELPER_H

#include <boost/shared_ptr.hpp>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

namespace dynamicgraph{

namespace FCL{

namespace SignalHelper{

typedef SignalTimeDependent < dynamicgraph::Matrix, int > SignalTimeMatrix;
typedef SignalTimeDependent < dynamicgraph::Vector, int > SignalTimeVector;
typedef SignalPtr<dynamicgraph::Vector, int > SignalPtrVector;
typedef SignalPtr<dynamicgraph::Matrix, int > SignalPtrMatrix;

inline boost::shared_ptr<SignalTimeVector> createOutputSignalTimeVector(std::string name){

    std::string signal_name = "DynamicGraphFCL(entFCL)::output(vector)::"+name;
    boost::shared_ptr<SignalTimeVector> signal =  boost::shared_ptr<SignalTimeVector>(
                new SignalTimeVector(NULL, signal_name));
    return signal;
}

inline boost::shared_ptr<SignalPtrMatrix> createInputSignalMatrix(std::string name){
    std::string signal_name = "DynamicGraphFCL(entFCL)::input(matrix)::"+name;
    boost::shared_ptr<SignalPtrMatrix> signal = boost::shared_ptr<SignalPtrMatrix> (
                new SignalPtrMatrix(NULL, signal_name));
    return signal;
}
}

}
}
#endif
