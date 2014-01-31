/*
 * Copyright 2011,
 * Florent Lamiraux
 *
 * CNRS
 *
 * This file is part of sot-test.
 * sot-test is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-test is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-test.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/factory.h>

#include <dynamic_graph_fcl/DynamicGraphFCL.h>

namespace dynamicgraph {
namespace FCL {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicGraphFCL, "DynamicGraphFCL");


DynamicGraphFCL::DynamicGraphFCL(const std::string& inName) :
    Entity(inName)
{
    z_in = boost::shared_ptr< SignalPtr<double, int > >(new SignalPtr<double,int>(NULL, "DynamicGraphFCL("+inName+")::input(vector)::zIN"));
    std::cout << "DynamicGraph is getting setup" << z_in.get() << std::endl;

    a_out = boost::shared_ptr<SignalTimeDependent < double, int > >(new SignalTimeDependent < double, int >(*z_in, "DynamicGraphFCL("+inName+")::output(int)::aOUT"));
    b_out = boost::shared_ptr<SignalTimeDependent < double, int > >(new SignalTimeDependent < double, int >(*z_in, "DynamicGraphFCL("+inName+")::output(int)::bOUT"));
//    b_out("DynamicGraphFCL("+inName+")::ouput(int)::bOUT");

    signalRegistration(*z_in);
    signalRegistration(*a_out);
    signalRegistration(*b_out);

    a_out->setConstant(1.);
    b_out->setConstant(1.);
    z_in->setConstant(4.);
    a_out->setFunction(boost::bind(&DynamicGraphFCL::a_function,
                                        this, _1, _2));
    b_out->setFunction(boost::bind(&DynamicGraphFCL::b_function,
                                        this, _1, _2));
}

DynamicGraphFCL::~DynamicGraphFCL() {}

double& DynamicGraphFCL::a_function(double &d, const int &i){
    std::cout << "a_function got: " << d << "& " << i << std::endl;
    return d;
}
double& DynamicGraphFCL::b_function(double &d, const int &i){

   std::cout << z_in.get() << std::endl;
   const double& zINd =(*z_in)(i);

    std::cout << "b_function got: d" << d << " &i " << i << "& zINd" << zINd << std::endl;
    return d;
}
} // namespace FCL
} // namespace dynamicgraph
