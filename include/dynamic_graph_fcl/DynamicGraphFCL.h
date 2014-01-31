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

#ifndef DYNAMIC_GRAPH_FCL_HH
#define DYNAMIC_GRAPH_FCL_HH

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/entity.h>
//#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>

namespace dynamicgraph {
namespace FCL {
class DynamicGraphFCL : public Entity
{
public:
    static const std::string CLASS_NAME;
    virtual const std::string& getClassName (void) const {
        return CLASS_NAME;
    }

    DynamicGraphFCL(const std::string& inName);
    ~DynamicGraphFCL();


    boost::shared_ptr< SignalPtr <double, int> > z_in;


    boost::shared_ptr< SignalTimeDependent<double, int> > a_out;
    boost::shared_ptr< SignalTimeDependent<double, int> > b_out;

    double& a_function(double &d, const int& i);
    double& b_function(double &d, const int& i);
};
} // namespace FCL
} // namespace dynamicgraph
#endif // DYNAMIC_GRAPH_FCL_HH
