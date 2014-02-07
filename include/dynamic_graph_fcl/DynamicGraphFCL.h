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
#include <dynamic-graph/factory.h>

#include <dynamic_graph_fcl/URDFParser.h>
#include <dynamic_graph_fcl/TFBroadcaster.h>
#include <dynamic_graph_fcl/SOTCompensator.h>

namespace dynamicgraph {
namespace FCL {
class DynamicGraphFCL : public Entity
{
public:
    static const std::string CLASS_NAME;
    virtual const std::string& getClassName (void) const {
        return CLASS_NAME;
    }
    DynamicGraphFCL(const std::string &inName);
    ~DynamicGraphFCL();


    boost::shared_ptr< SignalPtr <dynamicgraph::Vector, int> > joint_states_in;

    boost::shared_ptr< SignalTimeDependent<dynamicgraph::Vector, int> > joint_states_out;

    void set_collision_joints(const std::string& joint_collision_names);
    void init_collisions(const std::vector<std::string>& joint_collision_names);


    dynamicgraph::Vector& joint_state_update_function(dynamicgraph::Vector& vec, int i);

    // critical update function. For the state of the development this gets triggered for each collision pair twice!
    // optimal solution will be to set a dirty_flag when a recomputation of the FCL distance is actually needed.
    dynamicgraph::Vector& closest_point_update_function(dynamicgraph::Vector& point, int i, std::string& joint_name_1,int& idx, std::string& joint_name_2, int& idy);

    boost::shared_ptr< SignalTimeDependent<dynamicgraph::Vector, int> > debug_point_1_out;
    boost::shared_ptr< SignalTimeDependent<dynamicgraph::Vector, int> > debug_point_2_out;

private:


    const std::string inName_;
    std::vector<std::string> joint_collision_names_;
    int joint_collision_size_;

    boost::shared_ptr< TFBroadcaster> tfBroadcaster_;
    boost::shared_ptr< URDFParser> urdfParser_;
    boost::shared_ptr< SOTCompensator> sotCompensator_;

    std::vector<boost::shared_ptr< SignalPtr <dynamicgraph::Matrix, int> > >op_point_in_vec_;

    // length of this vector has to be joint_collision_names^2
    // idx is determined as it would be in matrix form
    //
    // width * i+ j , where i,j are the index of collision_joint_names.
    // leads to a unique idexing of the collison matrix
    std::vector<boost::shared_ptr< SignalTimeDependent<dynamicgraph::Vector, int> > >collision_matrix_;

    void initCollisionObjects();
    void initSignals();
    void initDebugSignals();

    void fillCollisionMatrix(int idx, int idy);

    void updateURDFParser(const dynamicgraph::Matrix& op_point, int id)const;


};
} // namespace FCL
} // namespace dynamicgraph
#endif // DYNAMIC_GRAPH_FCL_HH
