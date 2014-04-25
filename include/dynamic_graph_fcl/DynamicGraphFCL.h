/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012-2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Karsten Knese
 * @date 5.3.2014
 */

#ifndef DYNAMIC_GRAPH_FCL_HH
#define DYNAMIC_GRAPH_FCL_HH

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/entity.h>
//#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/factory.h>
#include <sot/core/matrix-homogeneous.hh>


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

    // exposed as python functions
    // for backward compability
    void setCollisionJoints(const std::string& joint_collision_names);
    // when adding joints in a loop
    void addCollisionJoint(const std::string& joint_name);
    void finalizeSignals();

    // critical update function. For the state of the development this gets triggered for each collision pair twice!
    // optimal solution will be to set a dirty_flag when a recomputation of the FCL distance is actually needed.
    sot::MatrixHomogeneous& closestPointUpdate(
            sot::MatrixHomogeneous& point,
            int i, std::string& joint_name_1,int& idx,
            std::string& joint_name_2, int& idy);

    boost::shared_ptr< SignalTimeDependent<dynamicgraph::Vector, int> > debug_point_1_out;
    boost::shared_ptr< SignalTimeDependent<dynamicgraph::Vector, int> > debug_point_2_out;

    void enableTF(const bool& enable){
        tf_topic_enabled_ = enable;
    }

    void enableCapsuleTopic(const bool& enable){
        ros_topic_enabled_ = enable;
    }

    bool tf_topic_enabled_;
    bool ros_topic_enabled_;
private:


    const std::string inName_;
    std::vector<std::string> joint_collision_names_;
    int joint_collision_size_;

    boost::shared_ptr< TFBroadcaster> tfBroadcaster_;
    boost::shared_ptr< URDFParser> urdfParser_;
    boost::shared_ptr< SOTCompensator> sotCompensator_;

    std::vector<boost::shared_ptr< SignalPtr <sot::MatrixHomogeneous, int> > >op_point_in_vec_;


    // length of this vector has to be joint_collision_names^2
    // idx is determined as it would be in matrix form
    //
    // width * i+ j , where i,j are the index of collision_joint_names.
    // leads to a unique idexing of the collison matrix
    std::vector<boost::shared_ptr<
        SignalTimeDependent<sot::MatrixHomogeneous, int> > >collision_matrix_;
    std::vector<
        boost::shared_ptr< SignalTimeDependent<dynamicgraph::Vector, int> > >oppoint_transformations_;

    int getMatrixIndex(int idx, int idy){
        return idx*joint_collision_size_ + idy;
    }


    void initCollisionObjects();
    void initSignals();
    void initDebugSignals();

    void fillCollisionMatrix(int idx, int idy);

    void updateURDFParser(const dynamicgraph::Matrix& op_point, int id)const;

};
} // namespace FCL
} // namespace dynamicgraph
#endif // DYNAMIC_GRAPH_FCL_HH
