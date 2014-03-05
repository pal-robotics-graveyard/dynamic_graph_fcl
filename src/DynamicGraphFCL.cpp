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

#include <math.h>
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/factory.h>

#include <dynamic_graph_fcl/DynamicGraphFCL.h>
#include <dynamic_graph_fcl/Conversions.h>
#include <dynamic_graph_fcl/SignalHelper.h>

#include <sot/core/matrix-rotation.hh>

#include <ros/ros.h>



namespace dynamicgraph {
namespace FCL {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicGraphFCL, "DynamicGraphFCL");

typedef SignalTimeDependent < sot::MatrixHomogeneous, int > SignalTimeMatrix;
typedef SignalTimeDependent < dynamicgraph::Vector, int > SignalTimeVector;
typedef SignalPtr<dynamicgraph::Vector, int > SignalPtrVector;
typedef SignalPtr<sot::MatrixHomogeneous, int > SignalPtrMatrix;

DynamicGraphFCL::DynamicGraphFCL(const std::string &inName):
    Entity(inName),
    inName_(inName),
    tfBroadcaster_(new TFBroadcaster()),
    sotCompensator_(new SOTCompensator())
{
    std::string docstring;
    docstring =
            "\n"
            "    Initializes collisions for fcl\n"
            "      takes a string of joint names (separated by :) for collision objects \n"
            "\n";
    addCommand(std::string("set_collision_joints"),
               new dynamicgraph::command::Setter<DynamicGraphFCL, std::string >
               (*this, &DynamicGraphFCL::set_collision_joints, docstring));

}

DynamicGraphFCL::~DynamicGraphFCL() {}

/// using a split for a continous string,
//because ATM I didn't find any other method for passing a vector of strings into an entity
void DynamicGraphFCL::set_collision_joints(const std::string &joint_collision_names)
{
    Conversions::split(joint_collision_names_, joint_collision_names, ':');
    joint_collision_size_ = joint_collision_names_.size();
    sotCompensator_->initVectorSize(joint_collision_size_);

    std::cerr << "amount of collison links found: " << joint_collision_size_ << std::endl;

    initCollisionObjects();
    initSignals();
    initDebugSignals();
}

void DynamicGraphFCL::initCollisionObjects()
{
    // give the sotcompensator pointer to urdf for applying to the urdf origin
    urdfParser_.reset(new URDFParser("robot_description", joint_collision_names_));
    urdfParser_->getCollisionObjects();
}

/// Signal Declaration
void DynamicGraphFCL::initSignals()
{

    /// Allocate one input signal for each collision object !!
    op_point_in_vec_.resize(joint_collision_size_);
    std::cerr << "op_vec initiliazed" << std::endl;

    // Allocate n^2 output signal for each collision pair
    collision_matrix_.resize((joint_collision_size_*joint_collision_size_));
    std::cerr << "matrix initiliazed" << std::endl;

    oppoint_transformations_.resize(joint_collision_size_ * joint_collision_size_);
    std::cerr << "oppoint transformations initialized" << std::endl;

    for (int joint_idx = 0; joint_idx < joint_collision_size_; ++joint_idx) {

        // for each collision object create one input signal to update the transform of the capsule/mesh
        op_point_in_vec_[joint_idx] =
                SignalHelper::createInputSignalMatrix(joint_collision_names_[joint_idx]);
        signalRegistration(*op_point_in_vec_[joint_idx]);

        // load the respective sot_compensation from the ros_param server
        // HERe!
        sotCompensator_->setSOTCompensation(joint_collision_names_[joint_idx], joint_idx);

        std::cerr << "input signal registrated "
                  << joint_collision_names_[joint_idx] << std::endl;
        std::cerr << "input signal registrated "
                  << "oppoint_"+joint_collision_names_[joint_idx] << std::endl;
    }

    // !! NOTE
    // this might be optimized as one collision-computation is birectional.
    // This means that e.g. arm_left & arm_right results in the same computation as arm_right & arm_left
    // one quick solution can be by introducing a dirty-flag, set or reset when one transform of those two pairs is updated
    //
    // compute arm_left & arm_right --> compute both points + return point[0] + reset dirty-flag
    // compute arm_right & arm_left --> return point[1]
    // update arm_right OR arm_left transform --> set dirty-flag

    // also cross-initialize all pairs of collision, excluding a pair of itself
    // IGNORING THE ABOVE NOTE!
    for (int joint_idx = 0; joint_idx < joint_collision_size_; ++joint_idx) {
        for (int joint_idy = 0; joint_idy < joint_collision_size_; ++joint_idy) {
            if (joint_idy != joint_idx){

                fillCollisionMatrix(joint_idx, joint_idy);
            }
        }
    }
}

void DynamicGraphFCL::fillCollisionMatrix(int idx, int idy)
{

    std::string joint_name_1 = joint_collision_names_[idx];
    std::string joint_name_2 = joint_collision_names_[idy];

    int collision_matrix_idx = getMatrixIndex(idx, idy);

    boost::shared_ptr<SignalTimeMatrix> collisionSignal =
            SignalHelper::createOutputSignalTimeMatrix(joint_name_1+joint_name_2);

    collisionSignal->addDependency(*op_point_in_vec_[idx]);
    collisionSignal->addDependency(*op_point_in_vec_[idy]);
    collisionSignal->setFunction(boost::bind(&DynamicGraphFCL::closest_point_update_function,
                                             this, _1, _2, joint_name_1, idx, joint_name_2, idy));

    signalRegistration(*collisionSignal);
    collision_matrix_[collision_matrix_idx] = collisionSignal;

    oppoint_transformations_[collision_matrix_idx] =
            SignalHelper::createOutputSignalTimeVector(
                "oppoint_"+joint_collision_names_[idx]+joint_collision_names_[idy]);
    signalRegistration(*oppoint_transformations_[collision_matrix_idx]);
}

void DynamicGraphFCL::initDebugSignals()
{
    std::string debug_point_1
            = "DynamicGraphFCL("+inName_+")::output(int)::debugPoint1";
    debug_point_1_out
            = boost::shared_ptr<SignalTimeVector>(
                new SignalTimeVector(NULL, debug_point_1));
    signalRegistration(*debug_point_1_out);

    std::string debug_point_2
            = "DynamicGraphFCL("+inName_+")::output(int)::debugPoint2";
    debug_point_2_out
            = boost::shared_ptr<SignalTimeVector>(
                new SignalTimeVector(NULL, debug_point_2));
    signalRegistration(*debug_point_2_out);

}

/// Update function for the ouput signals
sot::MatrixHomogeneous& DynamicGraphFCL::closest_point_update_function(
        sot::MatrixHomogeneous& point, int i,
        std::string &joint_name_1, int &idx,
        std::string &joint_name_2, int &idy)
{
    if (joint_collision_names_[idx] != joint_name_1){
        std::cerr << "SOMETHING'S BRUTALLY WRONG HERE" << std::endl;
    }
    if (joint_collision_names_[idy] != joint_name_2){
        std::cerr << "SOMETHING'S BRUTALLY WRONG HERE" << std::endl;
    }

    // receive the dependent transformation for both joints
    updateURDFParser(((*op_point_in_vec_[idx])(i)), idx);
    updateURDFParser(((*op_point_in_vec_[idy])(i)), idy);


    fcl::Vec3f closest_point_1, closest_point_2;
    urdfParser_->getClosestPoints(joint_collision_names_[idx],
                                  joint_collision_names_[idy],
                                  closest_point_1,
                                  closest_point_2);

    // IMPORTANT NOTE HERE:
    // The second point is getting ignored
    // due to the duplication of the signal matrix
    // Place here the update of the dirty flag
    // by a measurement if any of the given input signals has changed or not.

    const sot::MatrixHomogeneous& op_point_in = (*op_point_in_vec_[idx])(i);
    dynamicgraph::Vector cp1 = Conversions::convertToDG(closest_point_1);
    dynamicgraph::Vector relativ_point = op_point_in.inverse().multiply(cp1);

    int matrixIndex = getMatrixIndex(idx, idy);
    oppoint_transformations_[matrixIndex]->setConstant(relativ_point);


    sot::MatrixRotation rot1;
    op_point_in.extract(rot1);
    point.buildFrom(rot1,relativ_point);

    point.elementAt(0,3) = closest_point_1[0];
    point.elementAt(1,3) = closest_point_1[1];
    point.elementAt(2,3) = closest_point_1[2];
    return point;

}

void DynamicGraphFCL::updateURDFParser(const dynamicgraph::Matrix& op_point_sig, int id)const
{

    /* This case differencial has to be done based on JLR-Dynamic convention
      all rotations are going to be around X-Axis and not as mentioned in the URDF

      on the same hand, they ignore all FIXED-joint declarations, which means all frames like
                        end-effector, tool_joints, base_joints etc.
        are not being under this convention and can be processed as usual.
        */

    dynamicgraph::Matrix origin = Conversions::convertToDG(urdfParser_->getOrigin(joint_collision_names_[id]));
    dynamicgraph::Matrix urdf_frame = sotCompensator_->applySOTCompensation(op_point_sig, id);
    dynamicgraph::Matrix urdf_origin = urdf_frame.multiply(origin);

    // update collision objects. Rotation and Position is directly transformed into FCL
    // all transformations got capsuled into Conversion-namespace to keep up the separation between URDF/FCL and DG
    urdfParser_->updateLinkPosition(
                joint_collision_names_[id],
                *(Conversions::convertToFCLTransform(urdf_origin)));

}

} // namespace FCL
} // namespace dynamicgraph

