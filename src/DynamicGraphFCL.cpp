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
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/factory.h>

#include <dynamic_graph_fcl/DynamicGraphFCL.h>
#include <dynamic_graph_fcl/Conversions.h>
#include <dynamic_graph_fcl/SignalHelper.h>

#include <ros/ros.h>

namespace dynamicgraph {
namespace FCL {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicGraphFCL, "DynamicGraphFCL");

typedef SignalTimeDependent < dynamicgraph::Matrix, int > SignalTimeMatrix;
typedef SignalTimeDependent < dynamicgraph::Vector, int > SignalTimeVector;
typedef SignalPtr<dynamicgraph::Vector, int > SignalPtrVector;
typedef SignalPtr<dynamicgraph::Matrix, int > SignalPtrMatrix;

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

    for (int joint_idx = 0; joint_idx < joint_collision_size_; ++joint_idx) {

        // for each collision object create one input signal to update the transform of the capsule/mesh
        op_point_in_vec_[joint_idx] =
                SignalHelper::createInputSignalMatrix(joint_collision_names_[joint_idx]);
        signalRegistration(*op_point_in_vec_[joint_idx]);

        // load the respective sot_compensation from the ros_param server
        // HERe!
        sotCompensator_->setSOTCompensation(joint_collision_names_[joint_idx], joint_idx);

        std::cerr << "input signal registrated " << joint_collision_names_[joint_idx] << std::endl;
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
                std::cerr << "matrix filled " << std::endl;
            }
        }
    }
}

void DynamicGraphFCL::fillCollisionMatrix(int idx, int idy)
{

    std::string joint_name_1 = joint_collision_names_[idx];
    std::string joint_name_2 = joint_collision_names_[idy];

    int collision_matrix_idx = idx*joint_collision_size_ + idy;

    boost::shared_ptr<SignalTimeVector> collisionSignal =
            SignalHelper::createOutputSignalTimeVector(joint_name_1+joint_name_2);

    collisionSignal->addDependency(*op_point_in_vec_[idx]);
    collisionSignal->addDependency(*op_point_in_vec_[idy]);
    collisionSignal->setFunction(boost::bind(&DynamicGraphFCL::closest_point_update_function,
                                             this, _1, _2, joint_name_1, idx, joint_name_2, idy));

    signalRegistration(*collisionSignal);
    collision_matrix_[collision_matrix_idx] = collisionSignal;

}

void DynamicGraphFCL::initDebugSignals()
{
    std::string debug_point_1 = "DynamicGraphFCL("+inName_+")::output(int)::debugPoint1";
    debug_point_1_out = boost::shared_ptr<SignalTimeVector>(new SignalTimeVector(NULL, debug_point_1));
    signalRegistration(*debug_point_1_out);

    std::string debug_point_2 = "DynamicGraphFCL("+inName_+")::output(int)::debugPoint2";
    debug_point_2_out = boost::shared_ptr<SignalTimeVector>(new SignalTimeVector(NULL, debug_point_2));
    signalRegistration(*debug_point_2_out);

}

/// Update function for the ouput signals
dynamicgraph::Vector& DynamicGraphFCL::closest_point_update_function(
                    Vector &point, int i,
                    std::string &joint_name_1, int &idx,
                    std::string &joint_name_2, int &idy){


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

    std::cerr << "closest_point_1" << closest_point_1 << std::endl;

    // IMPORTANT NOTE HERE:
    // The second point is getting ignored due to the duplication of the signal matrix
    // Place here the update of the dirty flag by a measurement if any of the given input signals has changed or not.

    point = Conversions::convertToDG(closest_point_1);
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

    tfBroadcaster_->sendTransform(
                "sot_"+joint_collision_names_[id],
                Conversions::transformToTF(op_point_sig));

    dynamicgraph::Matrix origin = Conversions::convertToDG(urdfParser_->getOrigin(joint_collision_names_[id]));
    dynamicgraph::Matrix urdf_frame = sotCompensator_->applySOTCompensation(op_point_sig, id);
    dynamicgraph::Matrix urdf_origin = urdf_frame.multiply(origin);

    tfBroadcaster_->sendTransform(
                "urdf"+joint_collision_names_[id],
                Conversions::transformToTF(urdf_frame));

    tfBroadcaster_->sendTransform(
                "_urdf_origin"+joint_collision_names_[id],
                Conversions::transformToTF(urdf_origin));


    dynamicgraph::Matrix op_point(4,4);
    op_point.setIdentity();
    if (urdfParser_->isEndeffector(joint_collision_names_[id])){
//        std::cerr << "no sot_compensation for joint: " << joint_collision_names_[id] << std::endl;
        op_point = op_point_sig;
    }
    else{
//        std::cerr << "!! sot_compensation for joint: " << joint_collision_names_[id] << std::endl;
//        op_point = Conversions::sot_rotation_fix(op_point_sig);
//        op_point = op_point_sig.multiply(Conversions::convertToDG(urdfParser_->getOrigin(joint_collision_names_[id])));
//        op_point = sotCompensator_->applySOTCompensation(op_point_sig, id);
        op_point = sotCompensator_->applySOTCompensation(origin, id);
    }


    // update collision objects. Rotation and Position is directly transformed into FCL
    // all transformations got capsuled into Conversion-namespace to keep up the separation between URDF/FCL and DG
    urdfParser_->updateLinkPosition(
                joint_collision_names_[id],
                *(Conversions::convertToFCLTransform(op_point)));

}

} // namespace FCL
} // namespace dynamicgraph

