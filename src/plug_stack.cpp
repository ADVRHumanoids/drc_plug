/* Copyright [2014,2015] [Corrado Pavan, Alessandro Settimi, Luca Muratore]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#include "plug_stack.h"

walkman::drc::plug::plug_stack::plug_stack(const double dT,
                                              iDynUtils &model,
                                              const yarp::sig::Vector& q) :
    OpenSoT::AutoStack(q.size())  // initialize empty AutoStack
{
    using namespace OpenSoT;

    /* LEFT_ARM TASK */
    left_arm_task.reset(
        new tasks::velocity::Cartesian(
            "6d::l_arm::Waist", q, model, "LSoftHand", "world"));
    left_arm_task->setOrientationErrorGain(0.1);
    std::vector<bool> left_arm_active_joints = left_arm_task->getActiveJointsMask();
    for(unsigned int i = 0; i < left_arm_active_joints.size(); ++i){
        std::vector<unsigned int>::iterator it = std::find(
            model.left_arm.joint_numbers.begin(),
            model.left_arm.joint_numbers.end(),
            i);
        if(it != model.left_arm.joint_numbers.end())
            left_arm_active_joints[i] = true;
        else
            left_arm_active_joints[i] = false;
    }
//     for(unsigned int i = 0; i < 3; ++i)
//         left_arm_active_joints[model.torso.joint_numbers[i]] = true;
    left_arm_task->setActiveJointsMask(left_arm_active_joints);

    /* RIGHT ARM TASK */
    right_arm_task.reset(
        new tasks::velocity::Cartesian(
            "6d::r_arm::Waist", q,  model, "RSoftHand", "world"));
    right_arm_task->setOrientationErrorGain(0.1);

    std::vector<bool> rigth_arm_active_joints = right_arm_task->getActiveJointsMask();
    for(unsigned int i = 0; i < rigth_arm_active_joints.size(); ++i){
        std::vector<unsigned int>::iterator it = std::find(
            model.right_arm.joint_numbers.begin(),
            model.right_arm.joint_numbers.end(),
            i);
        if(it != model.right_arm.joint_numbers.end())
            rigth_arm_active_joints[i] = true;
        else
            rigth_arm_active_joints[i] = false;
    }
//     for(unsigned int i = 0; i < 3; ++i)
//         rigth_arm_active_joints[model.torso.joint_numbers[i]] = true;
    right_arm_task->setActiveJointsMask(rigth_arm_active_joints);
    
    /* RIGHT FOOT TASK */
    right_foot_task.reset(
        new tasks::velocity::Cartesian(
            "6d::r_foot::world", q,  model,"r_sole","world"));
    right_foot_task->setOrientationErrorGain(0.1);

    pelvis_task.reset(
        new tasks::velocity::Cartesian(
            "6d::r_arm::pelvis", q,  model,"Waist","world"));
    pelvis_task->setOrientationErrorGain(0.1);

    yarp::sig::Matrix pelvis_weight(6,6);
    pelvis_weight.eye();
    pelvis_weight(0,0) = 0.0;
    pelvis_weight(1,1) = 0.0;
//     pelvis_weight(2,2) = 0.0;
    pelvis_task->setWeight(pelvis_weight);

    /* COM Task */
    com_task.reset(
        new OpenSoT::tasks::velocity::CoM(q, model));
    yarp::sig::Matrix com_weight(3,3);
    com_weight.eye();
    com_weight(2,2) = 0.0;
    com_task->setWeight(com_weight);
    
    std::vector<bool> com_active_joints = com_task->getActiveJointsMask();
    for(unsigned int i = 0; i < com_active_joints.size(); ++i){
        std::vector<unsigned int>::iterator it = std::find(
            model.right_arm.joint_numbers.begin(),
            model.right_arm.joint_numbers.end(),
            i);
        if(it != model.right_arm.joint_numbers.end())
            com_active_joints[i] = false;
        else
            com_active_joints[i] = true;
    }
    for(unsigned int i = 0; i < com_active_joints.size(); ++i){
        std::vector<unsigned int>::iterator it = std::find(
            model.left_arm.joint_numbers.begin(),
            model.left_arm.joint_numbers.end(),
            i);
        if(it != model.left_arm.joint_numbers.end())
            com_active_joints[i] = false;
        else
            com_active_joints[i] = true;
    }
    com_task->setActiveJointsMask(com_active_joints);
    
    /* MANIPULABILITY TASKS */
    tasks::velocity::Manipulability::Ptr manip_left_arm_task(
        new tasks::velocity::Manipulability(q, model, left_arm_task));

    tasks::velocity::Manipulability::Ptr manip_right_arm_task(
        new tasks::velocity::Manipulability(q, model, right_arm_task));

    /* MINIMUM ACCELERATION TASK */
    tasks::velocity::MinimizeAcceleration::Ptr min_acc(
        new tasks::velocity::MinimizeAcceleration(q));

    /* POSTURAL TASK FOR TORSO */
    postural.reset(new tasks::velocity::Postural(q));
    /*
    yarp::sig::Vector zero;//(q.size(),0.0);
    zero.resize(q.size(),0.0);
    zero[model.left_arm.joint_numbers[1]] = 20 * M_PI/180.0;
    zero[model.right_arm.joint_numbers[1]] = -20 * M_PI/180.0;
    zero[model.left_arm.joint_numbers[3]] = -30 * M_PI/180.0;
    zero[model.right_arm.joint_numbers[3]] = -30 * M_PI/180.0;
    */
    postural->setReference(q);

    yarp::sig::Vector max = model.iDyn3_model.getJointBoundMax();
    yarp::sig::Vector min = model.iDyn3_model.getJointBoundMin();
    
    max[18]=-0.002;
    max[27]=-0.002;
    
    /* JOINT BOUNDS AND VELOCITY BOUNDS */
    constraints::velocity::JointLimits::Ptr joint_bounds(
        new constraints::velocity::JointLimits(
            q,
            max,
            model.iDyn3_model.getJointBoundMin()));
    
    double sot_speed_limit = 0.5;

    constraints::velocity::VelocityLimits::Ptr velocity_bounds(
        new constraints::velocity::VelocityLimits(
            sot_speed_limit,
            dT,
            q.size()));

    constraints::velocity::ConvexHull::Ptr convex_hull_constraint(
        new constraints::velocity::ConvexHull(
            q, model, 0.05));
    constraints::velocity::CoMVelocity::Ptr com_velocity_constraint(
        new constraints::velocity::CoMVelocity(
            yarp::sig::Vector(3, 0.3),
            dT, q, model));

    /* CREATING STACK AND BOUNDS */
    this->getStack() = ((right_foot_task << com_velocity_constraint) /
                        (com_task + pelvis_task<< com_velocity_constraint) /
                        ((left_arm_task + right_arm_task) << com_velocity_constraint) /
                        ((postural)))->getStack();

    this->getBoundsList().push_back(joint_bounds);
    this->getBoundsList().push_back(velocity_bounds);
    this->getBounds()->update(q);

    /* VELOCITY ALLOCATION */
    /*
    VelocityAllocation( this->getStack(),
                        dT, sot_speed_limit, sot_speed_limit);

    for(auto constraint : this->getStack()[3]->getConstraints())
    {
        using namespace OpenSoT::constraints::velocity;
        if(boost::dynamic_pointer_cast<VelocityLimits>(constraint))
        {
            boost::dynamic_pointer_cast<VelocityLimits>(constraint)->setVelocityLimits(
                sot_speed_limit+0.2);
            std::cout << "xxxxxx Setting joint space task velocity limits to " << sot_speed_limit+.2 << std::endl;
        }
    }
    */
}
