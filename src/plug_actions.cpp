#include "plug_actions.h"
#include <kdl/chain.hpp>
#include <tf_conversions/tf_kdl.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <mutex>
#include "plug_interface.h"
#include "../../../build/install/include/kdl_codyco/floatingjntspaceinertiamatrix.hpp"

#define FT_THS 20 // force threshold [Nm] WORKING IN GAZEBO 
#define FT_FILTER_SIZE 50;

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2
#define ROLL_INDEX 3
#define PITCH_INDEX 4
#define YAW_INDEX 5
#define QUAT_INDEX 6
#define RADIUS_INDEX 7

#define DRILL_HEIGHT 0.275 		// top of the drill wrt the base of the drill
#define DRILL_GRASP_HEIGHT_FACTOR 0.75	// where to place the hand to grasp the tool (percentage of DRILL_HEIGHT)
#define DRILL_LENGHT 0.25 		// lenght of the drill wrt the base of the drill
#define DRILL_HANDLE_DEPTH 0.03		// handle edge wrt origin
#define APPROACHING_OFFSET 0.1		// safe distance for reaching, to be shortened during approaching
#define INSERT_OFFSET 0.03		// how much to insert the pin in the hole of the valve
#define DRILL_ROTATION_ANGLE 90	        // rotation of the drill (deg) to set the button of the drill in the right position

//drill button offsets
#define DRILL_BUTTON_X 0.02
#define DRILL_BUTTON_Y -0.02
#define DRILL_BUTTON_Z 0.075
#define PIN_HAND_X 0.07
#define PIN_HAND_Y 0.05
#define PIN_HAND_Z 0

walkman::drc::plug::plug_actions::plug_actions()
{
    cmd_data_map["valvedatasent"] = &valve_data;
    cmd_data_map["buttondatasent"] = &button_data;
    
    valve_data.resize(8);
    valve_data[X_INDEX] = 0.0;
    valve_data[Y_INDEX] = 0.0;
    valve_data[Z_INDEX] = 0.0;
    valve_data[ROLL_INDEX] = 0.0;
    valve_data[PITCH_INDEX] = 0.0;
    valve_data[YAW_INDEX] = 0.0;
    
    button_data.resize(8);
    button_data[X_INDEX] = 0.0;
    button_data[Y_INDEX] = 0.0;
    button_data[Z_INDEX] = 0.0;
    button_data[ROLL_INDEX] = 0.0;
    button_data[PITCH_INDEX] = 0.0;
    button_data[YAW_INDEX] = 0.0;
    
    left_arm_controlled = false;
    right_arm_controlled = false;
}

void walkman::drc::plug::plug_actions::set_controlled_arms(bool left_arm, bool right_arm)
{
    left_arm_controlled = left_arm;
    right_arm_controlled = right_arm;
}

void walkman::drc::plug::plug_actions::init(OpenSoT::tasks::velocity::Cartesian::Ptr l_arm_task, 
					      OpenSoT::tasks::velocity::Cartesian::Ptr r_arm_task,
					      OpenSoT::tasks::velocity::Cartesian::Ptr r_foot_task,
					      OpenSoT::tasks::velocity::CoM::Ptr com_task_,
					      OpenSoT::tasks::velocity::Cartesian::Ptr pelvis_task_)
{
    left_arm_task = l_arm_task;
    right_arm_task = r_arm_task;
    right_foot_task = r_foot_task;
    com_task = com_task_;
    pelvis_task = pelvis_task_; 
    
    // SET HOME POSITION
    YarptoKDL(left_arm_task->getActualPose(), world_HomePositionL);
    YarptoKDL(right_arm_task->getActualPose(), world_HomePositionR);
}

void walkman::drc::plug::plug_actions::get_controlled_arms(bool& using_left, bool& using_right)
{
    using_left = left_arm_controlled;
    using_right = right_arm_controlled;
}

void walkman::drc::plug::plug_actions::compute_cartesian_error(KDL::Frame Start, KDL::Frame Target, KDL::Vector& position_error, KDL::Vector& orientation_error)
{
    quaternion q,qd;
    
    Start.M.GetQuaternion(q.x, q.y, q.z, q.w);
    Target.M.GetQuaternion(qd.x, qd.y, qd.z, qd.w);

    //This is needed to move along the short path in the quaternion error
    if(quaternion::dot(q, qd) < 0.0) q = q.operator *(-1.0);

    KDL::Vector xerr_p; // Cartesian position error
    KDL::Vector xerr_o; // Cartesian orientation error

    position_error = Target.p - Start.p;
    orientation_error = quaternion::error(q, qd);  
}

void walkman::drc::plug::plug_actions::get_left_arm_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error)
{
    KDL::Frame world_CurrentLarm;
    YarptoKDL(left_arm_task->getActualPose(),world_CurrentLarm);
    compute_cartesian_error(world_CurrentLarm,world_FinalLhand,position_error,orientation_error);
}

void walkman::drc::plug::plug_actions::get_right_arm_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error)
{
    KDL::Frame world_CurrentRarm;
    YarptoKDL(right_arm_task->getActualPose(),world_CurrentRarm);
    compute_cartesian_error(world_CurrentRarm,world_FinalRhand,position_error,orientation_error);
}

void walkman::drc::plug::plug_actions::get_right_foot_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error)
{
    KDL::Frame world_CurrentRfoot;
    YarptoKDL(right_arm_task->getActualPose(),world_CurrentRfoot);
    compute_cartesian_error(world_CurrentRfoot,world_FinalRfoot,position_error,orientation_error);
}

void walkman::drc::plug::plug_actions::get_com_cartesian_error(KDL::Vector& position_error)
{
    KDL::Frame world_CurrentCom;
    YarptoKDL(com_task->getActualPosition(),world_CurrentCom.p);
    
    // NOTE this doesn't work for the com, change it
//     compute_cartesian_error(world_CurrentCom,world_FinalCom,position_error);
}

// TODO
bool walkman::drc::plug::plug_actions::get_data(std::string command, std::string Frame, KDL::Frame object_data_, iDynUtils& model_)
{
    KDL::Frame  World_data;
    ref_frame = Frame;
    if (ref_frame != "world")
    {
        KDL::Frame Frame_data, Anchor_World, Anchor_Frame;
        Frame_data = object_data_;
        Anchor_World = model_.getAnchor_T_World();
        Anchor_Frame = model_.iDyn3_model.getPositionKDL(model_.iDyn3_model.getLinkIndex(model_.getAnchor()),model_.iDyn3_model.getLinkIndex(ref_frame));
        World_data = Anchor_World.Inverse() * Anchor_Frame * Frame_data;
    }
    else if (ref_frame == "world")
        World_data = object_data_;

    cmd_data_map.at(command)->at(X_INDEX) = World_data.p.x();
    cmd_data_map.at(command)->at(Y_INDEX) = World_data.p.y();
    cmd_data_map.at(command)->at(Z_INDEX) = World_data.p.z();
    double ro,pi,ya;
    World_data.M.GetRPY(ro,pi,ya);
    cmd_data_map.at(command)->at(ROLL_INDEX) = ro;
    cmd_data_map.at(command)->at(PITCH_INDEX) = pi;
    cmd_data_map.at(command)->at(YAW_INDEX) = ya;    
        
    std::cout<<command<<" data Received:"<<std::endl;
    std::cout<<"| x: "<<cmd_data_map.at(command)->at(X_INDEX)<<std::endl;
    std::cout<<"| y: "<<cmd_data_map.at(command)->at(Y_INDEX)<<std::endl;
    std::cout<<"| z: "<<cmd_data_map.at(command)->at(Z_INDEX)<<std::endl;
    std::cout<<"| roll: "<<cmd_data_map.at(command)->at(ROLL_INDEX)<<std::endl;
    std::cout<<"| pitch: "<<cmd_data_map.at(command)->at(PITCH_INDEX)<<std::endl;
    std::cout<<"| yaw: "<<cmd_data_map.at(command)->at(YAW_INDEX)<<std::endl;
    
    world_Valve.p = KDL::Vector(valve_data[X_INDEX],valve_data[Y_INDEX],valve_data[Z_INDEX]);
    world_Valve.M = KDL::Rotation::RPY(valve_data[ROLL_INDEX], valve_data[PITCH_INDEX], valve_data[YAW_INDEX] );
        
    world_Button.p = KDL::Vector(button_data[X_INDEX],button_data[Y_INDEX],button_data[Z_INDEX]);
    world_Button.M = KDL::Rotation::RPY(button_data[ROLL_INDEX], button_data[PITCH_INDEX], button_data[YAW_INDEX] );
    return true;       
}

bool walkman::drc::plug::plug_actions::init_reaching()
{      
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);  
    YarptoKDL(right_arm_task->getActualPose(), world_InitialRhand);
//     YarptoKDL(com_task->getActualPosition(), world_InitialCom.p);
    YarptoKDL(pelvis_task->getActualPose(), world_InitialPelvis);
    
//     world_FinalCom = world_InitialCom;
//     world_FinalCom.p.data[2] -= 0.1;
//     com_generator.line_initialize(5.0, world_InitialCom,world_FinalCom); 
    
    world_FinalPelvis = world_InitialPelvis;
    
    if (world_Valve.p.data[2] < 1.1)
    {
	world_FinalPelvis.p.data[2] = world_Valve.p.data[2];
	pelvis_generator.line_initialize(5.0, world_InitialPelvis,world_FinalPelvis); 
    }
    else
    {
	world_FinalPelvis.p.data[2] = 1.05;
	pelvis_generator.line_initialize(5.0, world_InitialPelvis,world_FinalPelvis); 
    }
    
    if (left_arm_controlled){ 
	Button_FinalLhand.p = KDL::Vector(-(APPROACHING_OFFSET + PIN_HAND_X),-PIN_HAND_Y,0);
	Button_FinalLhand.M = KDL::Rotation::RotY(-M_PI/2.0);
	world_FinalLhand = world_Button * Button_FinalLhand;
	
	left_arm_generator.line_initialize(5.0, world_InitialLhand,world_FinalLhand);
    }
    if (right_arm_controlled){
	Button_FinalRhand.p = KDL::Vector(-(APPROACHING_OFFSET + PIN_HAND_X),PIN_HAND_Y,0);
	Button_FinalRhand.M = KDL::Rotation::RotY(-M_PI/2.0);
	world_FinalRhand = world_Button * Button_FinalRhand;
	
	right_arm_generator.line_initialize(5.0, world_InitialRhand,world_FinalRhand); 
    }
     
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::plug::plug_actions::perform_reaching()
{
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_L, Xd_R, Xd_c, Xd_p;
    KDL::Twist dXd_L, dXd_R, dXd_c, dXd_p;
 
    if (left_arm_controlled){ 
	left_arm_generator.line_trajectory(time,Xd_L,dXd_L);
        left_arm_task->setReference( KDLtoYarp_position( Xd_L ) );
    }
    if (right_arm_controlled){
	right_arm_generator.line_trajectory(time,Xd_R,dXd_R);
	right_arm_task->setReference( KDLtoYarp_position( Xd_R ) );    
    }
//     com_generator.line_trajectory(time,Xd_c,dXd_c);
//     yarp::sig::Vector p_CoM;
//     KDLtoYarp(Xd_c.p , p_CoM);
//     com_task->setReference(p_CoM);

    pelvis_generator.line_trajectory(time,Xd_p,dXd_p);
    pelvis_task->setReference( KDLtoYarp_position( Xd_p ) );
   
    return true;
}

bool walkman::drc::plug::plug_actions::init_approaching()
{
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);  
    YarptoKDL(right_arm_task->getActualPose(), world_InitialRhand);
    
    if (left_arm_controlled)
    { 
	Button_FinalLhand.p = KDL::Vector(-PIN_HAND_X,-PIN_HAND_Y,0);
	Button_FinalLhand.M = KDL::Rotation::RotY(-M_PI/2.0);
	world_FinalLhand = world_Button * Button_FinalLhand;
        left_arm_generator.line_initialize(5.0, world_InitialLhand,world_FinalLhand);
    }
    if (right_arm_controlled)
    {
	Button_FinalRhand.p = KDL::Vector(-PIN_HAND_X,PIN_HAND_Y,0);
	Button_FinalRhand.M = KDL::Rotation::RotY(-M_PI/2.0);
	world_FinalRhand = world_Button * Button_FinalRhand; 
        right_arm_generator.line_initialize(5.0, world_InitialRhand,world_FinalRhand); 
    }
     
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::plug::plug_actions::perform_approaching()
{
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_L, Xd_R;
    KDL::Twist dXd_L, dXd_R;

    if (left_arm_controlled){ 
	left_arm_generator.line_trajectory(time,Xd_L,dXd_L);
        left_arm_task->setReference( KDLtoYarp_position( Xd_L ) );
    }
    if (right_arm_controlled){
	right_arm_generator.line_trajectory(time,Xd_R,dXd_R);
	right_arm_task->setReference( KDLtoYarp_position( Xd_R ) );    
    }
   
    return true;
}

bool walkman::drc::plug::plug_actions::init_rotating(double angle)
{
    double rot_amount = angle;
    double T_f = angle/10;
    double radius_hand = sqrt( pow((world_Valve.p.x()-world_Button.p.x()),2) 
			+ pow((world_Valve.p.y()-(world_Button.p.y()+PIN_HAND_Y)),2) 
			+ pow((world_Valve.p.z()-world_Button.p.z()),2));
    double radius_pin = sqrt( pow((world_Valve.p.x()-world_Button.p.x()),2) 
			+ pow((world_Valve.p.y()-world_Button.p.y()),2) 
			+ pow((world_Valve.p.z()-world_Button.p.z()),2));
    std::cout<<"Radius from hand to valve: "<<radius_hand<<std::endl;
    std::cout<<"Radius from pin to valve: "<<radius_pin<<std::endl;
    
    KDL::Frame world_OffsetValve = world_Valve;
    world_OffsetValve.p.data[1] -= PIN_HAND_Y;
    
    KDL::Twist dXd_L, dXd_R;
    
    YarptoKDL(right_arm_task->getActualPose(), world_InitialRhand);
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);
    
    if (left_arm_controlled)
    {
	left_arm_generator.circle_initialize( T_f, radius_pin, -rot_amount * DEG2RAD, world_InitialLhand, world_OffsetValve);
	left_arm_generator.circle_trajectory(T_f + 1.0, world_FinalLhand, dXd_L);
    }

    if (right_arm_controlled)
    {
	right_arm_generator.circle_initialize(T_f, radius_pin, -rot_amount * DEG2RAD, world_InitialRhand, world_OffsetValve);
	right_arm_generator.circle_trajectory(T_f + 1.0, world_FinalRhand, dXd_R);
    }
    
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::plug::plug_actions::perform_rotating()
{
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_L, Xd_R;
    KDL::Twist dXd_L, dXd_R;

    if (left_arm_controlled) 
    {
	left_arm_generator.circle_trajectory(time, Xd_L, dXd_L);
	Xd_L.M = world_InitialLhand.M;
	left_arm_task->setReference( KDLtoYarp_position( Xd_L ) );
    }
    
    if (right_arm_controlled)
    {
	right_arm_generator.circle_trajectory(time, Xd_R, dXd_R);
	Xd_R.M = world_InitialRhand.M;
	right_arm_task->setReference( KDLtoYarp_position( Xd_R ) );
    }
    
    return true;
}

// TODO Change it to move perpendicularly from the initial position
bool walkman::drc::plug::plug_actions::init_moving_away()
{
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);  
    YarptoKDL(right_arm_task->getActualPose(), world_InitialRhand);
    
    if (left_arm_controlled)
    { 
	KDL::Frame HandRotated_Hand, world_HandRotated;
        world_HandRotated = world_InitialLhand;
	HandRotated_Hand.p = KDL::Vector(0,0,APPROACHING_OFFSET + INSERT_OFFSET);
	world_FinalLhand = world_HandRotated * HandRotated_Hand;
        left_arm_generator.line_initialize(5.0, world_InitialLhand,world_FinalLhand);
    }
    if (right_arm_controlled) // TODO SAME AS THE LEFT ARM
    {
	KDL::Frame HandRotated_Hand, world_HandRotated;
        world_HandRotated = world_InitialRhand;
	HandRotated_Hand.p = KDL::Vector(0,0,APPROACHING_OFFSET + INSERT_OFFSET);
	world_FinalRhand = world_HandRotated * HandRotated_Hand;
        right_arm_generator.line_initialize(5.0, world_InitialRhand,world_FinalRhand); 
    }
     
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::plug::plug_actions::perform_moving_away()
{
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_L, Xd_R;
    KDL::Twist dXd_L, dXd_R;

    if (left_arm_controlled){ 
	left_arm_generator.line_trajectory(time,Xd_L,dXd_L);
        left_arm_task->setReference( KDLtoYarp_position( Xd_L ) );
    }
    if (right_arm_controlled){
	right_arm_generator.line_trajectory(time,Xd_R,dXd_R);
	right_arm_task->setReference( KDLtoYarp_position( Xd_R ) );    
    }
   
    return true;
}

bool walkman::drc::plug::plug_actions::init_safe_exiting()
{
    YarptoKDL(right_arm_task->getActualPose(), world_InitialRhand);
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);

    world_FinalRhand = world_HomePositionR;
    world_FinalLhand = world_HomePositionL;
    
    if (left_arm_controlled)
	left_arm_generator.line_initialize(5.0, world_InitialLhand, world_FinalLhand);
    if (right_arm_controlled)
	right_arm_generator.line_initialize(5.0, world_InitialRhand, world_FinalRhand);

    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::plug::plug_actions::perform_safe_exiting()
{
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_L, Xd_R;
    KDL::Twist dXd_L, dXd_R;

    if (left_arm_controlled)
    {
	left_arm_generator.line_trajectory(time, Xd_L, dXd_L);
	left_arm_task->setReference( KDLtoYarp_position( Xd_L ) );
    }
    if (right_arm_controlled)
    {
	right_arm_generator.line_trajectory(time, Xd_R, dXd_R);
	right_arm_task->setReference( KDLtoYarp_position( Xd_R ) );
    }

    return true;
}