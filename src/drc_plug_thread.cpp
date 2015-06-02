#include <yarp/os/all.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <iostream>
#include <iCub/iDynTree/yarp_kdl.h>
#include "drc_plug_thread.h"
#include "drc_plug_constants.h"

using namespace yarp::math;

using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman::drc::plug;

double left_offset[7] = {-0.001878101, -0.087266425, -0.00541460025, -0.04116454775, -0.0270602895, 0.05685963075, 0.05985226625};
double right_offset[7] = {0.00496249625, 0.01221735225, 0.023223271, -0.01633125525, -0.04591635675, 0.0131223505, -0.0860596935};
yarp::sig::Vector left_arm_offset(7,left_offset);
yarp::sig::Vector right_arm_offset(7,right_offset);

drc_plug_thread::drc_plug_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph ),
    command_interface( module_prefix ),
    status_interface( module_prefix ),
    plug_traj(model),real_robot(get_robot_name(),get_urdf_path(),get_srdf_path())
{
  //STATE MACHINE
    std::vector<std::tuple<state,std::string,state>> transition_table{
        //--------------initial state ----------+--------- command ---------------------------+------ final state--------- +
        std::make_tuple( state::idle            ,   WALKMAN_DRC_PLUG_COMMAND_BUTTON_DATA_SENT ,    state::ready            ),
        //--------------------------------------+---------------------------------------------+----------------------------+
	std::make_tuple( state::ready           ,   WALKMAN_DRC_PLUG_COMMAND_REACH            ,    state::reaching         ),
        //--------------------------------------+---------------------------------------------+----------------------------+
        std::make_tuple( state::reaching        ,   WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE      ,    state::reached          ),
        std::make_tuple( state::reached         ,   WALKMAN_DRC_PLUG_COMMAND_APPROACH         ,    state::approaching      ),
        std::make_tuple( state::reached         ,   WALKMAN_DRC_PLUG_COMMAND_REACH            ,    state::reaching         ),
        std::make_tuple( state::reached         ,   WALKMAN_DRC_PLUG_COMMAND_SAFE_EXIT        ,    state::safe_exiting     ),
        //--------------------------------------+---------------------------------------------+----------------------------+
        std::make_tuple( state::approaching     ,   WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE      ,    state::approached       ),
        std::make_tuple( state::approached      ,   WALKMAN_DRC_PLUG_COMMAND_REACH            ,    state::reaching         ),
	std::make_tuple( state::approached      ,   WALKMAN_DRC_PLUG_COMMAND_ROTATE           ,    state::rotating         ),
        std::make_tuple( state::approached      ,   WALKMAN_DRC_PLUG_COMMAND_MOVE_AWAY        ,    state::moving_away      ),
        //--------------------------------------+---------------------------------------------+----------------------------+
        std::make_tuple( state::rotating        ,   WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE      ,    state::rotated          ),
        std::make_tuple( state::rotated         ,   WALKMAN_DRC_PLUG_COMMAND_MOVE_AWAY        ,    state::moving_away      ),
        std::make_tuple( state::rotated         ,   WALKMAN_DRC_PLUG_COMMAND_ROTATE           ,    state::rotating         ),
        //--------------------------------------+---------------------------------------------+----------------------------+
	std::make_tuple( state::moving_away     ,   WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE      ,    state::moved_away       ),
	std::make_tuple( state::moved_away      ,   WALKMAN_DRC_PLUG_COMMAND_REACH            ,    state::reaching         ),
	std::make_tuple( state::moved_away      ,   WALKMAN_DRC_PLUG_COMMAND_SAFE_EXIT        ,    state::safe_exiting     ),
        //--------------------------------------+---------------------------------------------+----------------------------+
        std::make_tuple( state::safe_exiting    ,   WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE      ,    state::safe_exited      ),
        //--------------------------------------+---------------------------------------------+----------------------------+
        std::make_tuple( state::safe_exited     ,   WALKMAN_DRC_PLUG_COMMAND_REACH            ,    state::reaching         ),
    };
    
    state_map[state::idle] = "idle";
    state_map[state::ready] = "ready";
    state_map[state::reaching] = "reaching";
    state_map[state::reached] = "reached";
    state_map[state::approaching] = "approaching";
    state_map[state::approached] = "approached";
    state_map[state::rotating] = "rotating";
    state_map[state::rotated] = "rotated";
    state_map[state::moving_away] = "moving_away";
    state_map[state::moved_away] = "moved_away";
    state_map[state::safe_exiting] = "safe_exiting";
    state_map[state::safe_exited] = "safe_exited";

    stateMachine.insert(transition_table);
    
    walkman::drc::draw_state_machine<state,std::string> drawer;
    drawer.insert(transition_table);
    drawer.draw_on_file("state_machine.gml",state_map);

    current_state = state::idle;
    q_hands_desired.resize(2);

    seq_num = 0;
    status_seq_num = 0;

    fs.open ("plug_debug_trj.m", std::fstream::out);
    fs1.open ("plug_debug_jnt.m", std::fstream::out);
}

bool drc_plug_thread::custom_init()
{
   //  real time thread
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam ( pthread_self(), SCHED_FIFO, &thread_param );
    
    // start the status chain_interface
    status_interface.start();
    // notify the ready status
    status_interface.setStatus( status_definitions.status_to_code.at(WALKMAN_DRC_PLUG_STATUS_READY) );	
    
    // sense
    //-- using new walkmaninterface --//

//     wb_input_q=robot.sensePositionRefFeedback();
    wb_input_q = robot.sensePosition();
//     std::cout<<"INITIAL POSE"<<wb_input_q.toString()<<std::endl;
    input.q.resize(robot.getNumberOfKinematicJoints());
    output.q.resize(robot.getNumberOfKinematicJoints());
    wb_output_q.resize(robot.getNumberOfActuatedJoints());
    for(int i=0;i<input.q.size();i++) input.q[i]=wb_input_q[i]; //hands not considered

    left_hand_input_q = wb_input_q[robot.left_hand_index];
    right_hand_input_q = wb_input_q[robot.right_hand_index];
    
    //-- using new walkmaninterface --//
    
    auto max = model.iDyn3_model.getJointBoundMax();
    auto min = model.iDyn3_model.getJointBoundMin();
    for (int i=0;i<input.q.size();i++)
    {
        if (input.q[i]>max[i])
        {
            std::cout<<"error: "<<model.getJointNames().at(i)<<"("<<input.q[i]<<") is outside maximum bound: "<<max[i]<<std::endl;
        }
        if (input.q[i]<min[i])
        {
            std::cout<<"error: "<<model.getJointNames().at(i)<<"("<<input.q[i]<<") is outside minimum bound: "<<min[i]<<std::endl;
        }
    }
    // initializing output.q to current position
    output.q = input.q;

    // update model and com_model
    // NOTE why is working with the floating base set in the left foot?
    model.iDyn3_model.setFloatingBaseLink(model.left_leg.end_effector_index);
    model.updateiDyn3Model(input.q, true);

    // SOT INITIALIZATION
    // tasks initialization
    fixed_frame = "l_sole";
    bool created=create_sot_problem(fixed_frame);
    if (!created) return false;
    plug_traj.init(auto_stack->left_arm_task,
                   auto_stack->right_arm_task,
                   auto_stack->right_foot_task,
                   auto_stack->com_task,
                   auto_stack->pelvis_task,
                   auto_stack->postural);
    
    robot.setPositionDirectMode();

    return true;
}

void drc_plug_thread::init_actions(state new_state)
{
    if ( new_state == state::ready)
    {
    }
    if ( new_state == state::reaching)
    {	
	plug_traj.init_reaching();
    }
    if ( new_state == state::approaching)
    {
	plug_traj.init_approaching();
    }
    if ( new_state == state::rotating)
    {
	yarp::sig::Matrix r_w = auto_stack->right_arm_task->getWeight(); 
	r_w(3,3) = 0.0; 
	auto_stack->right_arm_task->setWeight(r_w);
	
	yarp::sig::Matrix l_w = auto_stack->left_arm_task->getWeight();
	l_w(3,3) = 0.0; 
	auto_stack->left_arm_task->setWeight(l_w);
	
	plug_traj.init_rotating(plug_cmd.angle);
    }
    if ( new_state == state::moving_away)
    {
      	yarp::sig::Matrix r_w = auto_stack->right_arm_task->getWeight(); 
	r_w(3,3) = 1; 
	auto_stack->right_arm_task->setWeight(r_w);
	
	yarp::sig::Matrix l_w = auto_stack->left_arm_task->getWeight();
	l_w(3,3) = 1; 
	auto_stack->left_arm_task->setWeight(l_w);
	plug_traj.init_moving_away();
    }

    if ( new_state == state::safe_exiting )
    {
	plug_traj.init_safe_exiting();
    }
}

void drc_plug_thread::run()
{   
    // get the command
    plug_cmd.command = WALKMAN_DRC_PLUG_COMMAND_NONE;
    command_interface.getCommand(plug_cmd,seq_num);
    
    // evolve the state machine accordingly to the received command
    state new_state=stateMachine.evolve_state_machine(current_state,plug_cmd.command);
    if (current_state!=new_state)
    {
	init_actions(new_state);
	current_state = new_state;
    }

    if ( plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_REACH ) {
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Reaching the valve ..." << std::endl;	
	if(!move_hands(1)) std::cout<<"Hands not available "<<std::endl;
    }
    if ( plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_APPROACH ) {
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Approaching the valve ..." << std::endl;
    }
    if ( plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_ROTATE ) {
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Rotating the valve ..." << std::endl;
    }
    if ( plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_RIGHT ) {
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Using RIGHT hand to open the valve ..." << std::endl;
	plug_traj.set_controlled_arms(false,true);
// 	std::vector<bool> left_arm_active_joints = auto_stack->left_arm_task->getActiveJointsMask();
// 	for(unsigned int i = 0; i < left_arm_active_joints.size(); ++i)
// 	    left_arm_active_joints[i] = false;
// 	left_arm_active_joints = auto_stack->right_arm_task->getActiveJointsMask();
// 	for(unsigned int i = 0; i < left_arm_active_joints.size(); ++i)
// 	    left_arm_active_joints[i] = true;
// 	auto_stack->right_arm_task->setActiveJointsMask(left_arm_active_joints);
    }
    if ( plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_LEFT ) {
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Using LEFT hand to open the valve ..." << std::endl;
	plug_traj.set_controlled_arms(true,false);
// 	std::vector<bool> rigth_arm_active_joints = auto_stack->right_arm_task->getActiveJointsMask();
// 	for(unsigned int i = 0; i < rigth_arm_active_joints.size(); ++i)
// 	    rigth_arm_active_joints[i] = false;
// 	auto_stack->right_arm_task->setActiveJointsMask(rigth_arm_active_joints);
// 	rigth_arm_active_joints = auto_stack->left_arm_task->getActiveJointsMask();
// 	for(unsigned int i = 0; i < rigth_arm_active_joints.size(); ++i)
// 	    rigth_arm_active_joints[i] = true;
// 	auto_stack->left_arm_task->setActiveJointsMask(rigth_arm_active_joints);
    }
    if (plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_VALVE_DATA_SENT ) 
    {
	std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Valve data received ..." << std::endl;
	plug_traj.get_data(plug_cmd.command, plug_cmd.frame, plug_cmd.valve_data);
    }   
    if (plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_BUTTON_DATA_SENT ) 
    {
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Valve point to reach  data received ..." << std::endl;
        plug_traj.get_data(plug_cmd.command, plug_cmd.frame, plug_cmd.button_data);
    }
    if ( plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_SAFE_EXIT ) {
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Exiting Safely ..." << std::endl;
    }
    if (plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_OPENING_HANDS) {
        if(!move_hands(0)) std::cout<<"Hands not available "<<std::endl;
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", opening the hands." << std::endl;
    }
    if (plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_CLOSING_HANDS) {
        if(!move_hands(1)) std::cout<<"Hands not available "<<std::endl;
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", closing the hands." << std::endl;
    }
    
    sense();

    control_law();

    move();
    
    KDL::Vector left_hand_position_error, left_hand_orientation_error, right_hand_position_error, right_hand_orientation_error;
    bool right_done=false;
    bool left_done=false;
    
    plug_traj.get_left_arm_cartesian_error(left_hand_position_error, left_hand_orientation_error);
    plug_traj.get_right_arm_cartesian_error(right_hand_position_error, right_hand_orientation_error);

    if( right_hand_position_error.Norm()<0.01 && right_hand_orientation_error.Norm()<0.1) right_done = true;
    if( left_hand_position_error.Norm()<0.01 && left_hand_orientation_error.Norm()<0.1) left_done = true;
    
    bool using_left,using_right;
    plug_traj.get_controlled_arms(using_left,using_right);
    
    if (current_state == state::rotating)
    {
      right_done = false;
      left_done = false;
    }
    
    if( (using_right && right_done) || (using_left && left_done) )
    {
	current_state=stateMachine.evolve_state_machine(current_state,WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE);
    }
    
    if(hands_in_position() &&
            (current_state == walkman::drc::plug::state::grasping || current_state == walkman::drc::plug::state::ungrasping))
		current_state=stateMachine.evolve_state_machine(current_state,WALKMAN_DRC_PLUG_COMMAND_HAND_DONE);
    
    if(status_definitions.status_to_code.count(state_map[current_state]))
    status_interface.setStatus(status_definitions.status_to_code.at(state_map[current_state]) , status_seq_num++);
}    

void drc_plug_thread::sense()
{
    input.q = output.q;
//     yarp::sig::Vector q_torso(3), q_left_arm(7), q_right_arm(7), q_left_leg(6), q_right_leg(6), q_head(2);
//     robot.fromIdynToRobot31(input.q, q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head);
    //OFFSET 
//     q_left_arm = q_left_arm - left_arm_offset;
//     q_right_arm = q_right_arm - right_arm_offset;
    
//     robot.fromRobotToIdyn31(q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head, input.q);
    
    //FAKE ROBOT    
    //-- using new walkmaninterface --//

    yarp::sig::Vector real_joints = robot.sensePosition();
    yarp::sig::Vector real_joints_no_hands(real_joints.size()-2,(double*)real_joints.getGslVector());
    
//     for(int i=0;i<real_joints_no_hands.size();i++) real_joints_no_hands[i]=real_joints[i];
    
    //-- using new walkmaninterface --//
    
    real_robot.iDyn3_model.setAng(real_joints_no_hands);
    real_robot.iDyn3_model.computePositions();
    
    KDL::Frame LeftFoot_LeftHand = real_robot.iDyn3_model.getPositionKDL(real_robot.left_leg.end_effector_index,real_robot.left_arm.end_effector_index);
    
    static int i=1;
    if(current_state == walkman::drc::plug::state::rotating) fs<<"LF_LH_real("<<i<<",:)=["<<LeftFoot_LeftHand.p.x()<<' '<<LeftFoot_LeftHand.p.y()<<' '<<LeftFoot_LeftHand.p.z()<<"];\n";
    
//     YarptoKDL(auto_stack->left_arm_task->getActualPose(),LeftFoot_LeftHand);
    LeftFoot_LeftHand = model.iDyn3_model.getPositionKDL(model.left_leg.end_effector_index,model.left_arm.end_effector_index);
    if(current_state == walkman::drc::plug::state::rotating) fs<<"LF_LH_SoT("<<i++<<",:)=["<<LeftFoot_LeftHand.p.x()<<' '<<LeftFoot_LeftHand.p.y()<<' '<<LeftFoot_LeftHand.p.z()<<"];\n";
}

void drc_plug_thread::control_law()
{
    model.updateiDyn3Model( input.q, true );
    auto_stack->update(input.q);
    
    bool success=false;
    
    if ( current_state == state::reaching )
    {
	if(!plug_traj.perform_reaching()){ std::cout<<"ERROR REACHING DRILL"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::approaching )
    {
	if(!plug_traj.perform_approaching()){ std::cout<<"ERROR REACHING DRILL"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::rotating )
    {
	if(!plug_traj.perform_rotating()){ std::cout<<"ERROR ROTATING DRILL: not possible with operating hand"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::moving_away )
    {
	if(!plug_traj.perform_moving_away()){ std::cout<<"ERROR MOVING AWAY"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::safe_exiting )
    {
	if(!plug_traj.perform_safe_exiting()){ std::cout<<"ERROR SAFE EXITING"<<std::endl; success=false;}
	else success=true;
    }
    
    if(solver->solve(output.q_dot))
	output.q = input.q + output.q_dot;
    else {
	output.q_dot = 0.0;
	output.q = input.q + output.q_dot;
    }
}

void drc_plug_thread::move()
{
    yarp::sig::Vector q_torso(3), q_left_arm(7), q_left_arm_real(7), q_right_arm(7), q_left_leg(6), q_right_leg(6), q_head(2);
//     robot.fromIdynToRobot31(output.q, q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head);
//     
//     //OFFSET 
//     q_left_arm = q_left_arm + left_arm_offset;
//     q_right_arm = q_right_arm + right_arm_offset;
// 
//     robot.fromRobotToIdyn31(q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head, wb_output_q);    
//     robot.move29(output.q);

//     yarp::sig::Vector real_joints = robot.sensePosition();
//     robot.fromRobotToIdyn31(real_joints, q_right_arm, q_left_arm_real, q_torso, q_right_leg, q_left_leg, q_head);
    
//     static int i=1;
//     if(current_state == walkman::drc::plug::state::rotating)
//         fs1<<"Jnt_SoT("<<i<<",:)=["<<q_left_arm[0]<<' '<<q_left_arm[1]<<' '<<q_left_arm[2]<<' '<<q_left_arm[3]<<' '<<q_left_arm[4]<<' '<<q_left_arm[5]<<' '<<q_left_arm[6]<<"];\n";
//     q_left_arm = q_left_arm_real;
//     if(current_state == walkman::drc::plug::state::rotating) 
//         fs1<<"Jnt_real("<<i++<<",:)=["<<q_left_arm[0]<<' '<<q_left_arm[1]<<' '<<q_left_arm[2]<<' '<<q_left_arm[3]<<' '<<q_left_arm[4]<<' '<<q_left_arm[5]<<' '<<q_left_arm[6]<<"];\n";
//     std::cout<<"OUT IDYN "<<output.q.toString()<<std::endl;
        robot.fromIdynToRobot31(output.q, q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head);
    
    //OFFSET 
//     q_left_arm = q_left_arm + left_arm_offset;
//     q_right_arm = q_right_arm + right_arm_offset;
    
    yarp::sig::Vector q_move(robot.getNumberOfActuatedJoints());
//     q_move.resize(31);
    robot.fromRobotToIdyn29(q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_move);
//     std::cout<<"OUT IDYN "<<output.q.toString()<<std::endl;

    robot.move29(q_move);
}

bool drc_plug_thread::move_hands(double close)
{	
  if (close <= 1.0 && close >= 0.0)
  {
      if (plug_traj.left_arm_controlled) q_hands_desired[1]   = MIN_CLOSURE + close*(MAX_CLOSURE - MIN_CLOSURE); 
      if (plug_traj.right_arm_controlled) q_hands_desired[0] = MIN_CLOSURE + close*(MAX_CLOSURE - MIN_CLOSURE);
      robot.moveHands(q_hands_desired);
      return true;
  }
  else
  {
      std::cout<<"closing amount is out of feasible bounds"<<std::endl;
  }
   return false;
}

bool drc_plug_thread::sense_hands(Vector& q_left_hand, Vector& q_right_hand)
{
    robot.senseHandsPosition(q_left_hand,q_right_hand);
    return true;
}

void drc_plug_thread::custom_release()
{
    generic_thread::custom_release();
}



bool drc_plug_thread::hands_in_position()
{
    yarp::sig::Vector q_left(1);
    yarp::sig::Vector q_right(1);
    sense_hands(q_left, q_right);

    if ( fabs(q_left[0]-q_hands_desired[1]) < 0.1 &&  fabs(q_right[0]-q_hands_desired[0]) < 0.1 ) return true;	  
	return false;
}

bool drc_plug_thread::create_sot_problem(string base_frame)
{
  try
  {
    auto_stack.reset(
        new plug_stack(
            static_cast<double>(get_thread_period())/1000.0,
            model,
            input.q));

    solver.reset(
        new OpenSoT::solvers::QPOases_sot(
            auto_stack->getStack(),
            auto_stack->getBounds()) );
  }
  catch (const char* ex) //Catching const chars in a control module: sadness
  {
    std::cout<<"Could not create a SoT stack or reset it or whatever "<<ex<<std::endl;
    return false;
  }
  return true;
}