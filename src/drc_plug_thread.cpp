#include <yarp/os/all.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <iostream>

#include "drc_plug_thread.h"
#include "drc_plug_constants.h"

using namespace yarp::math;

using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman::drc::plug;

drc_plug_thread::drc_plug_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph ),
    command_interface( module_prefix ),
    status_interface( module_prefix ),
    q_left_desired(1),q_right_desired(1),
    plug_traj()
{
  //STATE MACHINE
    std::vector<std::tuple<state,std::string,state>> transition_table{
        //--------------initial state ----------+--------- command ---------------------------+------ final state--------- +
        std::make_tuple( state::idle            ,   WALKMAN_DRC_PLUG_COMMAND_BUTTON_DATA_SENT ,    state::ready           ),
        //--------------------------------------+---------------------------------------------+----------------------------+
	std::make_tuple( state::ready           ,   WALKMAN_DRC_PLUG_COMMAND_REACH            ,    state::reaching         ),
        //--------------------------------------+---------------------------------------------+----------------------------+
        std::make_tuple( state::reaching        ,   WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE      ,    state::reached          ),
        std::make_tuple( state::reached         ,   WALKMAN_DRC_PLUG_COMMAND_APPROACH         ,    state::approaching      ),
        std::make_tuple( state::reached         ,   WALKMAN_DRC_PLUG_COMMAND_REACH            ,    state::reaching         ),
        //--------------------------------------+---------------------------------------------+----------------------------+
        std::make_tuple( state::approaching     ,   WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE      ,    state::approached       ),
        std::make_tuple( state::approached      ,   WALKMAN_DRC_PLUG_COMMAND_REACH            ,    state::reaching         ),
	std::make_tuple( state::approached      ,   WALKMAN_DRC_PLUG_COMMAND_ROTATE           ,    state::rotating         ),
        //--------------------------------------+---------------------------------------------+----------------------------+
        std::make_tuple( state::rotating        ,   WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE      ,    state::rotated          ),
        std::make_tuple( state::rotated         ,   WALKMAN_DRC_PLUG_COMMAND_MOVE_AWAY        ,    state::moving_away      ),
        std::make_tuple( state::rotated         ,   WALKMAN_DRC_PLUG_COMMAND_ROTATE           ,    state::rotating         ),
        //--------------------------------------+---------------------------------------------+----------------------------+
	std::make_tuple( state::moving_away     ,   WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE      ,    state::moved_away       ),
	std::make_tuple( state::moved_away      ,   WALKMAN_DRC_PLUG_COMMAND_REACH            ,    state::reaching         ),
	std::make_tuple( state::moved_away      ,   WALKMAN_DRC_PLUG_COMMAND_SAFE_EXIT        ,    state::safe_exiting     ),
	
    };
    
    state_map[state::idle] = "idle";
    state_map[state::ready] = "ready";
    state_map[state::reaching] = "reaching";
    state_map[state::reached] = "reached";
    state_map[state::approaching] = "approaching";
    state_map[state::approached] = "approached";
    state_map[state::ungrasping] = "ungrasping";
    state_map[state::ungrasped] = "ungrasped";
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

    seq_num = 0;
    status_seq_num = 0;

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
    robot.sense(input.q, input.q_dot, input.tau);
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
    create_sot_problem(fixed_frame);

    if(robot.left_hand.isAvailable) { 
		robot.left_hand.setPositionDirectMode();
    }
    if(robot.right_hand.isAvailable) { 
		robot.right_hand.setPositionDirectMode();
    }
    
    plug_traj.init(auto_stack->left_arm_task,
                   auto_stack->right_arm_task,
                   auto_stack->right_foot_task,
                   auto_stack->com_task,
                   auto_stack->pelvis_task);
    
    robot.left_arm.setPositionDirectMode();
    robot.left_leg.setPositionDirectMode();
    robot.right_arm.setPositionDirectMode();
    robot.right_leg.setPositionDirectMode();
    robot.torso.setPositionDirectMode();
    
    if(robot.left_hand.isAvailable)
	robot.left_hand.setPositionDirectMode();
    if(robot.right_hand.isAvailable)
	robot.right_hand.setPositionDirectMode();
    
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
    }
    if ( plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_LEFT ) {
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Using LEFT hand to open the valve ..." << std::endl;
	plug_traj.set_controlled_arms(true,false);
    }
    if (plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_VALVE_DATA_SENT ) 
    {
	std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Valve data received ..." << std::endl;
	plug_traj.get_data(plug_cmd.command, plug_cmd.frame, plug_cmd.valve_data, model);
    }   
    if (plug_cmd.command == WALKMAN_DRC_PLUG_COMMAND_BUTTON_DATA_SENT ) 
    {
        std::cout << "Command ["<<seq_num<<"]: "<<plug_cmd.command<<", Valve point to reach  data received ..." << std::endl;
        plug_traj.get_data(plug_cmd.command, plug_cmd.frame, plug_cmd.button_data, model);
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
    yarp::sig::Vector q_torso(3), q_left_arm(7), q_right_arm(7), q_left_leg(6), q_right_leg(6), q_head(2);
    robot.fromIdynToRobot(output.q, q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head);
    robot.right_arm.move(q_right_arm);
    robot.left_arm.move(q_left_arm);
    robot.torso.move(q_torso);
    robot.left_leg.move(q_left_leg);
    robot.right_leg.move(q_right_leg);
}

bool drc_plug_thread::move_hands(double close)
{	
  if (close <= 1.0 && close >= 0.0)
  {
      if (plug_traj.left_arm_controlled) q_left_desired   = MIN_CLOSURE + close*(MAX_CLOSURE - MIN_CLOSURE); 
      if (plug_traj.right_arm_controlled) q_right_desired = MIN_CLOSURE + close*(MAX_CLOSURE - MIN_CLOSURE);
      robot.moveHands(q_left_desired,q_right_desired);
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
    if(robot.hasHands())
    {
	yarp::sig::Vector q_left(1);
	yarp::sig::Vector q_right(1);
	sense_hands(q_left, q_right);

	if ( fabs(q_left[0]-q_left_desired[0]) < 0.1 &&  fabs(q_right[0]-q_right_desired[0]) < 0.1 ) return true;	  
	    return false;
    }
    else return true;
}

void drc_plug_thread::create_sot_problem(std::string base_frame)
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