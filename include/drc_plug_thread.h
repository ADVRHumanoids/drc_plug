#ifndef drc_plug_THREAD_H_
#define drc_plug_THREAD_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <idynutils/yarp_single_chain_interface.h>
#include <idynutils/comanutils.h>

#include "plug_interface.h"
#include "plug_actions.h"
#include "plug_stack.h"

#include <GYM/control_thread.hpp>
#include <GYM/yarp_command_interface.hpp>
#include <GYM/yarp_status_interface.h>

#include <OpenSoT/OpenSoT.h>
#include <OpenSoT/Solver.h>

#include <drc_shared/yarp_msgs/plug_msg.h>
#include <drc_shared/state_machine.hpp>
#include "drc_shared/module_statuses/drc_plug_statuses.h"
#include <drc_shared/draw_state_machine.hpp>

/**
 * @brief drc_plug control thread
 * 
 **/
namespace walkman
{
    namespace drc
    {
        namespace plug
        {
	    class drc_plug_thread : public control_thread
	    {
	    private:  
	      
        std::fstream fs,fs1;
        iDynUtils real_robot;
        
		state_machine<state> stateMachine;
		
		plug_actions plug_traj;
		walkman::yarp_custom_command_interface<plug_msg> command_interface;
		int seq_num;
		
		walkman::yarp_status_interface status_interface;
		int status_seq_num;
		walkman::drc::plug::status_definitions status_definitions;
		
		plug_msg plug_cmd;
		
		void control_law();
		
		robot_state_input input;
		
		robot_state_output output;
		
		yarp::sig::Vector wb_input_q, wb_output_q;
		yarp::sig::Vector left_hand_input_q, right_hand_input_q;
		
		state current_state;
		
		void init_actions(state new_state);
		
		bool hands_in_position();
			  
                std::string fixed_frame;
                void create_sot_problem(std::string base_frame);
		
		yarp::sig::Vector q_left_desired,q_right_desired;
			  
		std::map<state,std::string> state_map;
		
		OpenSoT::solvers::QPOases_sot::Ptr solver;
                
                plug_stack::Ptr auto_stack;
		double postural_yaw;
		
	    public:
		
		/**
		* @brief constructor
		* 
		* @param module_prefix the prefix of the module
		* @param rf resource finderce
		* @param ph param helper
		*/
		drc_plug_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );
		
		
		/**
		* @brief drc_plug control thread initialization
		* 
		* @return true on succes, false otherwise
		*/
		virtual bool custom_init();
		
		/**
		* @brief drc_plug control thread main loop
		* 
		*/
		virtual void run();
		
		virtual void custom_release();

		/**
		  * @brief sense function
		  */
		void sense();

		/**
		  * @brief move function
		  */
		void move();
		
		bool move_hands(double close);
		
		bool sense_hands(yarp::sig::Vector &q_left_hand, yarp::sig::Vector &q_right_hand);
	    };
	}
    }
}

#endif
