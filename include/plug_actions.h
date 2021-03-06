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

#ifndef PLUG_ACTIONS_H_
#define PLUG_ACTIONS_H_

#include "plug_interface.h"
#include <OpenSoT/OpenSoT.h>
#include <trajectory_generator/trajectory_generator.h>
#include <idynutils/cartesian_utils.h>
#include <mutex>

namespace walkman
{
    namespace drc
    {
        namespace plug
        {
	    class plug_actions {
	    private:
		
		/**
                 * @brief data of the door: X Y Z roll pitch yaw handle_length door_width
                 */
		
		// kdl frames for initial and final cartesian poses
                std::vector<double> valve_data;
                std::vector<double> button_data;
		
		KDL::Frame world_InitialRhand,world_InitialLhand,world_InitialRfoot,world_InitialCom;
		KDL::Frame world_InitialPelvis, world_FinalPelvis;
		KDL::Frame world_FinalRhand,world_FinalLhand,world_FinalRfoot,world_FinalCom;
		KDL::Frame world_Valve, world_Button;
                KDL::Frame Button_FinalLhand, Button_FinalRhand;
		KDL::Frame Valve_FinalRhand, Valve_FinalLhand;
                KDL::Frame world_HomePositionR;
                KDL::Frame world_HomePositionL;
		
		// local sot tasks
		OpenSoT::tasks::velocity::Cartesian::Ptr left_arm_task;
		OpenSoT::tasks::velocity::Cartesian::Ptr right_arm_task;
		OpenSoT::tasks::velocity::Cartesian::Ptr right_foot_task;
		OpenSoT::tasks::velocity::CoM::Ptr com_task;
		OpenSoT::tasks::velocity::Cartesian::Ptr pelvis_task;
		OpenSoT::tasks::velocity::Postural::Ptr postural_task;
		
		// trajectory generators for all tasks
		trajectory_generator left_arm_generator;
		trajectory_generator right_arm_generator;
		trajectory_generator right_foot_generator;
		trajectory_generator com_generator;
		trajectory_generator pelvis_generator;
		trajectory_generator joints_traj_gen;
		polynomial_coefficients poly;
		
                std::map<std::string, std::vector<double>*> cmd_data_map;

		double initialized_time;
		
		iDynUtils& model;
		double yaw_init;
                double radius_pin;
                double valve_radius;
		
		void compute_cartesian_error(KDL::Frame Start, KDL::Frame Target, KDL::Vector& position_error, KDL::Vector& orientation_error);

	    public:
		plug_actions(iDynUtils& model_);
		
		// initialization: receive sot task from the thread
		void init( OpenSoT::tasks::velocity::Cartesian::Ptr , 
			   OpenSoT::tasks::velocity::Cartesian::Ptr ,
			   OpenSoT::tasks::velocity::Cartesian::Ptr ,
		           OpenSoT::tasks::velocity::CoM::Ptr,
			   OpenSoT::tasks::velocity::Cartesian::Ptr ,
			   OpenSoT::tasks::velocity::Postural::Ptr
 			);
		
                bool get_data(std::string command, std::string Frame, KDL::Frame object_data_, double radius = -1);
		std::string ref_frame;
		                
		void set_controlled_arms(bool left_arm, bool right_arm);
		
		// declaration of cartesian actions
		bool init_reaching(mode current_mode);	
		bool init_approaching(mode current_mode);
		bool init_rotating(mode current_mode, double angle);
		bool init_moving_away(mode current_mode);
		bool init_moving_back();
		bool init_safe_exiting();
		
		bool perform_reaching();
		bool perform_approaching();
		bool perform_rotating(mode current_mode);
		bool perform_moving_away();
		bool perform_moving_back();
		bool perform_safe_exiting();
		
		void get_left_arm_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error);
		void get_right_arm_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error);
		void get_right_foot_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error);
		void get_com_cartesian_error(KDL::Vector& position_error);
		
                void get_controlled_arms(bool& using_left, bool& using_right);
		
		bool left_arm_controlled;
		bool right_arm_controlled;
        std::fstream fs;
		
	    };
	}
    }
}

#endif