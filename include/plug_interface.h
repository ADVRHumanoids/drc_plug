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

#ifndef PLUG_INTERFACE_H
#define PLUG_INTERFACE_H

#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#define WALKMAN_DRC_PLUG_COMMAND_NONE ""
#define WALKMAN_DRC_PLUG_COMMAND_IDLE "idle"
#define WALKMAN_DRC_PLUG_COMMAND_REACH "reach"
#define WALKMAN_DRC_PLUG_COMMAND_APPROACH "approach"
#define WALKMAN_DRC_PLUG_COMMAND_ROTATE "rotate"
#define WALKMAN_DRC_PLUG_COMMAND_GRASP "grasp"
#define WALKMAN_DRC_PLUG_COMMAND_UNGRASP "ungrasp"
#define WALKMAN_DRC_PLUG_COMMAND_MOVE_AWAY "move_away"
#define WALKMAN_DRC_PLUG_COMMAND_MOVE_BACK "move_back"
#define WALKMAN_DRC_PLUG_COMMAND_AUTO "auto"

#define WALKMAN_DRC_PLUG_COMMAND_VALVE_DATA_SENT "valvedatasent"
#define WALKMAN_DRC_PLUG_COMMAND_BUTTON_DATA_SENT "buttondatasent"
#define WALKMAN_DRC_PLUG_COMMAND_ACTION_DONE "action_done"
#define WALKMAN_DRC_PLUG_COMMAND_HAND_DONE "hand_done"

#define WALKMAN_DRC_PLUG_COMMAND_LEFT "left_hand"
#define WALKMAN_DRC_PLUG_COMMAND_RIGHT "right_hand"
#define WALKMAN_DRC_PLUG_COMMAND_STICK "stick"
#define WALKMAN_DRC_PLUG_COMMAND_HAND "hand"
#define WALKMAN_DRC_PLUG_COMMAND_CLOSING_HANDS "close_hands"
#define WALKMAN_DRC_PLUG_COMMAND_OPENING_HANDS "open_hands"
#define WALKMAN_DRC_PLUG_COMMAND_SAFE_EXIT "safe_exit"

#define WALKMAN_DRC_PLUG_STATUS_IDLE "idle"
#define WALKMAN_DRC_PLUG_STATUS_READY "ready"
#define WALKMAN_DRC_PLUG_STATUS_REACHING "reaching"
#define WALKMAN_DRC_PLUG_STATUS_REACHED "reached"
#define WALKMAN_DRC_PLUG_STATUS_APPROACHING "approaching"
#define WALKMAN_DRC_PLUG_STATUS_APPROACHED "approached"
#define WALKMAN_DRC_PLUG_STATUS_ROTATING "rotating"
#define WALKMAN_DRC_PLUG_STATUS_ROTATED "rotated"
#define WALKMAN_DRC_PLUG_STATUS_MOVING_AWAY "moving_away"
#define WALKMAN_DRC_PLUG_STATUS_MOVED_AWAY "moved_away"
#define WALKMAN_DRC_PLUG_STATUS_MOVING_BACK "moving_back"
#define WALKMAN_DRC_PLUG_STATUS_MOVED_BACK "moved_back"
#define WALKMAN_DRC_PLUG_STATUS_SAFE_EXITING "safe_exiting"
#define WALKMAN_DRC_PLUG_STATUS_SAFE_EXITED "safe_exited"
#define WALKMAN_DRC_PLUG_STATUS_GRASPING "grasping"
#define WALKMAN_DRC_PLUG_STATUS_GRASPED "grasped"
#define WALKMAN_DRC_PLUG_STATUS_UNGRASPING "ungrasping"
#define WALKMAN_DRC_PLUG_STATUS_UNGRASPED "ungrasped"

#define RAD2DEG    (180.0/M_PI)
#define DEG2RAD    (M_PI/180.0)

#define MIN_CLOSURE 0.0 * DEG2RAD
#define MAX_CLOSURE 600.0 * DEG2RAD

namespace walkman
{
    namespace drc
    {
	namespace plug
	{
	    struct robot_state_input
	    {
		yarp::sig::Vector q;
		yarp::sig::Vector q_dot;
		yarp::sig::Vector tau;
	    };

	    struct robot_state_output
	    {
		yarp::sig::Vector q;
		yarp::sig::Vector q_dot;
		yarp::sig::Vector tau;
	    };
	    
	    enum class state {
                idle,
                ready_stick,
		ready_hand,
		reaching,
		reached,
		approaching,
		approached,
		rotating,
		rotated,
		grasping,
		grasped,
		ungrasping,
		ungrasped,
		moving_away,
		moved_away,
		moving_back,
		moved_back,
		safe_exiting,
		safe_exited
	    };
	    
	    enum class mode {
		none,
	        stick,
		hand
	    };
	}
    }
}


#endif
