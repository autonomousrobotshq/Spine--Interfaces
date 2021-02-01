//
// Spine - Spine - MCU code for robotics.
// Copyright (C) 2019-2021 Codam Robotics
//
// This file is part of Spine.
//
// Spine is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Spine is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Spine.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef INTERFACE_ROS_HPP
#define INTERFACE_ROS_HPP

#include <stdint.h>
#include <ros.h>
#include "Interface.hpp"

class InterfaceROS : public Interface {
	public:
		enum e_state {
			CONNECTED,
			DISCONNECTED
		};
		enum e_loglevel {
			DEBUG,
			INFO,
			WARN,
			ERROR,
			CRIT
		};
	    InterfaceROS(const unsigned long update_interval);
	    ~InterfaceROS();
		bool Init(const unsigned long baudrate = 115200);
	    void AddPublisher(ros::Publisher& p);
	    void AddSubscriber(ros::Subscriber_& s);
	
	    void Log(const e_loglevel level, const char* msg);
	    bool Update();
	    bool IsConnected();
		bool AttemptConnect();
		e_state GetState();
	
	private:
		static ros::NodeHandle _nodehandle;
		e_state _state;
};

#endif
