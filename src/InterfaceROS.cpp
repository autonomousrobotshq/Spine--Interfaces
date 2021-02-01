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

#include "InterfaceROS.hpp"

InterfaceROS::InterfaceROS(const unsigned long update_interval)
    : Interface(update_interval)
{
}

InterfaceROS::~InterfaceROS()
{
}

bool InterfaceROS::Init(const unsigned long baudrate)
{
	_nodehandle.getHardware()->setBaud(baudrate);
	_nodehandle.initNode();
	return (_nodehandle.spinOnce());
}

void InterfaceROS::AddSubscriber(ros::Subscriber_& s)
{
    _nodehandle.subscribe(s);
}

void InterfaceROS::AddPublisher(ros::Publisher& p)
{
    _nodehandle.advertise(p);
}

bool InterfaceROS::IsConnected()
{
    return (_nodehandle.connected());
}

bool InterfaceROS::Update()
{
    if (!IsTimeToExecute())
        return (true);

    if (!IsConnected()) {
		_state = InterfaceROS::DISCONNECTED;
		return (false);
    }
    return (_nodehandle.spinOnce() == 0);
}

bool InterfaceROS::AttemptConnect()
{
    if (!this->IsConnected()) {
        _nodehandle.spinOnce();
	}
	if (this->IsConnected()) {
		_state = InterfaceROS::CONNECTED;
		return (true);
	}
	else {
		_state = InterfaceROS::DISCONNECTED;
		return (false);
	}
}

InterfaceROS::e_state InterfaceROS::GetState()
{
	return (_state);
}

void InterfaceROS::Log(const e_loglevel level, const char* msg)
{
    switch (level) {
    case DEBUG:
        _nodehandle.logdebug(msg);
        break;
    case INFO:
        _nodehandle.loginfo(msg);
        break;
    case WARN:
        _nodehandle.logwarn(msg);
        break;
    case ERROR:
        _nodehandle.logerror(msg);
        break;
    case CRIT:
        _nodehandle.logfatal(msg);
        break;
    }
}

ros::NodeHandle InterfaceROS::_nodehandle;
