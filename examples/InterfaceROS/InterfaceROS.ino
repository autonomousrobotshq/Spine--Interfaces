#include <InterfaceROS.hpp>

#define UPDATE_INTERVAL 1000 // update every 1 seconds
#define BAUDRATE 115200

InterfaceROS ROS(UPDATE_INTERVAL);

void setup() {
	ROS.Init(BAUDRATE);
}

void loop() {
	ROS.Log(InterfaceROS::e_loglevel::CRIT, "CRITICAL ERROR");
}
