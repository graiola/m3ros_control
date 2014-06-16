
#include <stdio.h>
#include <m3rt/base/component.h>
#include "m3ros_control/ros_control_component.h"


///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 
#define ROS_CONTROL_COMPONENT_NAME "ros_control_component"
///////////////////////////////////////////////////////
//Creators
m3rt::M3Component* create_ros_control_component(){return new ros_control_component::RosControlComponent;}

//Deletors
void destroy_ros_control_component(m3rt::M3Component* c) {delete c;}

///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[ROS_CONTROL_COMPONENT_NAME] = create_ros_control_component;
		m3rt::destroyer_factory[ROS_CONTROL_COMPONENT_NAME] =  destroy_ros_control_component;
	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
