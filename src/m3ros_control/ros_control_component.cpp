#include "m3ros_control/ros_control_component.h"

namespace ros_control_component{
	
using namespace m3rt;
using namespace std;
using namespace m3;

bool RosControlComponent::LinkDependentComponents()
{ 
	//Need to find at least one arm
	bot_shr_ptr_=(m3::M3Humanoid*) factory->GetComponent(bot_name_);
	if (bot_shr_ptr_==NULL)
		m3rt::M3_INFO("M3Humanoid component %s not found for component %s\n",bot_name_.c_str(),GetName().c_str());
	if (bot_shr_ptr_==NULL)
		return false;
	
	return true;
}

void RosControlComponent::Startup()
{	
	period_.fromSec(1/static_cast<double>(RT_TASK_FREQUENCY));
	RosInit(bot_shr_ptr_); //NOTE here the bot_shr_ptr_ is correctly loaded
}

void RosControlComponent::Shutdown()
{
	RosShutdown();
}
						  
bool RosControlComponent::ReadConfig(const char* cfg_filename)
{
	//YAML::Node doc;
	if (!M3Component::ReadConfig(cfg_filename))
		return false;
	//GetYamlDoc(cfg_filename, doc);
	doc["humanoid"] >> bot_name_;
	doc["hw_interface_mode"] >> hw_interface_mode_;
	if(hw_interface_mode_=="effort" ||  hw_interface_mode_=="position")
		m3rt::M3_INFO("Selected hardware interface mode %s for component %s\n",hw_interface_mode_.c_str(),GetName().c_str());
	else
	{
		m3rt::M3_INFO("Wrong hardware interface mode %s for component %s\n",hw_interface_mode_.c_str(),GetName().c_str());
		return false;
	}
	
	return true;
}

void RosControlComponent::StepStatus()
{
	hw_ptr_->read();
	cm_ptr_->update(ros::Time::now(),period_);
}

void RosControlComponent::StepCommand()
{	
	hw_ptr_->write();
}

}
