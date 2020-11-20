// ROS includes
#include "MMWR.h"


int main( int argc, char **argv )
{
	ros::init(argc, argv, "MMWR_node");
	MMWR_Process *mmwrProcess;
	
	mmwrProcess->detectObjections();	

	ros::spin();
	return 0;
}

