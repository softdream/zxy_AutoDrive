#include "lidarProcess.h"

int main( int argc, char **argv )
{
	ros::init(argc, argv, "lidar_process_node");

	LidarProcess lidarProcess;

	ros::spin();
	return 0;
}
