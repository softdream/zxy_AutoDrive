#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <auto_driver_msgs/ImuData.h>
#include <auto_driver_msgs/ControlCmd.h>
//#include <telecontrol/ChangeMode.h>
#include <autodrive/EmrgStop.h>
#include <autodrive/Dump.h>
#include <autodrive/ToDump.h>
#include <autodrive/ToLoad.h>
#include <autodrive/NetError.h>
#include <autodrive/StartRecord.h>
#include <autodrive/StopRecord.h>
#include "dataType.h"
#include "client.h"

typedef enum{
	startRecord = 1,
	stopRecord,
	dump,
	emrgStop,
	cancleEmrgStop,
	toLoad,
	toDump
}SrvState;// 

typedef enum{
	connected,
	unconnected,
}LinkState; // the link state between the TCP server and the client

Client client;// define a client 
int client_fd;
LinkState linkstate = unconnected;


float steeringLimit = 0; // steering_limit

HeartBeats heart;// define a heartbearts package message
void sendHeartBeats()
{
	heart.start1 = 'c';
	heart.start2 = 'c';
	heart.start3 = 'c';
	heart.start4 = 'c';
	heart.beating = 1;
	while(1){
		if( linkstate == connected ){
			if( client.sendData<HeartBeats>( client_fd, heart ) ){
				std::cout<<"fd: "<<client_fd<<std::endl;	
			}
			else{
				std::cerr<<"send HeartBearts failed ..."<<std::endl;
			}
		}
		sleep( 2 );
	}
}

bool GearNeedChange = false;
HarvesterDataType previous_cmd;
int8_t targetGear = 0;
/* convert the message type of T to the type of telecontrol::ControlCmd */
/* the message from the server need to be converted to the control command  */
template< typename T >
void msg2ControlCmd( T &cmd1, auto_driver_msgs::ControlCmd &cmd2 )
{
	cmd2.TargetThrottle = cmd1.x2 * 100;
	cmd2.TargetBrake = cmd1.x3 * 100;

	cmd2.TargetSteering =  cmd1.x1 * steeringLimit;
	//std::cout<<"steering limit = "<<steeringLimit<<std::endl;
	
	if( cmd1.io7 == 1 ){
		cmd2.TargetAuxSignal = 7;
	}
	if( cmd1.io8 == 1 ){
		cmd2.TargetAuxSignal = 7;
	}
	if( cmd1.io11 == 1 ){
		cmd2.TargetAuxSignal = 7;
	}

	if( cmd1.io20 == 0 && cmd1.io21 == 0 ){
		cmd2.TargetMode = 1;
	}
	else if( cmd1.io20 == 0 && cmd1.io21 == 1 ){
		cmd2.TargetMode = 0;
	}
	else if( cmd1.io20 == 1 && cmd1.io21 == 0 ){
		cmd2.TargetMode = 2;
	}
	
	if( cmd1.io4 == 0 ) {
		cmd2.TargetGear = 7;// R
		GearNeedChange = false;
	}
	else if( cmd1.io4 == 1 ) {
		cmd2.TargetGear = 0; // N
		GearNeedChange = false;
	}
	else if( cmd1.io4 == 6 ) {
		cmd2.TargetGear = 9; // P
		GearNeedChange = false;
	}

	if( cmd1.io4 == 2 ) {// D
		if( GearNeedChange == false ) {
			cmd2.TargetGear = 6;
			targetGear = 6;
		}
		else cmd2.TargetGear = targetGear;
	}
	if( cmd1.io4 == 2 && previous_cmd.io4 == 3  ) {
                targetGear ++;
                if( targetGear > 5 ) targetGear = 5;
                cmd2.TargetGear = targetGear;
                GearNeedChange = true;
        }
        else if( cmd1.io4 == 2 && previous_cmd.io4 == 4 ){
                targetGear --;
                if( targetGear < 1 ) targetGear = 1;
                cmd2.TargetGear = targetGear;
                GearNeedChange = true;
        }
        else if( cmd1.io4 == 3 || cmd1.io4 == 4 ){
                cmd2.TargetGear = targetGear;
        }


	cmd2.TargetSpeed = 0;
	cmd2.TargetEpb = 0;

	/*if( previous_cmd.io15 == 0 && cmd1.io15 == 1 ){
		cmd2.TargetPto = 1;
	}*/

	cmd2.TargetPto = cmd1.io15;
	
	if( cmd1.io19 == 1 ) cmd2.TargetEmrgStop = 1;
	else if( cmd1.io19 == 0 ) cmd2.TargetEmrgStop = 0;

	if( previous_cmd.io24 == 0 && cmd1.io24 == 1 ){
		ROS_INFO( "ShutDown --------------------------------------------------------- Shutdown" );
		system( "echo \"123456\" | sudo -S sh -c \"shutdown -h now\"" );// shutdown 
	}

	previous_cmd = cmd1;
}

HarvesterDataType previous_msg;
template< typename T >
void msg2Srv( T &msg, SrvState &state )
{

	if( previous_msg.io17 == 0 &&  msg.io17 == 1 ){
		std::cout<<"START record the path of the harvester ....................................... "<<(int)previous_msg.io17<<std::endl;
		state = startRecord;
	}
	if( previous_msg.io17 == 1 && msg.io17 == 0 ){
		std::cout<<"STOP record the pathe of the harvester ....................................... "<<(int)previous_msg.io17<<std::endl;
		state = stopRecord; 
	}

	/*if( previous_msg.io15 == 0 && msg.io15 == 1 ) {
		state = dump;
		std::cout<<"Dump .........................................................Dump "<<std::endl;
	}*/	

	if( previous_msg.io23 == 0 && msg.io23 == 1 ) {
		state = emrgStop;
		std::cout<<"Emergency STOP ..............................................EmrgSTOP"<<std::endl;
	}
	else if( previous_msg.io23 == 1 && msg.io23 == 0 ){
		state = cancleEmrgStop;
		std::cout<<"Cancle the Emergency STOP ..............................................Cancel EmrgSTOP"<<std::endl;
	}

	if( previous_msg.io16 == 0 && msg.io16 == 1 )
	{
		state = toLoad;
		std::cout<<"ToLoad ............................................................ToLoad"<<std::endl;
	}

	if( previous_msg.io14 == 0 && msg.io14 == 1 ){
		state = toDump;
		std::cout<<"ToDump ............................................................ToDump"<<std::endl;
	}

	previous_msg = msg;
}

/* IMU call back funtion */
void IMUCallback( const auto_driver_msgs::ImuData::ConstPtr &imu )
{
	MotionPlatformDataType imu_send;
        imu_send.start1 = 0x55;
        imu_send.start2 = 0xaa;
        imu_send.start3 = 0xbb;
        imu_send.cmd = 6;
        
        imu_send.rx = imu->Roll / 57.29578049044297 ;
        imu_send.ry = imu->Pitch / 57.29578049044297;
        imu_send.rz = imu->Yaw / 57.29578049044297;
        
        imu_send.tax = imu->AccX;
        imu_send.tay = imu->AccY;
        imu_send.taz = imu->AccZ;
        
        imu_send.rvx = imu->OmegaX / 57.29578049044297;
        imu_send.rvy = imu->OmegaY / 57.29578049044297;
        imu_send.rvz = imu->OmegaZ / 57.29578049044297;

	//imu_send.tz = imu->AccZ * 0.03f;

	imu_send.time = 50;

        int sendNum = client.sendData<MotionPlatformDataType>( client_fd, imu_send );
	if( sendNum > 0 ){
		std::cout<<"send IMU data to the server ..."<<std::endl;
	}
	else{
		std::cerr<<"send IMU data failed ..."<<std::endl;	
	}
}


/* thread of send IMU data to the TCP server */
void sendIMUData()
{
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<auto_driver_msgs::ImuData>("/imu/imu_data", 1000, IMUCallback);
	ros::spin();
}

/* exit signal process */
void mySignalHandle( int sig )
{
	MotionPlatformDataType imu_send;
        memset( &imu_send, 0, sizeof( imu_send ) );
        imu_send.start1 = 0x55;
        imu_send.start2 = 0xaa;
        imu_send.start3 = 0xbb;
        imu_send.cmd = 4;
        imu_send.rx = 20;
        imu_send.ry = 20;
        imu_send.time = 50;
        int sendNum = client.sendData<MotionPlatformDataType>( client_fd, imu_send );
        if( sendNum > 0 ){
                ROS_INFO( "send command 4 to the chair" );
        }
        sleep(1);

        client.closeClient();// close the tcp connecion
        ROS_INFO( "exit the TCP connection to the Server program ...." );
	client.closeClient();
        ros::shutdown();
        exit( sig );
}


int main( int argc, char **argv )
{
	int error_count = 0;
	ROS_INFO( "Program started ..." );
	ros::init( argc, argv, "telecontrol" );
	ros::NodeHandle n;	

	n.getParam( "steering_limit", steeringLimit );

	/* wait the exit signal */
        signal( SIGTERM | SIGINT | SIGKILL | SIGHUP | SIGABRT, mySignalHandle );

	/* Publish the control command */
	ros::Publisher pub = n.advertise<auto_driver_msgs::ControlCmd>( "/tele/control_cmd", 1);

	/* service of the Dump  */
	ros::ServiceClient DumpClient = n.serviceClient<autodrive::Dump>("/chassis/dump");
	autodrive::Dump DumpSrv;

	/* service of the NetError */
	ros::ServiceClient NetErrorClient = n.serviceClient<autodrive::NetError>("/networkerror");
	autodrive::NetError NetErrorSrv;

	/* service of the StartRecord */
	ros::ServiceClient StartRecordClient = n.serviceClient<autodrive::StartRecord>( "/recorder/start_record" );
	autodrive::StartRecord startRecordSrv;

	/* service of the StopRecord */
	ros::ServiceClient StopRecordClient = n.serviceClient<autodrive::StopRecord>( "/recorder/stop_record" );
	autodrive::StopRecord stopRecordSrv;

	/* service of Emergency Stop */
	ros::ServiceClient EmrgStopClient = n.serviceClient<autodrive::EmrgStop>( "/chassis/emrg_stop" );
	autodrive::EmrgStop emrgStopSrv;

	/* service of ToLoad */
	ros::ServiceClient ToLoadClient = n.serviceClient<autodrive::ToLoad>("/chassis/toLoad");
	autodrive::ToLoad toLoadSrv;

	/* service of ToDump */
	ros::ServiceClient ToDumpClient = n.serviceClient<autodrive::ToDump>("/chassis/toDump");
	autodrive::ToDump toDumpSrv;

	/* start the TCP client */
	client_fd = client.initClient();
	if( client_fd  > 0){
		linkstate = connected;
	}
	else{
		linkstate = unconnected;
	}

	/* create two threads */
	std::thread sendHeart( sendHeartBeats );
	std::thread sendIMU( sendIMUData );	

	while(ros::ok()){
		if( linkstate == connected ){
			int ret = client.recvFromServer( client_fd );// receive the data from the TCP server
			if ( ret == 1 ){
				HarvesterDataType harvester = client.getHarvester();// get the harvester data 
				
				SrvState state;
				memset( &state, 0, sizeof( state ) );
			       	msg2Srv<HarvesterDataType>( harvester, state );
				/*
				if( state == startRecord ) {
					if( StartRecordClient.call( startRecordSrv ) ){ // if the key is be pressed, call the startRecord service
                                                std::cout<<"call the startRecord service successfully ..."<<std::endl;
                                        }
				}
				else if( state == stopRecord ) {
					if( StopRecordClient.call( stopRecordSrv ) ){// if the key is be unpressed, call the stopRecord service
                                                std::cout<<"call the stopRecord service successfully ..."<<std::endl;
                                        }

				}
				//else if( state == dump ) {
				//	DumpSrv.request.cmd = 1;
				//	if( DumpClient.call( DumpSrv ) ) {
				//		std::cout<<"call the Dump service successfully ..."<<std::endl;
				//	}
				//}
				else if( state == emrgStop ) {
					emrgStopSrv.request.type = 1;
					if( EmrgStopClient.call( emrgStopSrv ) ) {
						std::cout<<"call the Emergency stop successfully ..."<<std::endl;
					}
				}
				else if( state == cancleEmrgStop) {
					emrgStopSrv.request.type = 0;
					if( EmrgStopClient.call( emrgStopSrv ) ){
						std::cout<<"Cancel the Emergency stop service successfully..."<<std::endl;
					}
				}
				else if( state == toLoad ) {
					if( ToLoadClient.call( toLoadSrv ) ){
						std::cout<<"call the ToLoad service successfully ..."<<std::endl;
					}
				}
				else if( state == toDump ) {
					if( ToDumpClient.call( toDumpSrv ) ){
						std::cout<<"call the ToDump service successfully ..."<<std::endl;
					}
				} */
				switch( state ){
					case startRecord: {
						if( StartRecordClient.call( startRecordSrv ) ){ // if the key is be pressed, call the startRecord service
        	                                        std::cout<<"call the startRecord service successfully ..."<<std::endl;
	      	                                }
						break;
					}
					case stopRecord: {
						if( StopRecordClient.call( stopRecordSrv ) ){// if the key is be unpressed, call the stopRecord service
                                                	std::cout<<"call the stopRecord service successfully ..."<<std::endl;
	                                        }	
					       break;	
					}
					case emrgStop: {
						emrgStopSrv.request.type = 1;
	                                        if( EmrgStopClient.call( emrgStopSrv ) ) {
        	                                        std::cout<<"call the Emergency stop successfully ..."<<std::endl;
                	                        }
						break;
					}
					case cancleEmrgStop: {
						emrgStopSrv.request.type = 0;
	                                        if( EmrgStopClient.call( emrgStopSrv ) ){
        	                                        std::cout<<"Cancel the Emergency stop service successfully..."<<std::endl;
                	                        }
		 				break;
					}
					case toLoad: {
						if( ToLoadClient.call( toLoadSrv ) ){
        	                                        std::cout<<"call the ToLoad service successfully ..."<<std::endl;
	                                        }
	     					break;
					}
					case toDump: {
						if( ToDumpClient.call( toDumpSrv ) ){
                	                                std::cout<<"call the ToDump service successfully ..."<<std::endl;
        	                                }
		     				break;
					}
					default: break;
				}

				auto_driver_msgs::ControlCmd cmd;
				msg2ControlCmd<HarvesterDataType>( harvester, cmd ); // process the message
				if( harvester.io3 == 0 && harvester.io13 == 0 ){
					std::cerr<<"ERRORDATA-------------------------------------------------------ERRORDATA"<<std::endl;
				}
				else pub.publish( cmd );// publish the command
			}
	
			if( ret == 3 ){
				error_count ++;
				if( error_count == 5 ){
					std::cerr<<"connection to the server is broken ..."<<std::endl;
					error_count = 0;
					linkstate = unconnected;
					
					NetErrorSrv.request.type = 1;
					if( NetErrorClient.call( NetErrorSrv ) ){ // if the connection between the TCP server and the client, call the NetError service
			
					}
					client.closeClient(); // close the socket
				}
			}
		}
		else if( linkstate == unconnected ) {
			client_fd = client.initClient();// reconnect
			if( client_fd > 0 ){
				std::cout<<"reconnected client fd = "<<client_fd<<std::endl;
				NetErrorSrv.request.type = 0;
				if( NetErrorClient.call( NetErrorSrv ) ){ // if the connection between the TCP server and the client is be reconnected, call the NetError service 
					
				}
				linkstate = connected;	// update the link state
			}
			else{
				std::cerr<<"reconnected failed ..."<<std::endl;
			}
			sleep(2);
		}
	}

	MotionPlatformDataType imu_send;
        memset( &imu_send, 0, sizeof( imu_send ) );
        imu_send.start1 = 0x55;
        imu_send.start2 = 0xaa;
        imu_send.start3 = 0xbb;
        imu_send.cmd = 4;
        imu_send.rx = 20;
        imu_send.ry = 20;
        imu_send.time = 50;
        int sendNum = client.sendData<MotionPlatformDataType>( client_fd, imu_send );
        if( sendNum > 0 ){
                ROS_INFO( "send command 4 to the chair" );
        }
        sleep(1);
	client.closeClient();
	return 0;
}
