#ifndef __MMWR_H_
#define __MMWR_H_

#include <ros/ros.h>
#include <auto_driver_msgs/ControlCmd.h>
#include <auto_driver_msgs/ImuData.h>
#include <auto_driver_msgs/VcuData.h>

// VCI inclueds
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "pcie_can/controlcan.h"

class MMWR_Process{
public:
	MMWR_Process();
	MMWR_Process( const MMWR_Process &obj );
	~MMWR_Process();

	MMWR_Process& operator=( const MMWR_Process &other )
	{
		if (this == &other) return *this;// 检测自赋值
		nDeviceType = other.nDeviceType;
		nDeviceInd = other.nDeviceInd;
		nCANInd = other.nCANInd;
	
	        vic.AccCode = other.vic.AccCode;
	        vic.AccMask = other.vic.AccMask;
        	vic.Filter = other.vic.Filter;
    	    	vic.Timing0 = other.vic.Timing0;
       		vic.Timing1 = other.vic.Timing1;
        	vic.Mode = other.vic.Mode ;

		recvBuff = new VCI_CAN_OBJ[10];
		*recvBuff = *( other.recvBuff );
		return *this;
	}

	bool openMMWR();
	bool initMMWR();
	VCI_BOARD_INFO readBoardInfo();
	bool startCan();
	void detectObjections();

private:
	/* variables of the ros */
	ros::NodeHandle nh;
	ros::Publisher cluster_pub;	
	
	/* variables of the CAN  communication */
	int nDeviceType; // 设备类型
	int nDeviceInd; // 设备号
	int nCANInd;   // 通道号

	VCI_INIT_CONFIG vic;
	VCI_BOARD_INFO vbi;
	VCI_CAN_OBJ *recvBuff;// 定义帧接收缓存
	
};

/* Construction */
MMWR_Process::MMWR_Process() : nDeviceType( 4 ),
				nDeviceInd( 0 ),
				nCANInd( 0 )
{
	vic.AccCode = 0x804;
	vic.AccMask = 0xFFFFFFFF;
	vic.Filter = 1;
	vic.Timing0 = 0x00;
	vic.Timing1 = 0x1C;
	vic.Mode = 0;
	
	recvBuff = new VCI_CAN_OBJ[ 100 ];

	//cluster_pub = nh.advertise<sensor_msgs::xxxxxxx>( "xxxxx", 1 );
	// }还没想好发布的消息格式
}

/* Copy-Construction */
MMWR_Process::MMWR_Process( const MMWR_Process &obj ) : nDeviceType( obj.nDeviceType ),
						nDeviceInd( obj.nDeviceInd ),
						nCANInd( obj.nCANInd )
						
{
	vic.AccCode = obj.vic.AccCode;
	vic.AccMask = obj.vic.AccMask;
	vic.Filter = obj.vic.Filter;
	vic.Timing0 = obj.vic.Timing0;
	vic.Timing1 = obj.vic.Timing1;
	vic.Mode = obj.vic.Mode ;
	
	recvBuff = new VCI_CAN_OBJ[ 100 ];
	*recvBuff = *( obj.recvBuff );
}

MMWR_Process::~MMWR_Process()
{
	assert( recvBuff != NULL );
	delete[] recvBuff;
	VCI_CloseDevice( nDeviceType, nDeviceInd );
}

bool MMWR_Process::openMMWR()
{
	int dwRel = VCI_OpenDevice( nDeviceType, nDeviceInd, 0 );
	if (dwRel != 1) {
		std::cerr << "Open device failed ..." << std::endl;
		return false;
	}
	else {
		std::cerr << "Open device ..." << std::endl;
		return true;
	}
}

bool MMWR_Process::initMMWR()
{
	int dwRel = VCI_InitCAN( nDeviceType, nDeviceInd, nCANInd, &vic );
	if (dwRel != 1) {
		std::cerr << "init CAN failed ..." << std::endl;
		return false;
	}
	else {
		std::cerr << "init CAN ..." << std::endl;
		return true;
	}
}

VCI_BOARD_INFO MMWR_Process::readBoardInfo()
{
	int dwRel = VCI_ReadBoardInfo( nDeviceType, nDeviceInd, &vbi );
	if (dwRel != 1) {
		std::cerr << "get the information of the device failed ..." << std::endl;
		exit( -1 );
	}
	else {
		std::cout << "get the information of the device ..." << std::endl;
		std::cout << "hw_Version = " << vbi.hw_Version << std::endl;
		std::cout << "fw_Version = " << vbi.fw_Version << std::endl;
		std::cout << "dr_Version = " << vbi.dr_Version << std::endl;
		std::cout << "in_Version = " << vbi.in_Version << std::endl;
		std::cout << "irq_Num = " << vbi.irq_Num << std::endl;
		std::cout << "can_Num = " << ( int )vbi.can_Num << std::endl;
		std::cout << "the str_Serial_Num = " << vbi.str_Serial_Num << std::endl;
		std::cout << "the str_hw_Type = " << vbi.str_hw_Type << std::endl;
	}
	return vbi;
}

bool MMWR_Process::startCan()
{
	if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd) != 1) {
		VCI_CloseDevice( nDeviceType, nDeviceInd );
		std::cerr << "start deveice failed ..." << std::endl;
		return false;
	}
	else {
		std::cerr << "start deveice successfully ..." << std::endl;
		return true;
	}
}

void MMWR_Process::detectObjections()
{
	if( !openMMWR() ) exit( -1 );
	if( !initMMWR() ) exit( -1 );
	readBoardInfo(); // get the information of the mmwr 
	if( !startCan() ) exit( -1 );
	
	while( 1 ) {
		int dwRel = VCI_Receive( nDeviceType, nDeviceInd, nCANInd, recvBuff, 2500, 0 );
		if (  dwRel > 0 ) {
			std::cout << "receive a frame data ... " << dwRel << std::endl;
			for (int i = 0; i < dwRel; i++) {
				//std::cout << "message ID: " << recvBuff[i].ID << std::endl;
				if (recvBuff[i].ID == 1546) {
					std::cout << "object list: Number of objects: " << ( int )recvBuff[i].Data[0] << std::endl;
					std::cout << "message len: " << ( int )recvBuff[i].DataLen << std::endl;
				}
				if (recvBuff[i].ID == 1547) {
					std::cout << "there is a object: ID: " << ( int )recvBuff[i].Data[0] << std::endl;
					float  object_dist_long_ = ((uint16_t)(recvBuff[i].Data[1] << 5) + (uint8_t)((recvBuff[i].Data[2] & 0xf8) >> 3)) * 0.2 - 500;
					float object_dist_lat_ = ((uint16_t)((recvBuff[i].Data[2] & 0x07) << 8) + (uint8_t)(recvBuff[i].Data[3])) * 0.2 - 204.6;
					float object_vre_long_ = ((uint16_t)(recvBuff[i].Data[4] << 2) + (uint8_t)((recvBuff[i].Data[5] & 0xc0) >> 6)) * 0.25 - 128;
					float object_vre_lat_ = ((uint16_t)((recvBuff[i].Data[5] & 0x3f) << 3) + (uint8_t)((recvBuff[i].Data[6] & 0xe0) >> 5)) * 0.25 - 64;
					
					std::cout << "dist_longitudinal = " << object_dist_long_ << std::endl;
					std::cout << "dist_lateral = " << object_dist_lat_ << std::endl;
					std::cout << "velocity_longitudinal = " << object_vre_long_ << std::endl;
					std::cout << "velocity_lateral = " << object_vre_lat_ << std::endl;
				}
			}
		}
		else if (dwRel == -1) {
			std::cout << "the device is offline ..." << std::endl;
		}

		//cluster_pub.publish( "还没想好发布的消息格式 ..." );
	}
}

#endif


