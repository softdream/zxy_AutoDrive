#ifndef __DATATYPE_H_
#define __DATATYPE_H_

#include <stdint.h>
#include <netinet/in.h>

/* 接收/发送电铲的数据 */
typedef struct {
	uint8_t start1;
	uint8_t start2;
	uint8_t start3;
	uint8_t start4;
	uint8_t io1;
	uint8_t io2;
	uint8_t io3;
	uint8_t io4;
	uint8_t io5;
	uint8_t io6;
	uint8_t io7;
	uint8_t io8;
	uint8_t io9;
	uint8_t io10;
	uint8_t io11;
	uint8_t io12;
	uint8_t io13;
	uint8_t io14;
	uint8_t io15;
	uint8_t io16;
	uint8_t io17;
	uint8_t io18;
	uint8_t io19;
	uint8_t io20;
	uint8_t io21;
	uint8_t io22;
	uint8_t io23;
	uint8_t io24;
	uint8_t io25;
	uint8_t io26;
	uint8_t io27;
	uint8_t io28;
	uint8_t io29;
	uint8_t io30;
	uint8_t io31;
	uint8_t io32;
	uint8_t io33;
	uint8_t io34;
	uint8_t io35;
	uint8_t io36;
	uint8_t io37;
	uint8_t io38;
	uint8_t io39;
	uint8_t io40;

	float x1;
	float x2;
	float x3;
	float x4;
	float x5;

	float var1;
	float var2;
	int var3;

}ExcavatorDataType;

/* 接收/发送矿车的数据 */
typedef struct {
	uint8_t start1;
	uint8_t start2;
	uint8_t start3;
	uint8_t start4;
	uint8_t io1;
	uint8_t io2;
	uint8_t io3;
	uint8_t io4;
	uint8_t io5;
	uint8_t io6;
	uint8_t io7;
	uint8_t io8;
	uint8_t io9;
	uint8_t io10;
	uint8_t io11;
	uint8_t io12;
	uint8_t io13;
	uint8_t io14;
	uint8_t io15;
	uint8_t io16;
	uint8_t io17;
	uint8_t io18;
	uint8_t io19;
	uint8_t io20;
	uint8_t io21;
	uint8_t io22;
	uint8_t io23;
	uint8_t io24;
	uint8_t io25;
	uint8_t io26;
	uint8_t io27;
	uint8_t io28;
	uint8_t io29;
	uint8_t io30;
	uint8_t io31;
	uint8_t io32;
	uint8_t io33;
	uint8_t io34;
	uint8_t io35;
	uint8_t io36;
	uint8_t io37;
	uint8_t io38;
	uint8_t io39;
	uint8_t io40;

	float x1;
	float x2;
	float x3;
	float x4;
	float x5;

	float var1;
	float var2;
	int var3;

}HarvesterDataType;

typedef struct {
	uint8_t start1;
	uint8_t start2;
	uint8_t start3;
	uint8_t cmd;
	float tx;
	float ty;
	float tz;
	float rx;
	float ry;
	float rz;
	float tax;
	float tay;
	float taz;
	float rvx;
	float rvy;
	float rvz;
	int time;
}MotionPlatformDataType;


typedef struct{
	uint8_t start1;
	uint8_t start2;
	uint8_t start3;
	uint8_t start4;
	char ipaddr[INET_ADDRSTRLEN];
	uint8_t io1;
	uint8_t io2;
	uint8_t io3;
	uint8_t io4;
	uint8_t io5;
	uint8_t io6;
	uint8_t io7;
	uint8_t io8;
	uint8_t io9;
	uint8_t io10;
}IPAddrInfo;


typedef struct{
	uint8_t start1;
	uint8_t start2;
	uint8_t start3;
	uint8_t start4;
	uint32_t beating;
}HeartBeats;

#endif
