#ifndef __CLIENT_H_
#define __CLIENT_H_

#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "dataType.h"

class Client{
public:
	Client(){}
	~Client(){
		close( sockfd );
	}
	
	int  initClient();
	
	template< typename T >
	int sendData( const int &fd, T &data );
	int recvFromServer( const int &fd );
	bool closeClient(  )
	{
		if ( close( sockfd ) == 0 ){
			return true;
			std::cerr<<"close the client socket ..."<<std::endl;
			ROS_INFO("Close the client socket ...");
		}
		else{
			return false;
		}
	}
	HarvesterDataType getHarvester(){
		return harvester;
	}

private:
	int sockfd;
	uint8_t recvBuff[256];
	uint8_t sendBuff[256];
	struct sockaddr_in server_addr;
	HarvesterDataType harvester;
};

int Client::initClient()
{
	if( ( sockfd = socket( AF_INET, SOCK_STREAM, 0 ) ) == -1 ){
		std::cout<<"socket error ..."<<std::endl;
		return false;
	}
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons( 4999 );
	server_addr.sin_addr.s_addr = inet_addr( "172.16.0.240" );
	//server_addr.sin_addr.s_addr = inet_addr( "192.168.43.20" );
	bzero( &( server_addr.sin_zero ), sizeof( server_addr.sin_zero ) );

	if (connect(sockfd, (struct sockaddr *)&server_addr,sizeof(struct sockaddr_in)) == -1){
       		std::cout<<"connect error ..."<<std::endl;
        	return false;
     	}
	
	std::cout<<"connet successfull ... "<< sockfd <<std::endl;
	return sockfd;
}


template<typename T>
int Client::sendData( const int &fd, T &data )
{
	memset( sendBuff, 0, sizeof( sendBuff ) );
	memcpy( sendBuff, &data, sizeof( data ) );
	int sendNum = send( fd, sendBuff, sizeof( data ), 0 );
	if( sendNum > 0 ){
		std::cout<<"send data ..."<<sendNum<<std::endl;
		return sendNum;
	}
	else{
		return false;
	}
}

int Client::recvFromServer( const int &fd )
{
	memset( recvBuff, 0, sizeof( recvBuff ) );
	//int recvNum = recv( fd, ( char* )recvBuff, sizeof( recvBuff ), 0 );
	int recvNum = recv( fd, ( char* )recvBuff, sizeof( HarvesterDataType ), 0 );
	if( recvNum > 0 ){
		std::cout<<"receive "<<recvNum<<" bytes from server ..."<<std::endl;
		if( recvBuff[0] == 0xAA && recvBuff[1] == 0xBB && recvBuff[2] == 0xCC && recvBuff[3] == 0xDD ){
			std::cout<<"receive a harvester data ..."<<std::endl;
			memset( &harvester, 0, sizeof( harvester ) );
			memcpy( &harvester, recvBuff, sizeof( harvester ) );
			std::cout<<"harvester.x1 = "<<harvester.x1<<std::endl;
			return 1;
		}
		if( recvBuff[0] = 'c' && recvBuff[1] == 'c' && recvBuff[2] == 'c' && recvBuff[3] == 'c' ){
			std::cout<<"receive a heartBeats data ..."<<std::endl;
			return 2;
		}
	}
	else{
		std::cerr<<"error =================================== "<<recvNum<<std::endl;
		return 3;
	}
}

#endif










