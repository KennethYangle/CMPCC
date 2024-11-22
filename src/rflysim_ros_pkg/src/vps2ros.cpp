/* 发布器，从socket接收导航数据并发布到vpsnavdata话题
*/
#include <sys/socket.h>   // socket 头文件
#include <arpa/inet.h> // socket 常量头文件
#include <errno.h> // 获取socket错误码
#include <cstdlib> // argc,argv c语言标准库文件
#include <iostream> 
#include <sys/time.h>
#include <sys/ioctl.h>
#include <map>
#include <string.h> // memset 函数头文件
#include <unistd.h> // sleep 函数头文件
#include "ros/ros.h" // ros包头文件
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/console.h>
#include "ros/message_operations.h"

#include "std_msgs/Int64.h"

#include <ros/console.h>
using std::map;

//#define MULTICAST

// 定义socket文件描述符、接收字节长度、socket地址
int socket_srv = -1;
struct sockaddr_in address_srv, from;
typedef std::map<int,  ros::Publisher*> CPublisherContainer;
unsigned short GenerateCRCCode(const unsigned char * pBuf, int nLength)
{
	unsigned short wCrc = (unsigned short)0xFFFF;

	for(int i = 0; i < nLength; i++) 
	{
		wCrc ^= (unsigned short)(unsigned char)pBuf[i];

		for(int j = 0; j < 8; j++)
		{
			if(wCrc & 1)
			{
				wCrc >>= 1; 
				wCrc ^= 0xA001;
			}
			else {
				wCrc >>= 1;
			}
		}
	}

	return wCrc;
}

bool SetSocketTimeout()
{
	timeval tm;
	fd_set set;
	tm.tv_sec= 0;
	tm.tv_usec = 5000;
	FD_ZERO(&set);
	FD_SET(socket_srv, &set);
	int nSelectRet =  select(socket_srv+1, &set, NULL, NULL, &tm);
	if(  -1 == nSelectRet )
	{
	       int error=-1, len = sizeof(int);
	       getsockopt(socket_srv, SOL_SOCKET, SO_ERROR, &error, (socklen_t *)&len);
	       std::cout<<"select fail"<<error<<"\n";
	       return false;
	}
              
    else if(0 == nSelectRet)
			return false;
    return true;

}
bool InitSocket()
{
	
	
	// 初始化服务端UDP socket并获取其文件描述符
            socket_srv = socket(AF_INET,SOCK_DGRAM,0);
            if (socket_srv == -1) 
            {
		std::cout << "Unable to create a valid socket. EXIT." << std::endl;
        		return false; 
	}
	
	unsigned long ul = 1;
	
#ifndef MULTICAST	   
	ioctl(socket_srv, FIONBIO, &ul); //设置为非阻塞模式         

	bzero(&address_srv, sizeof(struct sockaddr_in));
	address_srv.sin_family = AF_INET;
	address_srv.sin_addr.s_addr = htonl(INADDR_ANY);
	address_srv.sin_port = htons(20005);
	
	// 广播地址

	bzero(&from, sizeof(struct sockaddr_in));
	from.sin_family = AF_INET;
	from.sin_addr.s_addr = htonl(INADDR_ANY);
	from.sin_port = htons(20005);
	const int opt = 1;
	//设置该套接字为广播类型，
	int nb = 0;
	nb = setsockopt(socket_srv, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt));
	if(nb == -1)
	{
		ROS_ERROR("set socket error...");
		return false;
	}
 
	if(bind(socket_srv,(struct sockaddr *)&(address_srv), sizeof(struct sockaddr_in)) == -1) 
	{   
		ROS_ERROR("bind error...");
		return false;
	}

	
#else
	int reuse = 1;
	if(setsockopt(socket_srv, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(reuse)) < 0){
		perror("Setting SO_REUSEADDR error");
		close(socket_srv);
		exit(1);
	} 
	else
		printf("Setting SO_REUSEADDR...OK.\n");



	memset((char *) &address_srv, 0, sizeof(address_srv));
	address_srv.sin_family = AF_INET;
	address_srv.sin_port = htons(5500);
	address_srv.sin_addr.s_addr = INADDR_ANY;
	if(bind(socket_srv, (struct sockaddr*)&address_srv, sizeof(address_srv)))
	{
		perror("Binding datagram socket error");
		close(socket_srv);
		exit(1);
	}
	else
		printf("Binding datagram socket...OK.\n");
	struct ip_mreq group;
	group.imr_multiaddr.s_addr = inet_addr("226.1.1.1");
	group.imr_interface.s_addr =  htonl(INADDR_ANY);
	if(setsockopt(socket_srv, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0)
	{
		perror("Adding multicast group error");
		close(socket_srv);
		exit(1);
	}
	else
		printf("Adding multicast group...OK.\n");
  

#endif
	return true;
}

void releasePublisher(CPublisherContainer *pPubContainer)
{
     for (CPublisherContainer::const_iterator it = pPubContainer->begin();
            it != pPubContainer->end(); ++it )
     {
     	delete  it->second;
     }

     pPubContainer->clear();
}

ros::Publisher * findPub( CPublisherContainer  *pPubContainer, int nRigId)
{
       CPublisherContainer::const_iterator it = pPubContainer->find(nRigId);
       if(it != pPubContainer->end())
       {
       	return  it->second;
       }

       return 0;
}

#include <cstdint> // include this header for uint64_t

//把八个char转为一个double
double eight_char_to_double(unsigned char a, unsigned char b, unsigned char c, unsigned char d, unsigned char e, unsigned char f, unsigned char g, unsigned char h)
{
    double ret_val = 0.0;
    char chs[8];
    chs[0] = a;
    chs[1] = b;
    chs[2] = c;
    chs[3] = d;
    chs[4] = e;
    chs[5] = f;
    chs[6] = g;
    chs[7] = h;
    ret_val = *(double *)chs;
    return ret_val;
}

struct RflyTimeStmp{
    int checksum; //校验位，取123456789
    int copterID; //当前飞机的ID号
    long long SysStartTime; //Windows下的开始仿真时的时间戳（单位毫秒，格林尼治标准起点）
    long long SysCurrentTime;//Windows下的当前时间戳（单位毫秒，格林尼治标准起点）
    long long HeartCount; //心跳包的计数器
    RflyTimeStmp(){
        reset();
    }
    void reset(){
        checksum=123456789;
        copterID=-1;
        SysStartTime=-1;
        SysCurrentTime=-1;
        HeartCount=0;
    }
};
int main(int argc, char **argv) {

// 初始化发布器
	if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug))
	{
		ros::console::notifyLoggerLevelsChanged();
	}
	
	ros::init(argc, argv, "vps_driver");            
	

	ros::NodeHandle vps;

        CPublisherContainer  pubContainer[3] ;
/******************************************************************************/
	InitSocket();
/***********************************************************************/	


            // socket地址长度必须定义为"unsigned int"类型或 "socklen_t"类型
	socklen_t len_caddr=sizeof(from); 
	int nSizeoDouble = sizeof(double);
	int nSizeOfInt = sizeof(int);

	std::string sec = "wintime";
	ros::Publisher pub_sec = vps.advertise<std_msgs::Int64>(sec, 1000);

	while (ros::ok()) 
	{	
		char szRecvBuf[128] = {0};

		int recv_len = recvfrom(socket_srv, (char*)szRecvBuf, sizeof( szRecvBuf), 0, (struct sockaddr *) &from,&len_caddr);  
		if (-1 == recv_len )
		{
			continue;
		}

		std_msgs::Int64 systime;
		
		RflyTimeStmp times;
		memcpy(&times, szRecvBuf, sizeof(struct RflyTimeStmp));
		systime.data = times.SysStartTime;
		pub_sec.publish(systime);	

		// ROS_ERROR("msec: %lld\n", times.SysStartTime); 
		

		sleep(1);                       
	} // end while
	 releasePublisher(&pubContainer[0]);
 	releasePublisher(&pubContainer[1]);
 	releasePublisher(&pubContainer[2]);
	close(socket_srv);
	std::cout << "Closing UDP socket at port " << 5500 << ". EXIT." << std::endl;
	return 0;
}
