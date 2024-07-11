#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fstream>
#include <dirent.h>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/Int64.h"

using namespace std;
using namespace cv;

int PORT = 9999;
static long long systime = 0;
//把四个char转为一个int
int four_char_to_int(unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{
    unsigned int ret_val = 0;
    ret_val = d;
    ret_val <<= 8;
    ret_val |= c;
    ret_val <<= 8;
    ret_val |= b;
    ret_val <<= 8;
    ret_val |= a;

    return ret_val;
}

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
static int socket_srv = -1;
static  struct sockaddr_in address_srv, from;


static	int start_sec = 0;
static	int setart_nsec = 0;
void chatterCallback(const std_msgs::Int64 & msg)
{
  //ROS_ERROR("I *********************************************************************************heard: [%lld]", msg.data);
  start_sec = msg.data/1000;
  setart_nsec = msg.data - (start_sec * 1000);
  setart_nsec *= 1000000;
  //ROS_ERROR("start_sec %d\n", start_sec);
  //ROS_ERROR("setart_msec %d\n",  setart_nsec);
  
}

//戴老师新的UDP传图协议（分片带有序号）说明(2022.1)
//每个UDP包=信息头（24字节）+图像数据（最大60000字节，一般除开最后一个分片，其他分片的图像数据都是60000字节）
//信息头依次是 校验，包长度，包序号，总包数，时间戳 ，前四个是int型，每个占四个字节，最后一个是double型，占8个字节。
//接收完后，首先判断序号是否完整，小于总包数的每个序号是否都有，第一个序号是从0开始。
//如果每个序号都有，然后看看每个序号对应的位置，根据序号对应的位置调整每个分片的图像数据，按照顺序组合成完整正确的图片数据。（实际看基本原本就是顺序的）
//得到按照正确顺序排序组合成的完整的图像数据后，也就是一个字符数组，然后通过imdecode转为opencv的Mat类型，再转为ROS话题发布出去。
//其实和之前相比主要是多了个序号，根据序号重新排序每个分片里的图像数据，怕出现一个图片的多个分片传过来的顺序乱了的情况。

//实测情况，通过UDP传输时 RGB 2个分片 灰度图 4个分片 深度图 3个分片，经过imdecode解码之后都是一张640*480的图片。

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_rec_node");
	
    ros::NodeHandle nh("~");
	
    image_transport::ImageTransport it(nh);
    string camera_id, port_str;
    string TYPE;
    string ENCODING;
    string FRAME_ID;
    string TOPIC_NAME;
    nh.param<std::string>("camera_id", camera_id, "left");
    nh.param<int>("port", PORT, 9999);
    nh.param<std::string>("type", TYPE, "rgb");
    nh.param<std::string>("encoding", ENCODING, "bgr8");
    nh.param<std::string>("frame_id", FRAME_ID, "camera_color_optical_frame");
    nh.param<std::string>("topic_name", TOPIC_NAME, "/camera/rgb");
    cout << "camera_id: " << camera_id << endl;
    cout << "port: " << PORT << endl;

    // image_transport::Publisher pub = it.advertise("/camera/" + camera_id, 1);
    image_transport::Publisher pub = it.advertise(TOPIC_NAME, 1);
	
	

    struct sockaddr_in addr;
    socklen_t sockfd, addr_len = sizeof(struct sockaddr_in);

    /* 建立socket，注意必须是SOCK_DGRAM */
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket failed!");
        exit(1);
    }

    /* 填写sockaddr_in 结构 */
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    /* 绑定socket */
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("connect failed!");
        exit(1);
    }

    //戴老师新的协议，头是五个数，4个int，一个double，依次是  校验，包长度，包序号，总包数，时间戳，之前的文件头是   校验，图片总字节数，包长度，总包数，时间戳

    int imgFlag = 1234567890;
    // int fhead_size = 4*sizeof(int);
    int fhead_size = 24;
    char data_total[900000]; //这个小了可能会出现段错误(核心已转储)的报错导致节点挂掉，所以这里由原先的600000改为900000
    // char data_total_1[900000]; //用来测试用的
    // char data_total_temp[10][70000];  //不用vector改为多维数组试试。
    // int data_total_temp_row_size[10]; //存储data_total_temp二维字符数组每行的长度
    // int data_total_temp_row = 0;      // data_total_temp的第几行
    char data_pack[62001]; //存单个UDP包的数据
    // char data_pack_picture[62001];  //存单个UDP包里的图像数据，也就是初开头信息的部分的数据。

    Mat image;

    int recPackNum = 0;
    int recvd_size = 0;

    vector<int> seqvector;
    vector<vector<char>> charvectorvector; //这种形式是可以的。
    vector<vector<char>> charvectorvector_reorder;  //存放根据序号重新排序后的分片中的图像数据

	ros::Rate loop_rate(50);
	char szRecvBuf[128] = {0};
	socklen_t len_caddr=sizeof(from); 
	
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("wintime", 1000, chatterCallback);
	



	
	while (ros::ok)
    {

	
        int recvpack = recvfrom(sockfd, data_pack, 62000, 0, (struct sockaddr *)&addr, &addr_len);

        // recvpack是实际接收到的字节数，不是等于62000，可以根据这个提取出哪些字节是真正接收到的字节。
        // int imgFlagrecv[16000];
        int imgFlagrecv;
        imgFlagrecv = four_char_to_int(data_pack[0], data_pack[1], data_pack[2], data_pack[3]);
        int imgPackUnit = four_char_to_int(data_pack[4], data_pack[5], data_pack[6], data_pack[7]);
        int packSeq = four_char_to_int(data_pack[8], data_pack[9], data_pack[10], data_pack[11]);
        int imgpackNum = four_char_to_int(data_pack[12], data_pack[13], data_pack[14], data_pack[15]);
        double timeStmp = eight_char_to_double(data_pack[16], data_pack[17], data_pack[18], data_pack[19], data_pack[20], data_pack[21], data_pack[22], data_pack[23]); //打印看了下，这样得到的时间戳整数部分是对的，是每秒加一。
        int imgtimestampsec = floor(timeStmp);
        int imgtimestampnsec = (timeStmp - imgtimestampsec) * 1000000000;
	
	
		imgtimestampsec += start_sec;
		imgtimestampnsec += setart_nsec;		
		

		
        double timeStmpStore; //存储第一个分片的时间戳
/*        
		printf("imgFlagrecv is %d\n", imgFlagrecv);
        printf("imgPackUnit is %d\n", imgPackUnit);
        printf("packSeq is %d\n", packSeq);
        printf("imgpackNum is %d\n", imgpackNum);
        printf("timeStmp is %f\n", timeStmp);
*/
        if (imgFlagrecv == imgFlag)
        {
            //printf("imgFlag is ok\n");
        }
        if (imgFlagrecv != imgFlag)
        {
            printf("wrong data");
            continue;
        }
        if (packSeq == 0) //如果是第一个包
        {
            seqvector.clear(); //清空容器中所有元素
            charvectorvector.clear();
            charvectorvector_reorder.clear(); //这个不要漏掉，不清零就会有问题。

            recvd_size = 0;
            recvd_size = recvd_size + recvpack - 24;

            // data_total_temp_row = 0;
            timeStmpStore = eight_char_to_double(data_pack[16], data_pack[17], data_pack[18], data_pack[19], data_pack[20], data_pack[21], data_pack[22], data_pack[23]);

            seqvector.push_back(packSeq);

            vector<char> charvector1(data_pack + 24, data_pack + recvpack); //这样可以把data_pack_picture这个中间商去掉了，进一步减少计算量。
            charvectorvector.push_back(charvector1);

            recPackNum = imgpackNum;
        }
        if (packSeq != 0) //如果不是包头，直接将其存入列表
        {
            if (recPackNum == 0)
            {
                continue;
            }

            if (abs(timeStmp - timeStmpStore) > 0.00001) //判断第二及之后的分片的时间戳和第一个分片的时间戳是否相差很小。
            {
                continue;
            }

            recvd_size = recvd_size + recvpack - 24;
            // data_total_temp_row = data_total_temp_row + 1;
            seqvector.push_back(packSeq);

            vector<char> charvector2(data_pack + 24, data_pack + recvpack);
            charvectorvector.push_back(charvector2);
        }

        if (seqvector.size() == recPackNum) //如果收到的包达到总数了，开始处理图像
        {
            recPackNum = 0;
            //printf("Start Img Cap\n");
            bool dataOk = true;
            // vector里面是可以搜索某个数出现的次数。
            int i_seq = 0;
            for (i_seq = 0; i_seq < seqvector.size(); i_seq++)
            {

                int num = 0;
                // num= count(seqvector.begin(),seqvector.end(),1);//统计出现的次数
                num = count(seqvector.begin(), seqvector.end(), i_seq); //统计出现的次数
                if (num < 1)
                {
                    dataOk = false;                       //如果某序号不在包中，报错
                    //printf("Failed to process img pack"); //实测的时候，这句话没有被打印，说明大部分接收的都是完整的。
                    break;
                }
                //按次序搜索包序号
                vector<int>::iterator it = find(seqvector.begin(), seqvector.end(), i_seq);
                vector<int>::iterator is = seqvector.begin();
                int index = it - is; //得到对应值的下标序号
                //printf("seq%d is in position %d\n", i_seq, index);

                charvectorvector_reorder.push_back(charvectorvector[index]); // charvectorvector_temp存放重新排序后的图像数据。我还是想最后再把charvectorvector_temp的值给charvectorvector，这样后面的代码不需要改,后来发现需要改动的很小，后面就改一处就可以了。
            }
            if (dataOk)
            {
                int i_charvectorvector = 0;
                int j_charvectorvector = 0;
                //下面是循环赋值的方式
                /*******************************************************************************************************************************************************
                int data_total_count = 0;
                for (i_charvectorvector = 0; i_charvectorvector < charvectorvector.size(); i_charvectorvector++) 
                {
                    for (j_charvectorvector = 0; j_charvectorvector < charvectorvector[i_charvectorvector].size(); j_charvectorvector++)
                    {
                        // data_total[data_total_count]=charvectorvector[i_charvectorvector][j_charvectorvector];
                        data_total[data_total_count] = charvectorvector_reorder[i_charvectorvector][j_charvectorvector];
                        data_total_count = data_total_count + 1; //刚刚发现通过vector+循环赋值的方式也是可以的，只是因为刚刚发现之前少写了一句代码。
                    }
                    printf("charvectorvector[%d].size() is %d\n", i_charvectorvector, charvectorvector[i_charvectorvector].size());
                }
                ********************************************************************************************************************************************************/
                // memcpy(data_total, &charvectorvector_reorder[0][0], charvectorvector_reorder[0].size() * sizeof(charvectorvector_reorder[0][0])+charvectorvector_reorder[1].size() * sizeof(charvectorvector_reorder[0][0]));//不能这么一次性赋值，一次性赋值会导致只有第一个分片赋值过去了，可能因为vecor<vector<char>>里面vector<char>和vector<char>之间的地址不是连续的，所以下面外加一层循环。
                int data_total_count_1 = 0;
                for (i_charvectorvector = 0; i_charvectorvector < charvectorvector.size(); i_charvectorvector++) 
                {
                   memcpy(data_total+data_total_count_1, &charvectorvector_reorder[i_charvectorvector][0], charvectorvector_reorder[i_charvectorvector].size() * sizeof(charvectorvector_reorder[0][0]));
                   data_total_count_1=data_total_count_1+charvectorvector_reorder[i_charvectorvector].size();
                   //printf("charvectorvector[%d].size() is %d\n", i_charvectorvector, charvectorvector[i_charvectorvector].size());
                }

               // printf("recvd_size is %d\n", recvd_size);
                // image = imdecode(Mat(1, recvd_size, CV_8UC1, data_total), IMREAD_COLOR);
                //int i_data_total;
                //for (i_data_total = 0; i_data_total < recvd_size; i_data_total++)
                //{
                    // data_total[i_data_total]={0x00};  //全部赋值为0，测试发布的图像话题。
                    // printf("%c",data_total[i_data_total]); //打印发现没有内容，怪不得没有东西？那应该是imdecode这一步出问题了。
                //}

                Mat white = cv::Mat(250, 250, CV_8UC3, Scalar(255, 255, 255)); //创建一张白图,测试用。
                // image = imdecode(Mat(1, 60000, CV_8UC1, data_total), IMREAD_COLOR);
                if (TYPE == "rgb")
                {
                    image = imdecode(Mat(1, recvd_size, CV_8UC1, data_total), IMREAD_COLOR);
                }
                if (TYPE == "gray")
                {
                    image = imdecode(Mat(1, recvd_size, CV_8UC1, data_total), IMREAD_GRAYSCALE);
                }
                if (TYPE == "depth")
                {
                    image = imdecode(Mat(1, recvd_size, CV_8UC1, data_total), IMREAD_ANYDEPTH);
                }

                // image = imdecode(white, IMREAD_COLOR);  //弄出一张白图，不需要imdecode这一步。
                //printf("image中元素的个数:%d\n", image.total());
                std_msgs::Header hd;
                hd.stamp.sec = imgtimestampsec;
                hd.stamp.nsec = imgtimestampnsec;
                // hd.frame_id = "camera_color_optical_frame";
                hd.frame_id = FRAME_ID;
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(hd, ENCODING, image).toImageMsg();
                pub.publish(msg);
			
                // imshow("image",image); //有报错，说未定义。
                // waitKey(1); //有报错，说未定义。
            }
        }
		ros::spinOnce();

    }

    close(sockfd);
    return 0;
}

