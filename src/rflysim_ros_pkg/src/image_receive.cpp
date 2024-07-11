#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
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

using namespace std;
using namespace cv;

int PORT=9999;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rec_node");
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
    //image_transport::Publisher pub = it.advertise("/camera/"+camera_id, 1);
    image_transport::Publisher pub = it.advertise(TOPIC_NAME, 1);

    struct sockaddr_in addr;
    socklen_t sockfd, addr_len = sizeof(struct sockaddr_in);

    /* 建立socket，注意必须是SOCK_DGRAM */
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror ("socket failed!");
        exit(1);
    }

    /* 填写sockaddr_in 结构 */
    bzero(&addr, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY) ;
 
    /* 绑定socket */
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr))<0)
    {
        perror("connect failed!");
        exit(1);
    }
 
    int imgFlag = 1234567890;
    //int fhead_size = 4*sizeof(int);
    int fhead_size = 24;
    //int img_head[4];
    int img_head[6];
    char data_total[900000];  //这个小了可能会出现段错误(核心已转储)的报错导致节点挂掉，所以这里由原先的600000改为900000
    Mat image;

    while (ros::ok) {
        int recvbytes = recvfrom(sockfd, (char*)img_head, fhead_size, 0, (struct sockaddr *)&addr, &addr_len);
        if (recvbytes != fhead_size || img_head[0] != imgFlag) {
            continue;
        }
        // cout << img_head[0] << "," << img_head[1] << "," << img_head[2] << "," << img_head[3] << endl << endl; 
        int imgLen = img_head[1];
        int imgPackUnit = img_head[2];
        int imgpackNum = img_head[3];
        int recvd_size = 0;

        for (int i=0; i<imgpackNum; i++) {
            int recvpack = recvfrom(sockfd, data_total+i*imgPackUnit, imgPackUnit, 0, (struct sockaddr *)&addr, &addr_len);
            recvd_size += recvpack;
        }

        if (recvd_size == imgLen) {
            // cout << "received image." << endl;
            //image = imdecode(Mat(1, recvd_size, CV_8UC1, data_total), IMREAD_COLOR);
            if(TYPE=="rgb")
            {
                image = imdecode(Mat(1, recvd_size, CV_8UC1, data_total), IMREAD_COLOR); 
            }
            if(TYPE=="gray")
            {
                image = imdecode(Mat(1, recvd_size, CV_8UC1, data_total), IMREAD_GRAYSCALE); 
            }
            if(TYPE=="depth")
            {
                image = imdecode(Mat(1, recvd_size, CV_8UC1, data_total), IMREAD_ANYDEPTH); 
            }
            //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), ENCODING, image).toImageMsg();
            pub.publish(msg);
            // imshow("image", image);
            // if (char(waitKey(1))=='q') { break; }
        }
    }

    close(sockfd);
    return 0;
}
