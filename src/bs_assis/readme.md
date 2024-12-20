# dds_temple

一个fast-dds开发的模版和例子

## 安装
官网: https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html

1. 下载install.sh脚本，[eProsima Fast DDS 2.11.2](https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-dds/eprosima-fast-dds-2-11-2)

2. 运行
    ```
    sudo ./install.sh
    ```

3. 在安装过程中可能会提示cmake版本过低，按照[安装、升级Cmake](https://note.youdao.com/s/9tKOApB4) 安装新版本cmake。

## DDS上手

DDS通信的也是订阅发布的机制，底层没怎么研究，大概的流程是先创建一个话题，然后通过局域网自动match这个话题，match上了进行推送和订阅。

### 消息定义

fast-dds的消息格式定义后缀是idl，消息的语法定义参见[官网的文档](https://fast-dds.docs.eprosima.com/en/latest/fastddsgen/dataTypes/dataTypes.html)

C++的原生类型和一些STL都有实现

![image-20220517190553559](https://s2.loli.net/2022/05/17/YlzD6VJoWGNOKx8.png)

并且可以在idl里面引用idl。比如定义下面一个这样的数据

```
//BBoxData.idl
struct BBoxData
{
    short id;

    string obj_class;
    int64 xmin;
    int64 ymin;
    int64 xmax;
    int64 ymax;

    float probability;

    // geometry_msgs/Point
    float pose_x;
    float pose_y;
    float pose_z;

};
```

然后定义一个`BBoxData`的列表可以这么弄

```
//BBoxesData.idl
#include "BBoxData.idl"

struct BBoxesData
{
    short system_ID;
	sequence<BBoxData> bounding_boxes;
};
```

### 消息生成

消息定义完成后就可以通过dds的命令行生成对应的cpp文件。运行下面的指令生成

```
fastddsgen dds_msg/*.idl -d dds_gen
```

一个idl文件大概会生成四个文件，比如`BBoxData.idl`会生成下面四个文件。

```
BBoxData.cxx
BBoxData.h
BBoxDataPubSubTypes.cxx
BBoxDataPubSubTypes.h
```

具体干啥的不用管，基本是生成的模版代码，有需要的时候具体再读就行。



## DDS和ROS1联动

有分布式的数据交互需要但是又没用上ROS2的话可以用ROS1+dds联动的方式。PC1上的ROS的话题推给dds，dds通过局域网推给PC2的dds，PC2的dds接收到数据之后再发给PC2的ROS。

![image-20220517192037471](https://s2.loli.net/2022/05/17/aRleT1cKUNMQPGI.png)

### DDS的Node编写

首先在`DDS_NodeBase.h`里面定义一些基类，后面会用的上，主要参考了[官网的demo](https://fast-dds.docs.eprosima.com/en/latest/fastdds/getting_started/simple_app/simple_app.html)

主要是定义了`DDS_Publisher`和`DDS_Subscriber`，从官方的`DataWriterListener`和`DataReaderListener`继承而来

然后针对定义的数据类型编写对应的Node，并在Node下实现**发送函数和接收函数**。

以BBoxes为例，接收函数如下，把从dds来的数据推送到对应的ros话题中

```
class BBoxesDataSubscriber : public DDS_Subscriber
{
    // 接收，数据可用的时候
    void on_data_available(DataReader* reader) override
    {
        SampleInfo info;
        if (reader->take_next_sample(&recvData, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (info.valid_data)
            {
                swarm_msgs::BoundingBoxes bboxesMsg;
                

                bboxesMsg.header.stamp = ros::Time::now();
                for (size_t i = 0; i < recvData.bounding_boxes().size(); i++)
                {
                    swarm_msgs::BoundingBox bboxMsg;
                    auto bbox_data = recvData.bounding_boxes()[i];

                    bboxMsg.id = bbox_data.id();
                    bboxMsg.Class = bbox_data.obj_class();

                    bboxMsg.xmin = bbox_data.xmin();
                    bboxMsg.ymin = bbox_data.ymin();
                    bboxMsg.xmax = bbox_data.xmax();
                    bboxMsg.ymax = bbox_data.ymax();
                    bboxMsg.probability = bbox_data.probability();

                    bboxMsg.point.x = bbox_data.pose_x();
                    bboxMsg.point.y = bbox_data.pose_y();
                    bboxMsg.point.z = bbox_data.pose_z();

                    bboxesMsg.bounding_boxes.push_back(bboxMsg);
                }
                bboxesPubs[recvData.system_ID()].publish(bboxesMsg);
            }
        }
    }

    BBoxesData  recvData;
};
```

发送函数如下：

```
bool BBoxesDataNode::publish(BBoxesData* sendData)
{
    if (dwListener_.matched_ > 0)
    {
        writer_->write(sendData);
        return true;
    }
    return false;
}
```

要点主要是准备好打包的数据。

其他的细节看代码就行。

代码仓库：https://github.com/BrightSoulXYHY/dds_temple
