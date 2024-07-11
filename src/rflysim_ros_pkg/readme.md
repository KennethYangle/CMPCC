## rflysim_ros_pkg功能包

可以发出RGB图像话题，灰度图像话题，深度图像话题及imu话题

### 代码文件说明

#### image_receive.cpp

基于原UDP图像传输协议的图像接收话题发布cpp节点，RGB图像，灰度图像，深度图像都是用这个image_receive.cpp。

#### image_receive.py

基于原UDP图像传输协议的图像接收话题发布python节点，RGB图像，灰度图像，深度图像都是用这个image_receive.py。

#### image_receive_newprotocol.cpp

基于新的UDP图像传输协议(2022.1图像分片有序号)的图像接收话题发布cpp节点，RGB图像，灰度图像，深度图像都是用这个image_receive_newprotocol.cpp。

#### image_receive_newprotocol.py

基于新的UDP图像传输协议(2022.1图像分片有序号)的图像接收话题发布python节点，RGB图像，灰度图像，深度图像都是用这个image_receive_newprotocol.py。

#### imu_receive.py
imu数据接收和IMU话题发布python节点。  

### launch文件说明

rgb_cpp.launch :启动基于原UDP图像传输协议的RGB图像接收话题发布cpp节点  
rgb_py.launch :启动基于原UDP图像传输协议的RGB图像接收话题发布python节点  
rgb_newprotocol_cpp.launch :启动基于新的UDP图像传输协议(2022.1图像分片有序号)的RGB图像接收话题发布cpp节点  
rgb_newprotocol_py.launch :启动基于新的UDP图像传输协议(2022.1图像分片有序号)的RGB图像接收话题发布python节点  
gray_cpp.launch :启动基于原UDP图像传输协议的灰度图像接收话题发布cpp节点  
gray_py.launch :启动基于原UDP图像传输协议的灰度图像接收话题发布python节点  
gray_newprotocol_cpp.launch :启动基于新的UDP图像传输协议(2022.1图像分片有序号)的灰度图像接收话题发布cpp节点  
gray_newprotocol_py.launch :启动基于新的UDP图像传输协议(2022.1图像分片有序号)的灰度图像接收话题发布python节点  
gray_stereo_cpp.launch :启动基于原UDP图像传输协议的双目灰度图像接收话题发布cpp节点  
gray_stereo_py.launch :启动基于原UDP图像传输协议的双目灰度图像接收话题发布python节点  
gray_stereo_newprotocol_cpp.launch :启动基于新的UDP图像传输协议(2022.1图像分片有序号)的双目灰度图像接收话题发布cpp节点  
gray_stereo_newprotocol_py.launch :启动基于新的UDP图像传输协议(2022.1图像分片有序号)的双目灰度图像接收话题发布python节点  
depth_cpp.launch :启动基于原UDP图像传输协议的深度图像接收话题发布cpp节点  
depth_py.launch :启动基于原UDP图像传输协议的深度图像接收话题发布python节点  
depth_newprotocol_cpp.launch :启动基于新的UDP图像传输协议(2022.1图像分片有序号)的深度图像接收话题发布cpp节点  
depth_newprotocol_py.launch :启动基于新的UDP图像传输协议(2022.1图像分片有序号)的深度图像接收话题发布python节点  
imu_py.launch :启动imu数据接收及IMU话题发布python节点  
rgb_gray_depth_imu_py.launch :启动基于原UDP图像传输协议的RGB，灰度，深度图像接收话题发布python节点和imu数据接收话题发布python节点  


### 功能包使用说明

注意：板载ubuntu端和windows端都用网线连接，也就是千兆网线（蓝色网线），不要用黄色网线，可能会导致发布话题频率很低。

建立工作空间droneyee_ws，并将功能包放到droneyee/src/目录下，在工作空间根目录下打开终端运行catkin_make进行编译。

在相应launch文件里面修改端口和IP和话题名称，IP为接收端所在主机的局域网IP地址，端口需要和发送端确认对应好。再次强调，启动launch文件前确认好IP和端口，否则节点启动会失败或者挂掉。端口不建议使用的1000或者1001这种，因为这种端口在ubuntu下需要权限。


对功能包src文件夹下的每个python文件都进行赋予权限操作。

```
chmod +x image_receive.py
chmod +x image_receive_newprotocol.py
```

对于NX，可以先运行script文件夹里面的两个sh脚本max_cpu_freq.sh和max_gpu_freq.sh把NX的CPU频率和GPU频率调到最大，运行命令如下:
```
sudo sh ~/max_cpu_freq.sh
sudo sh ~/max_gpu_freq.sh
sudo jeston_clocks
```

打开一个终端

```
source ~/droneyee_ws/devel/setup.bash
```
运行这个launch文件可以同时发出RGB图像话题，灰度图像话题，深度图像话题和imu话题。
```
roslaunch rflysim_ros_pkg rgb_gray_depth_imu_py.launch
```

可以用rqt_image_view查看图像话题。

图像UDP数据实际发送30hz,图像话题频率一般可以达到25hz左右，但是具体可能会受实际计算机板载内存等因素影响。

想通过python节点单独发布rgb图像话题可以运行下面命令

```
roslaunch rflysim_ros_pkg rgb_py.launch
```

想通过python节点单独发布灰度图像话题可以运行下面命令

```
roslaunch rflysim_ros_pkg gray_mono_py.launch
```

想通过python节点单独发布深度图像话题可以运行下面命令

```
roslaunch rflysim_ros_pkg depth_py.launch
```



想单独发布imu话题可以运行下面命令，imu话题默认频率200hz

```
roslaunch rflysim_ros_pkg imu_py.launch
```

双目灰度图可以运行下面命令，需要在Config.json已经改好的情况下，注意launch文件里面的端口和Config.json里面的端口对应。
```
roslaunch rflysim_ros_pkg gray_stereo_py.launch
```

对于戴老师新的没有文件头，每个图像分片有序号的新的UDP传图协议，用python节点接收并发布图像话题，用以下命令  
通过python节点单独发RGB图像话题  
```
roslaunch rflysim_ros_pkg rgb_newprotocol_py.launch
```
通过python节点单独发深度图像话题  
```
roslaunch rflysim_ros_pkg depth_newprotocol_py.launch
```
通过python节点单独发灰度图像话题  
```
roslaunch rflysim_ros_pkg gray_mono_newprotocol_py.launch
```

通过python节点单独发双目灰度图像话题  
```
roslaunch rflysim_ros_pkg gray_stereo_newprotocol_py.launch
```
通过C++节点单独发布rgb图像话题可以运行下面命令  

```
roslaunch rflysim_ros_pkg rgb_newprotocol_cpp.launch 
```
通过C++节点单独发布灰度图像话题可以运行下面命令  

```
roslaunch rflysim_ros_pkg gray_newprotocol_cpp.launch 
```
通过C++节点单独发布深度图像话题可以运行下面命令  

```
roslaunch rflysim_ros_pkg depth_newprotocol_cpp.launch 
```

发双目灰度图时的Config.json ，可如下面所示，就是把第一个结构体里面的TypeID由1改为3，就是由rgb摄像头改为灰度摄像头（不唯一，这里只是作为参考）
```
{
    "VisionSensors":[
        {
            "SeqID":0,
            "TypeID":3,
            "TargetCopter":1,
            "TargetMountType":0,
            "DataWidth":640,
            "DataHeight":480,
            "DataCheckFreq":200,
            "SendProtocol":[0,127,0,0,1,9999,0,0],
            "CameraFOV":90,
            "SensorPosXYZ":[0.3,0,0],
            "SensorAngEular":[0,0,0],
            "otherParams":[0,0,0,0,0,0,0,0]
        },
        {
            "SeqID":1,
            "TypeID":2,
            "TargetCopter":1,
            "TargetMountType":0,
            "DataWidth":640,
            "DataHeight":480,
            "DataCheckFreq":200,
            "SendProtocol":[0,127,0,0,1,1000,0,0],
            "CameraFOV":90,
            "SensorPosXYZ":[0.3,0,0],
            "SensorAngEular":[0,0,0],
            "otherParams":[0,0,0,0,0,0,0,0]
        },
        {
            "SeqID":2,
            "TypeID":3,
            "TargetCopter":1,
            "TargetMountType":0,
            "DataWidth":640,
            "DataHeight":480,
            "DataCheckFreq":200,
            "SendProtocol":[0,127,0,0,1,1001,0,0],
            "CameraFOV":90,
            "SensorPosXYZ":[0.3,0,0],
            "SensorAngEular":[0,0,0],
            "otherParams":[0,0,0,0,0,0,0,0]
        }
    ]
}

```


## 传输协议说明

### 图像UDP传输协议(2021.12)

图像传输先传一次文件头，然后循环发送图像分片。目前是一张图片就发一次文件头。

文件头内容如下  
imgFlag = 1234567890         //图像标志位，32位无符号整型。  
imgLen = len(data)           //图片字节长度，32位无符号整型。  
imgPackUnit = 60000          //图片分片长度，即单次传送长度，32位无符号整型。  
imgpackNum = imgLen//imgPackUnit+1     //图片分片次数，32位无符号整型。  
timeStmp                     //时间戳，double，64位浮点型。  

#### 新的图像UDP传输协议(2022.1)

每个UDP包=信息头（24字节）+图像数据（最大60000字节，一般除开最后一个分片，其他分片的图像数据都是60000字节）  

信息头依次是 校验，包长度，包序号，总包数，时间戳 ，前四个是int型，每个占四个字节，最后一个是double型，占8个字节。  

接收完后，首先判断序号是否完整，小于总包数的每个序号是否都有，,第一个序号是从0开始。  

如果每个序号都有，然后看看每个序号对应的位置，根据序号对应的位置调整每个分片的图像数据，按照顺序组合成完整正确的图片数据。（实际看基本原本就是顺序的）  

得到按照正确顺序排序组合成的完整的图像数据后，也就是一个字符数组，然后通过imdecode转为opencv的Mat类型，再转为ROS话题发布出去。  

其实和之前相比主要是多了个序号，根据序号重新排序每个分片里的图像数据，怕出现一个图片的多个分片传过来的顺序乱了的情况。但是个人感觉传输的时候分片顺序还好，这样做反而可能为了那比较小的概率增加了每次的计算消耗。  

### IMU数据UDP传输协议

坐标系参考的D435i的加速度计和陀螺仪的坐标系。

目前imu数据传输协议参考下面imu数据结构体

struct imuDataCopter{  
    int checksum; //数据校验位1234567898  
    int seq; //消息序号  
    double timestmp;//时间戳  
    float acc[3];  
    float rate[3];  
}   2i1d6f  


还有注意协议里的 角速度顺时针方向为正，此顺时针方向为朝向对应轴正方向的顺时针方向，如对于绕x轴转的角速度，顺时针方向是朝向x轴正方向的顺时针方向，顺时针方向的角速度为正，逆时针方向的角速度为负。

在描述加速度正方向时，是以相机镜头朝前，相机固定孔位朝下平放在桌面上，人的正面和相机镜头正面朝同一方向作为标准，来描述的正右方，正下方，正前方。



下面是将D435i正放静止时打印的D435i的/camera/imu话题内容，可作参考

![输入图片说明](https://images.gitee.com/uploads/images/2021/1201/111421_788fad46_8668676.png "屏幕截图.png")
