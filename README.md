# ROS
记录一下转行找工作的学习历程。
# ROS1
## ros1通信模型 

 *  talker注册
 *  listener注册
 *  ROS Master进行信息匹配
 *  listener 发送链接请求
 *  talker确认请求
 *  建立连接
 *  talker给listener发数据

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/886915d91fb1002c9cb8269b58955019.png)

## ROS的应用框架是怎样的？ 

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/61c77a10d78e627c62f72ee638615e25.png)  
总结：ROS就是一个为了提高机器人开发的软件复用率，开发的一款具有独特的通信机制、丰富的开发工具、海量的应用功能、良好的生态系统集的工具。

## 如何新建一个工作空间？ 

```java
mkdir  -p ~/target_ws/src   //其中target_ws为我新建的工作空间名
cd ~/target_ws/src
catkin_init_workspace    //初始化工作空间
cd ~/target_ws
catkin_make  //编译，编译完成后，会发现catkin_ws中多了两个文件 build 和 devel
echo "source ~/target_ws/devel/setup.bash" >> ~/.bashrc  
source ~/.bashrc
```

需要注意一点，同一个工作空间下不能存在同名功能包，不同的工作空间下可以存在同名功能包。但是要注意一个问题，ROS运行时会优先选择最前端工作空间的同名功能包。为了避免出现想不到的问题，所有工作空间尽量不使用同名功能包。

功能包的名称尽量按照a\_b\_c的格式书写，否则，编译过程将会出现警告。

## 如何新建一个功能包？ 

```java
cd ~/target_ws/src/
catkin_create_pkg target_pub std_msgs roscpp rospy   //target_pub为功能包的名称
cd ~/target_ws-
catkin_make   //编译功能包
```

## 如何新建一个ROS节点？ 

接着上面的测试功能包，我们在功能包的src文件下新建test.cpp文件：

```java
cd ~/catkin_test_ws/src/test_package/src/
vim test.cpp
```

然后就是ROS节点的模板程序，都是在模板程序之上做的修改。

```java
#include "ros/ros.h"                    //引入ROS头文件
 
int main(int argc,char **argv)
{
	ros::init(argc,argv,"test");        //初始化ROS节点
	ros::NodeHandle nh;                 //创建ROS句柄
	ros::Rate loop_rate(10);            //定义循环频率 10HZ
	while(ros::ok())
	{
    	std::cout<< "hello ros" <<std::endl;
		loop_rate.sleep();             //按照循环速率延时
	}
	return 0;
}
```

到这里，然后编译就可以了？这个功能包的编译还需要修改对应的CMakeLIsts.txt文件，一般为三步。

 *  设置需要编译的代码和生成的可执行文件；
 *  设置链接库；
 *  设置依赖；

所以我们修改功能包下的CMakeLIsts.txt文件，一般功能包会自动生成CMakeLists.txt文件内容，我们进行修改即可。

```java
add_executable(${PROJECT_NAME}_node src/test.cpp)                             #这里注意自动生成的源文件名字一般不对，一定要修改成对的名字
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

这里因为我们没有自定义的消息或者服务类型，所以其他的地方不做更改，如果使用了自定义的消息或者服务类型，注意修改其他的地方。

```java
cd ~/catkin_test_ws                                            
catkin_make           #编译
```

此时`~/catkin_test_ws/devel/lib/test_package`目录下生成了`test_package_node`的可执行文件  
执行功能包，在使用rosrun执行功能包之前，使用roscore启动rosmaster。启动`rosmaster`是启动一切ros节点的前提。

```java
roscore            #启动rosmaster
```

打开一个新终端启动，test\_package功能包下的test\_package\_node节点。

```java
rosrun test_package test_package_node
```

终端循环打印hello ros，一个新的ros节点就创建完成了。


## 生成自定义消息的步骤
### 创建时依赖`message_generation`和`message_runtime`
### 新建msg消息文件
### 编译配置 `add_message_files()`
### 编译配置中 将依赖的其他包名加入`geneate_messages()`
### 将 `message_runtime`加入`catkin_package()`的`CATKIN_DEPENDS`
### 将`message_generation`和`message_runtime`加入`package.xml`的`<build_depend>`和`<exec_depend>`标签内
### 编译

## common_msgs

- actionlib_msgs
  - **GoalID**  
    用于标识目标的唯一 ID，常用于管理目标的执行和取消。
  - **GoalStatus**  
    表示目标的状态，如“正在处理”、“已达成”等。
  - **GoalStatusArray**  
    包含多个 GoalStatus 信息的数组，用于追踪多个目标的状态。

- diagnostic_msgs
  - **DiagnosticArray**  
    包含诊断状态的数组，通常用于系统健康状况报告。
  - **DiagnosticStatus**  
    单个诊断状态，包含状态等级、名称和详细信息。
  - **KeyValue**  
    键值对，用于 DiagnosticStatus 中的附加信息。
  - **AddDiagnostics**  
    用于添加新的诊断源。
  - **SelfTest**  
    触发自测并报告测试结果。

- geometry_msgs
  - 加速度相关
    - **Accel**  
      包含加速度的向量消息。
    - **AccelStamped**  
      包含时间戳的加速度消息。
    - **AccelWithCovariance**  
      加速度信息附带协方差数据，用于表示测量不确定性。
    - **AccelWithCovarianceStamped**  
      包含时间戳和协方差的加速度消息。
  - 惯性相关
    - **Inertia**  
      表示物体的惯性张量及其质量中心。
    - **InertiaStamped**  
      包含时间戳的惯性消息。
  - 点与位置相关
    - **Point**  
      简单的三维点表示，包含 x, y, z 坐标。
    - **Point32**  
      以 32 位浮点数表示的三维点。
    - **PointStamped**  
      包含时间戳的三维点消息。
  - 多边形相关
    - **Polygon**  
      表示一个多边形，包含多个 Point32 点的数组。
    - **PolygonStamped**  
      带时间戳的多边形消息。
  - 姿态相关
    - **Pose**  
      表示物体的位置和方向。
    - **Pose2D**  
      2D 平面上的位置和方向。
    - **PoseArray**  
      包含多个 Pose 的数组，用于存储多个物体的位置和方向。
    - **PoseStamped**  
      带时间戳的姿态消息。
    - **PoseWithCovariance**  
      姿态信息附带协方差数据，用于表示测量不确定性。
    - **PoseWithCovarianceStamped**  
      包含时间戳和协方差的姿态消息。
  - 四元数相关
    - **Quaternion**  
      四元数，用于表示物体的旋转。
    - **QuaternionStamped**  
      带时间戳的四元数消息。
  - 变换相关
    - **Transform**  
      表示物体的位置和方向变化（平移和旋转）。
    - **TransformStamped**  
      带时间戳的变换消息，通常用于坐标变换。
  - 速度相关
    - **Twist**  
      包含线速度和角速度的信息。
    - **TwistStamped**  
      带时间戳的速度消息。
    - **TwistWithCovarianceStamped**  
      带时间戳和协方差的速度消息。
  - 向量相关
    - **Vector3**  
      三维向量，包含 x, y, z 值。
    - **Vector3Stamped**  
      带时间戳的三维向量消息。
  - 力矩相关
    - **Wrench**  
      包含力和力矩的消息。
    - **WrenchStamped**  
      带时间戳的力矩消息。

- nav_msgs
  - **GridCells**  
    包含一个表示栅格单元的点数组，常用于表示路径或占据的区域。
  - **MapMetaData**  
    地图的元数据，包括地图的分辨率、宽度、高度和起始位置等信息。
  - **OccupancyGrid**  
    占据栅格地图，用二维数组表示栅格的占用状态，适用于导航和路径规划。
  - **Odometry**  
    里程计消息，包含位置、方向和速度等信息，用于描述移动机器人在空间中的状态。
  - **Path**  
    路径消息，包含多个位姿点的数组，表示规划的路径。

- nav_msgs Services
  - **GetMap**  
    服务类型，用于请求当前的栅格地图。返回一个 `OccupancyGrid` 消息，用于表示整个地图。
  - **SetMap**  
    服务类型，用于设置或更新当前的栅格地图。传入一个包含 `OccupancyGrid` 数据的请求消息，以及可选的初始位姿。

- nav_msgs Actions
  - **GetMapAction**  
    Action 类型，用于请求并接收地图数据。包括三个部分：
    - **Goal**：触发地图获取的目标。
    - **Result**：包含 `OccupancyGrid` 数据的结果，用于返回地图。
    - **Feedback**：在地图生成时提供反馈信息，用于实时状态更新。

- sensor_msgs Messages
  - **BatteryState**  
    表示电池状态的消息，包含电压、电流、电量、剩余时间等信息。
  - **CameraInfo**  
    包含摄像头的内参和外参信息，用于图像处理中的坐标校正和转换。
  - **ChannelFloat32**  
    单个通道的浮点数数组，常用于多传感器数据的同步和传输。
  - **CompressedImage**  
    压缩格式的图像数据，适用于传输效率要求较高的场景。
  - **FluidPressure**  
    表示流体压力的消息，常用于水下机器人或气压传感器数据。
  - **Illuminance**  
    表示光照度的消息，用于测量环境的光强。
  - **Image**  
    未压缩的图像数据，包含图像宽度、高度、编码等信息。
  - **Imu**  
    惯性测量单元数据，包含加速度、角速度和方向的三维信息。
  - **JointState**  
    表示机器人关节状态的消息，包含位置、速度和力矩数据。
  - **Joy**  
    游戏控制器（摇杆）输入数据，用于机器人远程控制。
  - **LaserScan**  
    激光雷达扫描数据，包含距离信息和角度分布。
  - **MagneticField**  
    表示磁场强度的消息，用于磁力计或方向感应。
  - **NavSatFix**  
    全球导航卫星系统（GNSS）数据，包含经度、纬度和海拔信息。
  - **PointCloud**  
    三维点云数据，用于描述空间中的点集合。
  - **PointCloud2**  
    更灵活的点云数据格式，适合存储较大或自定义格式的点云数据。
  - **Range**  
    表示测距传感器数据，包含最小和最大范围、读取距离等信息。
  - **RegionOfInterest**  
    图像中的感兴趣区域，常用于图像处理中的目标检测。
  - **RelativeHumidity**  
    相对湿度数据，适用于环境监测。
  - **Temperature**  
    温度数据，常用于监测环境或设备温度。
  - **TimeReference**  
    时间参考数据，用于同步不同传感器的时间戳。

- sensor_msgs Services
  - **SetCameraInfo**  
    服务类型，用于设置摄像头的校准信息，包括内参和外参。常用于摄像头标定。

- shape_msgs    形状消息包
- stereo_msgs   双目视觉消息包
- trajectory_msgs   运动轨迹消息包
- visualization_msgs   图形显示消息包


## 如何在一个ros节点文件中调用其他的源文件？ 

这个问题很常见，但是他不应该是个问题，但是发现一些小伙伴还是不太会操作。我们在开发的过程中，随着程序的不断扩大，模块化编程肯定必不可少。有时，我们要调用别人写好的特定功能的源文件，这个时候我们应该怎么做吗？

首先，将你需要的源文件复制到你需要编写的功能包的src文件夹下，规范地讲，.hpp文件应该放到include文件下，如果文件比较少，也没有必要，和.cpp文件放在一起也可以。

ros节点源文件只需要添加你复制的头文件下就行了，函数自己调用就行了。

CMakeLists.txt文件需要做相应的修改，添加你添加的文件名字，放在ros节点文件后面，如下所示：

```java
add_executable(${
     PROJECT_NAME}_node src/test.cpp src/test2.cpp src/test3.cpp)  
#这里注意自动生成的源文件名字一般不对，一定要修改成对的名字
```

## 如何设置两台机器的分布式通信？ 

第一步：`ifconfig`分别查看主机和从机的ip  
例如  
主机ip：192.168.31.73  
从机ip：192.168.31.35

第二步：修改`/etc/hosts`文件，加入主从机ip  
在主机和从机的hosts文件中加入均加入主机和从机的ip和host名，例如

```java
192.168.31.73   localhostA
192.168.31.35   localhostB
```

然后ping一下保证能ping通就行。

第三步：修改`~/.bashrc` 文件,增加`ROS_MASTER_URI`  
在主机的~/.bashrc 文件末尾增加两行

```java
第一行, 主机的ip,第二行,主机的URI
export ROS_HOSTNAME=192 .168.31.73
export RoS_MASTER_URI=http: //192.168.31.73:11311
```

在从机的~/.bashrc 文件末尾增加两行

```java
第一行, 从机的ip,第二行,主机的RUI
export RoS_HOSTNAME=1192.168.31.35
export RoS_MASTER_URI=http://192.168.31.73:11311
```

然后对主机和从机分别更新一下环境

```java
source ~/.bashrc
```


## ROS的订阅和发布机制了解吗？实现的原理是什么 

ROS底层的通信是通过HTTP完成的，因此ROS内核本质上是一个HTTP服务器，它的地址一般是http://localhost:11311/。  
发布者与订阅者通过互相发送header，建立TCPROS/UDPROS连接。




## ROS延时是一个大问题，图像和imu的延时是怎么解决的？ 

参考 23 ROS中订阅（Subscribe）最新消息以及消息队列相关问题


## 这些项目里用到了哪些算法的哪些函数？ 

```java
1、opencv的
2、矩阵的变换
3、姿态，四元数和欧拉角的转换
3、协方差矩阵、伯努利曲线方程、无人机反馈控制器、位姿转换、误差计算等
```


## imu出来的是什么数据？ 

三轴加速度和角速度；分别用来测量物体在三维空间中的角速度和加速度，
imu系统输出：Odometry消息：位置、姿态、线速度、角速度和每个量的协方差矩阵


## imu的当前的姿态，和落点之间的位置；他们两个的相对位置是怎么算出来的？ 

位姿计算过程：  
`0时刻位姿初始化`：一般设置0时刻为里程计起点，即位姿速度全为0  
`重力初始化`：只有IMU一个传感器，所以直接用了第一帧数据（假设当前载体处于静止状态）的加速度作为重力加速度项  
`求解位姿`：初始化完成后，先求解位姿，因为求解位置的时候需要使用位姿结果将IMU坐标系下的加速度转化到全局坐标系下的加速度，用旋转矩阵表示的方法求解的位姿

## ROS中的spin()和spinOnce()相关问题 

spin()和spinOnce()叫ROS消息回调处理函数；相关消息的订阅函数，接受订阅的消息并不是立刻就被处理，而是必须要等到ros::spin()或ros::spinOnce()执行的时候才被调用，这就是消息回到函数的原理.

ROS默认有维护一个全局回调队列（名为：Global Callback Queue），将已可用的回调插入Callback队列中。再通过Spinner线程获取并执行当前可用的回调。

区别：  
`ros::spin()`调用后不会再返回，也就是你的主程序到这儿就不往下执行了.  
`ros::spinOnce()`在调用后还可以继续执行之后的程序

详细解释：  
`消息发布器(ros::Publisher)`在一个while循环里一直循环发布消息（“hello world”）到话题（“chatter”）上。  
`消息订阅器(ros::Subscriber)`一直监视话题，一旦知道话题上有数据，就会将该话题上的数据（message）作为参数传入到对应的回调函数（callback）中，但是这时候回调函数还没有被执行，而是把callback函数放到了回调函数队列中。所以当消息发布器不断发送message到该话题上时，就会有相应的callback函数存入队列中，它们函数名一样，只是实参内容不一样。（其实，是将传入的message存入到队列中，这里说是将callback函数存入到队列中，应该是为了便于理解）。

那什么时候会执行callback函数呢？那就是`ros::spin()`和`ros::spinOnce()`的事情了。

当`spinOnce()`函数被调用时，spinOnce()会调用回调函数队列中`第一个callback`函数，此时callback函数才被执行，然后等到`下次spinOnce`函数又被调用时，回调函数队列中`第二个callback`函数就会被调用，以此类推。

所以，这会有一个问题。因为回调函数队列的长度是有限的，如果发布器发送数据的速度太快，spinOnce函数调用的频率太少，就会导致队列溢出，一些callback函数就会被挤掉，导致没被执行到。

而对于`spin`函数，一旦进入`spin`函数，它就不会返回了，相当于它在自己的函数里面死循环了。只要回调函数队列里面有callback函数在，它就会马上去执行callback函数。如果没有的话，它就会阻塞，不会占用CPU。

## spin()和spinOnce()函数意义 

首先要知道，这俩兄弟学名叫`ROS消息回调处理函数`。它俩通常会出现在ROS的主循环中，程序需要不断调用`ros::spin()` 或`ros::spinOnce()`，两者区别在于前者调用后不会再返回，也就是你的主程序到这儿就不往下执行了，而后者在调用后还可以继续执行之后的程序。

如果你的程序写了相关的消息订阅函数，那么程序在执行过程中，除了主程序以外，ROS还会自动在后台按照你规定的格式，接受订阅的消息，但是所接到的消息并不是立刻就被处理，而是必须要`等到ros::spin()或ros::spinOnce()执行的时候才被调用`，这就是消息回调函数的原理。

## spin()和spinOnce()的区别 

就像上面说的，`ros::spin()` 在调用后不会再返回，也就是你的主程序到这儿就不往下执行了，而 `ros::spinOnce()` 后者在调用后还可以继续执行之后的程序。

其实看函数名也能理解个差不多，一个是一直调用；另一个是只调用一次，如果还想再调用，就需要加上循环了。

这里一定要记住，

 *  ros::spin()函数一般不会出现在循环中，因为程序执行到spin()后就不调用其他语句了，也就是说该循环没有任何意义。
 *  spin()函数后面一定不能有其他语句(return 0 除外)，有也是白搭，不会执行的。
 *  ros::spinOnce()的用法相对来说很灵活，但往往需要考虑调用消息的时机，调用频率，以及消息池的大小，这些都要根据现实情况协调好，不然会造成数据丢包或者延迟的错误。

## spin()和spinOnce()常见使用方法 

如果你的程序写了相关的消息订阅函数，那千万千万千万不要忘了在相应位置加上`ros::spin()`或者`ros::spinOnce()`函数，不然你是永远都得不到另一边发出的数据或消息的。

1、ros::spin()  
ros::spin()函数用起来比较简单，一般都在主程序的最后，加入该语句就可。例子如下：

```java
// 发送端
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
 
        /**
         * 向 Topic: chatter 发送消息, 发送频率为10Hz（1秒发10次）；消息池最大容量1000。
         */
        chatter_pub.publish(msg);
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
```

接收端代码中用到spin()函数：

```java
// 接收端
#include "ros/ros.h"
#include "std_msgs/String.h"
 
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{     
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
 
    /**
     * ros::spin() 将会进入循环， 一直调用回调函数chatterCallback(),每次调用1000个数据。
     * 当用户输入Ctrl+C或者ROS主进程关闭时退出，
     */
    ros::spin();
    return 0;
}
```

2、ros::spinOnce()

对于ros::spinOnce()的使用，虽说比ros::spin()更自由，可以出现在程序的各个部位，但是需要注意的因素也更多。比如：

1.  对于有些传输特别快的消息，尤其需要注意合理控制消息池大小和ros::spinOnce()执行频率; 比如消息送达频率为10Hz, ros::spinOnce()的调用频率为5Hz，那么消息池的大小就一定要大于2，才能保证数据不丢失，无延迟。

```java
// 接收端
 
#include "ros/ros.h"
#include "std_msgs/String.h"
  
void chatterCallback(const std_msgs::String::ConstPtr& msg){}
  
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 2, chatterCallback);
  
    ros::Rate loop_rate(5);
    while (ros::ok())
    { 
        /*...TODO...*/ 
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
```

1.  ros::spinOnce()用法很灵活，也很广泛，具体情况需要具体分析。但是对于用户自定义的周期性的函数，最好和ros::spinOnce并列执行，不太建议放在回调函数中；

```java
/*...TODO...*/
ros::Rate loop_rate(100);
  
while (ros::ok())
{    
    /*...TODO...*/
    user_handle_events_timeout(...);
    ros::spinOnce();                 
    loop_rate.sleep();
}
```

## ROS中订阅（Subscribe）最新消息以及消息队列相关问题 

详细参考：[https://blog.csdn.net/qq\_32618327/article/details/121650164][https_blog.csdn.net_qq_32618327_article_details_121650164]

机器人应用中难免会遇到运算起来很费时间的操作，比如图像的特征提取、点云的匹配等等。有时候，不可避免地，我们需要在ROS的Subscriber的Callback回调函数中进行这些费时的操作。Subscriber所订阅的消息的发布频率可能是很高的，而这些操作的运算速度肯定达不到消息发布的速度。所以，如果我们要是没有取舍的对于每个消息都调用一次回调函数，那么势必会导致计算越来越不实时，很有可能当下在处理的还是几十秒以前的数据。所以，我们希望每次回调函数都处理当前时刻最新的一个消息，这就是我们的目标。

要达到这个目标有三点，  
第一点是要设置`Publisher`的`queue_size`等于1；  
第二点是要设置`Subscriber`的`queue_size`（消息队列大小）等于`1`；  
第三点非常重要，要设置`Subscriber`的`buff_size`（缓冲区大小）足够大，大于一个消息的大小。  
像这样：

```java
# ROS Python
pcdpub = rospy.Publisher("lidardata", PointCloud, queue_size=1)
rospy.Subscriber("lidardata", PointCloud, self.pcd_resolve_callback,queue_size=1,buff_size=52428800)
```

## Subscriber和Publisher的消息队列起什么作用，队列的大小有什么影响？ 

简单描述一下，Publisher的消息队列是为了缓存发布节点发布的消息，一旦队列中消息的数量超过了queue\_size，那么最先进入队列的（最老的）消息被舍弃。Subscriber的消息队列是为了缓存节点接收到的信息，一旦自己处理的速度过慢，接收到的消息数量超过了queue\_size，那么最先进入队列的（最老的）消息会被舍弃。所以，`我们想只处理最新的消息，实际上只需要把两个queue_size都设置成1，那么系统不会缓存数据，自然处理的就是最新的消息`。

## Subscriber有消息队列缓存消息了，为什么Publisher还要有消息队列？ 

在我看来，Publisher的消息队列是一定要有的，因为ROS中发布节点往外发送消息是基于Topic发送，而不是直接向Subscriber订阅者发送，所以必须要有一个消息队列来存放发布的消息，以供订阅者来获取。而且这个消息队列的好处是在网络差、带宽小、延时高的时候，保证数据不容易丢失。

## 既然Publisher有消息队列了，为什么Subscriber还要有消息队列？ 

这个问题比较难一点。我的理解是，由于ROS毕竟是分布式系统，Publisher和Subscriber不一定在同一台主机上，因此消息需要通过网络来交换。但是网络的性能时好时坏，如果Subscriber没有消息队列，那么每次运行Callback函数前都要先通过网络取回消息，然后才能处理。当网络很差时，就会让系统堵塞。而有消息队列的话，Subscriber就可以一边处理队列中的消息，一边通过网络缓存新的消息，而不用每次处理消息前都要临时去读一个回来。这样就增加了系统的可靠性。

## 为什么要设置缓冲区的大小？ 

这个缓冲区的大小是指消息队列使用的缓冲区物理内存空间大小。如果这个空间小于一个消息所需要的空间，比如消息是一副图片或者一帧点云，数据量超过了缓冲区的大小。这个时候为了保证通信不发生错误，就会触发网络通信的保护机制，TCP的Buffer会为你缓存消息。这种机制就会导致每一帧消息都被完整的缓存下来，没有消息被丢弃，感觉上就像queue\_size被设置成了无穷大。

## 话题和服务的区别： 

话题：异步通信、发布/订阅机制、多对多、传递rosmsg  
服务：同步通信、请求/应答机制、多对一（多客户端一服务端）、传递rossrv

## ROS常用指令： 

`roscore` 启动节点管理器ros master  
`rosrun`： 运行一个ros节点，rosrun pkg\_name node\_name  
`roslaunch` 功能包名称 launch文件 \[参数设置\]：roslaunch vins\_estimator euroc.launch  
roslaunch会检测是否有master存在，不存在会先运行roscore

`rosnode list` //列出当前运行的node信息  
`rosnode info` 节点名 //显示某个node详细信息  
`rosnode kill` 节点名 //终止节点

`rostopic list` 能够列出所有当前订阅和发布的话题  
`rostopic echo topic-name` //实时查看指定话题里发布的任何消息  
`rostopic info topic-name` //查看话题详细信息，比如类型，发布订阅关系等

`rosmsg show + 消息类型`; //显示该消息类型的具体构成

`rosbag record`：录制数据集，rosbag record test\_topic  
`rosbag info`：输出数据集信息  
`rosbag play` ：播放数据集

`tf_monitor` src_frame target
`tf_echo` src_frame target

`static_transform_publisher` x y z yaw pitch roll frameID childFrameID period_ms
`tf view_frames`



常用图形工具：  
`rqt_graph`：各个节点之间的关系，话题的发布订阅关系等  
`rqt_plot`：曲线绘图工具  
`rqt_image_view`：查看摄像头图像

`rviz`：非常强大的图像工具，显示三维地图、坐标系、点云数据、二维信息等；vins-mono以及fast-planner都可以在rviz中进行最终成果的展示  
`gazebo`：gazebo是一款免费的机器人仿真软件，能够在复杂的室内和室外环境中准确高效地模拟机器人工作的功能

## ROS常用函数 

`ros::ok()` 用于检查系统状态，适用于在while循环中判断状态  
`ros::init()` 定义的节点名称,ros::init(argc,argv,“imu\_node”)  
`ros::Rate` 用于设置循环频率，适用于Publisher程序无限循环  
`ros::spin()和ros::spinOnce()` 用于处理ROS的回调函数，适用于Subscriber处理回调数据,ros::spin()调用后不会再返回，也就是你的主程序到这儿就不往下执行了，而ros::spinOnce()在调用后还可以继续执行之后的程序，如果还想再调用，就需要加上循环了。必须由spin或spinOnce回调函数才能被执行。  
`ros::Publisher` 话题发布函数，ros::Publisher pub = nd.advertise<消息类型>("话题名＂, 缓冲区大小);  
`ros::subscribe` 话题订阅函数：subscribe(“话题名”, 缓冲区大小, 消息回调函数)

## ROS常用Eigen库函数 

`Eigen::Matrix(double,m,n) a`;//构造一个大小为(m,n)的double类型的矩阵  
`Eigen::Vector(float,n) b`;//构造一个大小为(n,1)的float类型的vector  
`Eigen::MatrixXf c`;//构造一个动态大小的float类型的矩阵,Eigen支持对动态大小的矩阵和向量重新指定大小  
`Eigen::VectorXf d(30)`;//构造一个动态大小的float类型的vector,大小分配为30  
`Eigen::Matrix4d e;`//比如大小（4，4）的double矩阵  
`Eigen::Vector4d f`;//比如大小（4，1）的double vecto  
`h=g.inverse()`;//对g矩阵求逆矩阵，得到h  
`i=g.transpose()`;//矩阵求转置  
`Eigen::Quaterniond q1 = Eigen::Quaterniond (0.1,0.2,0.1,0.3).normalized();`//四元数归一化  
`Eigen::toRotationMatrix()` 四元数转旋转矩阵  
`Eigen::Quaterniond()` 旋转矩阵转四元数

## ROS多个订阅者的回调之间，是并发的还是单线程的？两个topic同时访问一个资源，会有线程冲突的问题嘛？ 

在只有一个Spinner thread的情况下，callback queue只能顺序执行。这就说明了单线程的不足，不管有多少个Subscriber，节点都只能顺序执行回调，这在某些时候是不能忍受的，因此，多线程有了用武之地，我们要做的事情就是增加spinner thread。

单线程回调spin方法：  
`ros::spin()`——相当于while(true)的大循环，不断遍历执行Callback队列中的可用回调  
`ros::spinOne()`——相当于马上执行一次Callback队列中的可用回调  
多线程回调spin类：  
`ros::MultiThreadedSpinner`——相当于while(true)大循环，启动指定数量的Spinner线程并发执行Callback队列中的可用回调。可指定Callback队列。  
`ros::AsyncSpinner`——异步启停，开启指定数量的Spinner线程并发执行Callback队列中的可用回调。可指定Callback队列。  
简单总结，如果程序简单用ros::spin()就够了；如果程序复杂推荐使用`ros::AsyncSpinner`类。他们的详细用法和区别在ROS官方教程中已经写得比较清楚。

ros::MultiThreadedSpinner  
ros::MultiThreadSpinner是一个阻塞式的spin，类似于ros::spin()。但可以在它的构造函数中指定线程数，如果没有指定(或设置为0)，它将为每个CPU核心使用一个线程。

```java
ros::MultiThreadedSpinner spinner(4); // Use 4 threads
spinner.spin(); // spin() will not return until the node has been shutdown
```

ros::AsyncSpinner  
一个更有用的线程微调器是AsyncSpinner。它不是一个阻塞的spin()调用，而是调用start()和stop()，并且会在它被销毁时自动停止。AsyncSpinner与上面MultiThreadedSpinner示例的等效用法是:

```java
1、ros::AsyncSpinner spinner(4); // Use 4 threads
2、spinner.start();
3、ros::waitForShutdown(); // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown(), or similar.
```

请注意，ros::waitForShutdown()函数不会自己旋转，因此上面的示例总共会旋转4个线程。

## launch启动多个节点，会有订阅者的节点还没起来，但是发布者的节点已经起来了，这样就导致发布的信息订阅者没订阅到，即接收的时候会丢一些数据，如何避免这个问题？ 

发布者发布时检测有没有订阅者，有订阅者再发布；或检测订阅者的节点有没有起来。

## 发布者和订阅者的`buff_size`如何确定该定为多少？ 

对于发布者：

```java
ros::Publisher publisher = nh.advertise<std_msgs::String>("topic_name", 10)
```

以上publisher中的10就是queue\_size，大小并不太重要，一般设置为超过1就行。

一般话题消息序列化和本地TCP发送所需的时间很短，你publish消息的频率很难超过序列化和发送的处理能力，`因此queue_size设置为大概10就行`。

如果话题消息内容很大，比如是图像或者点云，那么为了保证发布和订阅的实时性，一般要另外使用`nodelet`来实现节点间消息传输的零拷贝，即略去了序列化和反序列化过程。

对于订阅者：

对于数据实时性要求高的应用，callback函数要写的尽可能简单，避免执行时间过长。同时`queue_size要设置成1`。  
对于非实时数据流、不能遗漏任何一次消息的情况，比如发送的一些指令话题，queue\_size要设置尽可能的大

## ROS里时间戳的同步是如何做的？比如vins里的imu和相机的时间戳同步，使用的是ros里的什么机制？ 

VINS里相机和IMU时间同步的函数代码：`getMeasurements（）`  
ROS提供了`message_filters::TimeSynchronizer`等同步类来完成多传感器数据同步融合的任务

## TF树里，静态发布和一般发布有什么区别？ 
静态发布：适用于固定的变换关系，仅需发布一次

一般发布：适用于动态的变换关系，需要定时发布
## TF树可以获取过去某个时刻的位姿变化嘛？ 
是的，TF树可以获取过去某个时刻的位姿变化。但需要满足一定条件：

时间缓存：TF树会缓存一段时间内的变换数据（在ROS1中默认是10秒，ROS2中默认是一定的缓存大小）。如果查询的时间点在这个缓存时间范围内，就可以获取该时刻的位姿变化。

数据可用性：在调用查询时，所有相关的变换数据必须已经发布并被缓存。否则，查询可能失败或返回不完整的结果。
## TF树对不同坐标系的发布时间有什么要求吗？ 
TF树对不同坐标系的发布时间有以下要求：

时间同步：不同坐标系的变换时间应当尽量同步。如果存在时间差异，TF树可能会抛出“Extrapolation Exception”异常，因为变换的时间戳不匹配。
连续发布：动态变换通常需要连续发布，以确保坐标系的实时性和数据的有效性。例如，在机器人运动时base_link到odom的变换需要高频率发布，以反映机器人的实际位姿。
缓冲时长：变换发布的频率和持续时间应保证在缓存时长内都有数据，以便后续查询到过去时刻的位姿变化。




## ROS中订阅（Subscribe）最新消息以及消息队列相关问题
    我们希望每次回调函数都处理当前时刻最新的一个消息,要达到这个目标有三点:
    第一点是要设置Publisher的queue_size等于1；
    第二点是要设置Subscriber的queue_size（消息队列大小）等于1；
    第三点非常重要，要设置Subscriber的buff_size（缓冲区大小）足够大，大于一个消息的大小。


```java
# ROS Python
pcdpub = rospy.Publisher("lidardata", PointCloud, queue_size=1)
rospy.Subscriber("lidardata", PointCloud, self.pcd_resolve_callback,queue_size=1,buff_size=52428800)
```



## Subscriber和Publisher的消息队列起什么作用，队列的大小有什么影响？ 
        简单描述一下，Publisher的消息队列是为了缓存发布节点发布的消息，一旦队列中消息的数量超过了queue_size，那么最先进入队列的（最老的）消息被舍弃。Subscriber的消息队列是为了缓存节点接收到的信息，一旦自己处理的速度过慢，接收到的消息数量超过了queue_size，那么最先进入队列的（最老的）消息会被舍弃。所以，我们想只处理最新的消息，实际上只需要把两个queue_size都设置成1，那么系统不会缓存数据，自然处理的就是最新的消息。

## Subscriber有消息队列缓存消息了，为什么Publisher还要有消息队列？
        在我看来，Publisher的消息队列是一定要有的，因为ROS中发布节点往外发送消息是基于Topic发送，而不是直接向Subscriber订阅者发送，所以必须要有一个消息队列来存放发布的消息，以供订阅者来获取。而且这个消息队列的好处是在网络差、带宽小、延时高的时候，保证数据不容易丢失。


## 既然Publisher有消息队列了，为什么Subscriber还要有消息队列？
        这个问题比较难一点。我的理解是，由于ROS毕竟是分布式系统，Publisher和Subscriber不一定在同一台主机上，因此消息需要通过网络来交换。但是网络的性能时好时坏，如果Subscriber没有消息队列，那么每次运行Callback函数前都要先通过网络取回消息，然后才能处理。当网络很差时，就会让系统堵塞。而有消息队列的话，Subscriber就可以一边处理队列中的消息，一边通过网络缓存新的消息，而不用每次处理消息前都要临时去读一个回来。这样就增加了系统的可靠性。

## 为什么要设置缓冲区的大小？
        这个缓冲区的大小是指消息队列使用的缓冲区物理内存空间大小。如果这个空间小于一个消息所需要的空间，比如消息是一副图片或者一帧点云，数据量超过了缓冲区的大小。这个时候为了保证通信不发生错误，就会触发网络通信的保护机制，TCP的Buffer会为你缓存消息。这种机制就会导致每一帧消息都被完整的缓存下来，没有消息被丢弃，感觉上就像queue_size被设置成了无穷大。

## 消息队列的运行机制
        首先，发布节点把消息发布，消息进入Publisher的消息队列，同时通知订阅了该话题消息的Subscriber来取消息。
        其次，Subscriber来Publisher的消息队列里取消息，但取走的也是最老的消息，因为毕竟这是先入先出的队列。这也是为什么Publisher的消息队列的大小也要设置为1。
        最后，被取走的消息存放入了Subscriber的消息队列中，等待被Callback执行。如果Callback执行很慢，消息越堆越多，最老的消息会逐渐被顶替。
        当然，这里究竟是Subscriber来取消息，还是Publisher直接把消息推给Subscriber，我只是猜测，反正这里交换的消息肯定不是最新的消息，而是队列里最老的消息。



## ROS的核心组件有哪些？

ROS的核心组件包括节点（Node）、话题（Topic）、服务（Service）、参数服务器（Parameter Server）、消息（Message）等 。




## ROS是如何处理消息传递的？

ROS使用基于发布-订阅模型的消息传递机制。在这种机制下，消息的发布者（Publisher）将消息发布到一个话题（Topic）中，而消息的订阅者（Subscriber）则从该话题中接收消息。多个订阅者可以同时从一个话题中接收消息 。

## ROS的节点是什么？节点之间如何通信？

节点（Node）是ROS中的一个基本组件，它是一个可执行的进程，用于执行一个特定的任务。节点之间通过话题（Topic）和服务（Service）进行通信 。

## ROS的话题是什么？话题通信的方式有哪些？

话题（Topic）是ROS消息传递机制中的一种基本通信方式。它是一个具有特定类型的消息数据的名称，发布者将消息发布到话题中，订阅者从该话题中接收消息。ROS话题通信的方式包括同步通信和异步通信两种 。

## ROS的服务是什么？服务通信的方式有哪些？

服务（Service）是ROS消息传递机制中的一种基本通信方式。它定义了一个请求和响应的消息类型，客户端可以请求服务并接收响应 。

如何处理订阅最新消息以及消息队列相关问题？

当Subscriber的Callback回调函数中进行费时的操作时，可以通过设置Publisher的queue_size等于1，设置Subscriber的queue_size（消息队列大小）等于1，以及设置Subscriber的buff_size（缓冲区大小）足够大来确保每次回调函数都处理当前时刻最新的一个消息 。

## ROS的总体设计目标是什么？

ROS的总体设计目标是提高机器人开发的软件复用率，开发具有独特的通信机制、丰富的开发工具、海量的应用功能、良好的生态系统集的工具 。



## 重定位和回环检测的区别是什么？

重定位是跟丢以后重新找回当前的姿态，通过当前帧和关键帧之间的特征匹配，定位当前帧的相机位姿。重定位就是重新定位，当前图像因为和最近的图像或者局部地图之间缺乏足够的匹配，导致机器人无法确定自己的位姿，此时处于当前状态的机器人不再知道其在地图中的位置，也叫做机器人被“绑架”，就说的是人质被蒙上双眼带到未知地方，蒙罩去掉后完全不知道自己在哪里，这时候就需要充分利用之前建好的地图或者存好的数据库。此时机器人需要观察周围环境，并且从已有地图中寻找可靠的匹配关系，一般是关键帧信息，这样就可以根据已有信息“重新”估计机器人的姿态。

回环检测是为了解决位置估计随时间漂移的问题。主要是通过识别曾经到过的场景，将其与当前帧对应，优化整个地图信息，包括3D路标点、相机位姿和相对尺度信息。回环的主要目的是降低机器人随时间增加，轨迹中累积的漂移，一般发生在建图过程中。这是因为基于运动传感器或者视觉信息的里程计容易出错，使估计的轨迹偏离其实际真实的情况。通过回环，优化整个地图信息，包括3D路标点、相机位姿和相对尺度信息。回环检测提供了回环帧与所有历史帧的关系，可以极大减少误差。回环主要是纠正机器人/相机轨迹，而重新定位再从未知状态找回姿态。两者都需要当前图像预先访问过之前的位置附近，本质上都是一个图像识别问题。

重定位和回环检测的区别：

重定位主要为了恢复姿态估计，而回环是为了解决漂移，提高全局精度。二者容易混淆的原因是重定位通常也需要找到与之前帧的对应关系求解出姿态，而这可以通过回环来完成，很多算法是可以共享的。

## 单应矩阵H和基础矩阵F的区别是什么？

（1）基础矩阵F和单应矩阵H所求相机获取图像状态不同而选择不同的矩阵。

（2）本质矩阵E和基础矩阵F之间相差相机内参K的运算。

（3）只旋转不平移求出F并分解出的R，T和真实差距大不准确，能求H并分解得R。

## 视觉SLAM方法的分类和对应的特点分析。

视觉SLAM可以分为特征点法和直接法。特征点法是根据提取、匹配特征点来估计相机运动，优化的是重投影误差，对光照变化不敏感，是比较成熟的方案，常见的开源方法有ORB-SLAM等。

特征点法的优点：

（1）特征点本身对光照、运动、旋转比较不敏感，因此稳定性更好。

（2）相机运动较快，也能跟踪成功，鲁棒性较好。

（3）研究时间较久，方案比较成熟。

特征点法的缺点：

（1）关键点提取、描述子匹配时间长。

（2）特征点丢失的场景无法使用。

（3）只能构建稀疏地图。

直接法根据相机的亮度信息估计相机的运动，可以不需要计算关键点和描述子，优化的是光度误差，根据使用像素可分为稀疏、半稠密、稠密三种，常见的方案是SVO、LSD-SLAM等。

直接法的优点：

（1）速度快，可以省去计算特征点和描述子时间。

（2）可以在特征缺失的场合，特征点法在该情况下会急速变差。

（3）可以构建半稠密乃至稠密地图。

直接法的缺点：

（1）因为假设了灰度不变，所以易受光照变化影响。

（2）要求相机运动较慢或采样频率较高。

（3）单个像素或像素块区分度不强，采用的是数量代替质量的策略。

4.关键帧的作用是什么？

关键帧目前是一种非常常用的方法，可以减少待优化的帧数，并且可以代表其附近的帧。

## 如何选择关键帧？

选取关键帧的指标：

（1）距离上一关键帧的帧数是否足够多（时间）。运动很慢的时候，就会选择大量相似的关键帧，冗余、运动快的时候又丢失了很多重要的帧。

（2）距离最近关键帧的距离是否足够远（空间）运动。相邻帧根据姿态计算运动的相对大小，可以是位移，也可以是旋转，或者二者都考虑了。

（3）跟踪质量（主要根据跟踪过程中搜索到的点数和搜索的点数比例）/共视特征点。这种方法记录了当前视角下的特征点数或者视角，当相机离开当前场景时才会新建关键帧，避免了上一种方法存在的问题，缺点是比较复杂。

## 相机传感器的分类及其优缺点是什么？

视觉SLAM常用的相机包括单目相机、双目相机和深度相机。

单目相机的优点：

（1）应用最广，成本可以做到非常低。

（2）体积小，标定简单，硬件搭建也简单。

（3）在有适合光照的情况下，可以适用于室内和室外环境。

单目相机的缺点：

（1）具有纯视觉传感器的通病：在光照变化大，纹理特征缺失、快速运动导致模糊的情况下无法使用。

（2）SLAM过程中使用单目相机具有尺度不确定性，需要专门的初始化。

（3）必须通过运动才能估计深度，帧间匹配三角化。

双目相机一般有Indemind、小觅和ZED等。

双目相机的优点：

（1）相比于单目相机，在静止时就可以根据左右相机视差图计算深度。

（2）测量距离可以根据基线调节。基线距离越大，测量距离越远。

（3）在有适合光照的情况下，可以适用于室内和室外。

双目相机的缺点：

（1）双目相机标定相对复杂。

（2）用视差计算深度比较消耗资源。

（3）具有纯视觉传感器的通病：在光照变化较大、纹理特征缺失、快速运动导致模糊的情况下无法使用。

深度相机一般有Kinect系列、Realsense系列、Orbbec和Pico等。

深度相机的优点：

（1）使用物理测距方法测量深度，避免了纯视觉方法的通病，适用于没有光照和快速运动的情况。

（2）相对双目相机，输出帧率较高，更适合运动场景。

（3）输出深度值比较准，结合RGB信息，容易实现手势识别、人体姿态估计等应用。

深度相机的缺点：

（1）测量范围窄，容易受光照影响，通常只能用于室内场景。

（2）在遇到投射材料、反光表面、黑色物体情况下表现不好，造成深度图确实。

（3）通常分辨率无法做到很高，目前主流的分辨率是640×480.

（4）标定比较复杂。

## ROS中rosrun和roslaunch的区别是什么？

rosrun允许在任意软件包中运行可执行文件，而无需先在其中进行cd或roscd。

Roslaunch可以通过ssh在本地和远程轻松启动多个ros节点，以及在参数服务器上设置参数。它包括自动重生已经死掉的进程的选项。roslaunch接收一个或多个XML配置文件，这些文件指定要设置的参数和要启动的节点以及应在其上运行的计算机。

rosrun只能运行一个节点，如果要运行多个节点，就需要多次使用rosrun命令，而roslaunch可以采用xml格式描述运行的节点，同时运行多个节点。

## 请描述视觉SLAM的框架以及各个模块的作用是什么？

（1）传感器信息读取。在视觉SLAM中主要是相机图像信息的读取和预处理，在机器人中，还会有码盘、惯性传感器等信息的读取和同步。

（2）视觉里程计就是前端，其任务是估算相邻图像间相机运动，以及局部地图的样子。

（3）后端优化。后端接受不同时刻视觉里程计测量的相机位姿，以及回环检测的信息，对它们进行优化，得到全局一致的轨迹和地图。

（4）回环检测。判断机器人是否到达过去先前的位置，如果检测到回环，它会把信息提供给后端进行检测。

（5）建图。根据估计的轨迹，建立与任务要求对应的地图。

## SLAM中的绑架问题是什么？

绑架问题就是重定位，指的是机器人缺少先前位置信息的情况下确定当前位姿。比如机器人在一个已经构建好地图的环境中，但它并不知道自己在地图中的相对位置，或者在移动过程中，由于传感器的暂时性功能故障或者相机的快速移动，导致先前的位置信息丢失，因此得重新确定机器人的位置。初始化绑架是一个通常状况的初始化问题，可以使用粒子滤波方法，重新分散例子到三维空间，被里程信息和随机扰动不断更新，初始化粒子收敛到可解释观察结果的区域。追踪丢失状态绑架，即在绑架发生之前，系统已经保存当前状态，则可以使用除视觉传感器之外的其他的传感器作为候补测量设备。

## 在视觉SLAM中可能用到有关的边缘检测算子有哪些？

在边缘检测一般分为滤波、增强和检测三个步骤，其基本原理是用高斯滤波器进行去噪，之后再用卷积内核寻找像素梯度。边缘检测算子：

（1）canny算子：一种完善的边缘检测算法，抗噪能力强，用高斯滤波平滑图像，用一阶偏导的有限差分计算梯度的幅值和方向，对梯度幅值进行非极大值抑制，采用双阈值检测和连接边缘。

（2）sobel算子：一阶导数算子，引入局部平均运算，对噪声具有平滑作用，抗噪声能力强，计算量较大，但定位精度不高，得到的边缘比较粗，适用于精度要求不高的场合。

（3）laplacian算子：二阶微分算子，具有旋转不变性，容易受噪声影响，不能检测边缘的方向，一般不直接用于检测边缘，而是判断明暗变化。

11.在SLAM中，如何对匹配好的点做进一步的处理，更好保证匹配效果？

（1）确定匹配最大距离，汉明距离小于最小距离的两倍。

（2）使用KNN-matching算法，在这里设置K为2，每个匹配得到两个最接近的描述子，然后计算最接近距离和次接近距离之间的比值，当比值大于既定值时，才作为最终匹配。

（3）使用RANSAC算法找到最佳单应性矩阵，该函数使用的特征点同时包含正确和错误匹配点，因此计算的单应性矩阵依赖于二次投影的准确性。

## SLAM后端有滤波方法和非线性优化方法，这两种方法的优缺点是什么？

滤波方法的优点：在当前计算资源受限、待估计量比较简单的情况下，EKF为代表的滤波方法非常有效，经常用在激光SLAM中。

滤波方法的缺点：存储量和状态量是平方增长关系，因为存储的是协方差矩阵，因此不适合大型场景。但是现在视觉SLAM的方案中特征点的数据很大，滤波方法效率是很低的。

非线性优化方法一般以图优化为代表，在图优化中BA是核心，而包含大量特征点和相机位姿的BA计算量很大，无法实时。在后续的研究中，人们研究了SBA和硬件加速等先进方法，实现了实时的基于图优化的视觉SLAM方法。

## 什么是BA优化？

BA的全称是Bundle Adjustment优化，指的是从视觉重建中提炼出最优的三维模型和相机参数，包括内参和外参。从特征点反射出来的几束光线，在调整相机姿态和特征点空间位置后，最后收束到相机光心的过程。BA优化和冲投影的区别在于，对多段相机的位姿和位姿下的路标点的空间坐标进行优化。


## 描述一下RANSAC算法。

RANSAC算法是随机采样一致算法，从一组含有“外点”的数据中正确估计数学模型参数的迭代算法。“外点”一般指的是数据中的噪声，比如匹配中的误匹配和估计曲线中的离群点。因此，RANSAC算法是一种“外点”检测算法，也是一种不确定的算法，只能在一种概率下产生结果，并且这个概率会随着迭代次数的增加而加大。RANSAC主要解决样本中的外点问题，最多可以处理50%的外点情况。

RANSAC主要通过反复选择数据中的一组随机子集来达成目标，被选取的子集假设为局内点，验证步骤如下：

（1）一个模型适用于假设的局内点，也就是说所有的未知参数都能从假设的局内点计算得到。

（2）使用（1）中得到的模型测试所有其他数据，如果某个点适用于估计的模型，认为它也是局内点。

（3）如果有足够多的点被归类为假设的局内点，则估计的模型就足够合理。

（4）使用假设的局内点重新估计模型，因为它仅仅被初始的假设局内点估计。

（5）最终，通过估计局内点和模型的错误率估计模型。


## 四元数的相关概念是什么，请解释一下。

四元数在程序中使用很广泛，但在SLAM中四元数的概念比较难理解。四元数是Hamilton找到的一种扩展复数，四元数具有一个实部和三个虚部：
其中i,j,k是四元数的三个虚部，满足下式：
也可以使用标量和向量来表示四元数：
在上式中，标量s是四元数的实部，向量v是虚部。
四元数可以表示三维空间中任意一个旋转，与旋转矩阵类似，假设某个旋转是围绕单位向量
进行了角度为θ的旋转，则该旋转的四元数形式为：
上式实质上是模长为1的四元数，也就是单位四元数。反之，也可以通过任意长度为1的四元数计算对应旋转轴和夹角：
如果某个四元数的长度不为1，可以通过归一化转化为模长为1的四元数。
对四元数的θ加上2π，就可以得到相同旋转，但对应的四元数变为-q。因此，在四元数中，任意的旋转都可以由两个互为相反数的四元数表示。如果θ为0的话，则得到一个没有任何旋转的四元数：


## 激光SLAM中的具体方法有什么？请解释一下每种方法的特点。

激光雷达分为单线和多线两种，单线雷达一般应用在平面运动场景，多线雷达应用在三维运动场景。

（1）单线雷达构建二维地图的SLAM算法称为2D lidar SLAM，包括Gmapping、hector、karto和cartographer算法，在二维平面内运动，扫描平面与运动平面平行。

Gmapping是一种基于粒子滤波的2D激光雷达SLAM，构建二维栅格地图。融合里程计信息，没有回环检测。优点是在小场景中，计算量小，速度较快。 缺点是每个粒子都携带一幅地图，无法应对大场景（内存和计算量巨大）；如果里程不准或标定参数不准，在长回廊等环境中容易把图建歪。

hector SLAM是完全基于scan-matching的，使用迭代优化的方法来求匹配的最佳位置，为避免陷入局部极值，也采用多分辨率的地图匹配。 由于完全依赖于scan matching，要求雷达的测量精度较高、角度范围大，扫描速度较高（或移动速度慢）。噪声多、边角特征点少的场景就很容易失败。 原文所提出方法的特点还在于，加入IMU，使用EKF估计整体的6DoF位姿，并根据roll, pitch角将激光扫描数据投影到XY平面，因而支持激光雷达有一定程度的倾斜，比如手持或机器人运动在不是很平整的地面上。

karto是基于scan-matching，回环检测和图优化SLAM算法，采用SPA（Sparse Pose Adjustment）进行优化。

cartographer是谷歌开源的激光SLAM框架，主要特点在于： 1.引入submap，scan to submap matching，新到的一帧数据与最近的submap匹配，放到最优位置上。如果不再有新的scan更新到最近的submap，再封存该submap，再去创建新的submap。 2.回环检测和优化。利用submap和当前scan作回环检测，如果当前scan与已经创建的submap在距离上足够近，则进行回环检测。检测到回环之后用ceres进行优化，调整submap之间的相对位姿。为了加快回环检测，采用分枝定界法。

（2）3D lidar SLAM算法是针对多线雷达的SLAM方法，包括LOAM、Lego-LOAM和LOAM-livox等。

LOAM是针对多线激光雷达的SLAM算法，主要特点在于：1) 前端抽取平面点和边缘点，然后利用scan-to-scan的匹配来计算帧间位姿，也就形成了里程计；2) 由估计的帧间运动，对scan中的每一个点进行运动补偿；3) 生成map时，利用里程计的信息作为submap-to-map的初始估计，再在利用submap和map之间的匹配做一次优化。 LOAM提出的年代较早（2014），还没有加入回环优化。

LeGO-LOAM在LOAM的基础上主要改进：1) 地面点分割，点云聚类去噪；2）添加了ICP回环检测和gtsam优化。

LOAM_livox是大疆2019年公布的面向小FOV Lidar的LOAM算法。相比LOAM，做了一些改动。算法的特点： 1.添加策略提取更鲁棒的特征点：a) 忽略视角边缘有畸变的区域; b) 剔除反射强度过大或过小的点 ; c) 剔除射线方向与所在平台夹角过小的点; d) 部分被遮挡的点 2.与LOAM一样，有运动补偿 3.里程计中剔除相对位姿解算后匹配度不高的点（比如运动物体）之后，再优化一次求解相对位姿。

## 说明UKF，EKF和PF之间的关系。

从精度的角度来看，所有高精度都是通过增加计算量来换来的，如果UKF通过加权减少Sigma点的方法来降低计算负载，那么精度在一定程度上会低于一阶泰勒展开的EKF线性化。除了EKF和UKF之间的时间复杂性问题外，我们还需要检查它们的理论性能。从以往的一些研究中中，我们知道UKF可以将状态估计和误差协方差预测到4阶精度，而EKF只能预测状态估计的2阶和误差协方差的4阶。但是，只有在状态误差分布中的峰度和高阶矩很明显的情况下，UKF才能进行更准确的估计。在我们的应用中，四元数分量协方差的大小显着小于统一性，这意味着峰度和更高阶矩非常小。这一事实说明了为什么UKF的性能不比EKF好。

另外，采样率也是另外一个拉小UKF与EKF差距的因素，对许多动态模型（有论文提到四元数动态）随着采样间隔的缩短，模型愈发趋近准线性化，那么越小的步长，积分步长把（四元数）传播到单位球面的偏差就越小。因此最小化了线性化误差。

最后，也是最根本的在选取EKF和UKF最直观的因素，UKF不用进行雅可比矩阵计算，但是，许多模型的求导是极为简单的，在最根本的地方UKF没有提供更优的解决方案。这也使得，状态模型的雅可比计算的简单性允许我们在计算EKF和UKF用相同的方法计算过程误差协方差。

UKF并不是万能的，也不是一定比EKF优秀，很多时候需要根据情况选择特定的滤波。

## 点云配准算法目前有哪些？

点云配准算法目前有ICP、KC、RPM、形状描述符配准和UPF/UKF。

（1）ICP

ICP算法简单且计算复杂度度低，使它成为最受欢迎的刚性点云配准方法。ICP算法以最近距离标准为基础迭代地分配对应关系，并且获得关于两个点云的刚性变换最小二乘。然后重新决定对应关系并继续迭代知道到达最小值。目前有很多点云配追算法都是基于ICP的改进或者变形，主要改进了点云选择、配准到最小控制策略算法的各个阶段。ICP算法虽然因为简单而被广泛应用。但是它易于陷入局部最大值。ICP算法严重依赖初始配准位置，它要求两个点云的初始位置必须足够近，并且当存在噪声点、外点时可能导致配准失败。

（2）KC

KC算法应用了稳健统计和测量方法。Tsin和Kanade应用核密度估计，将点云表示成概率密度，提出了核心相关（Kernel Correlation，简称KC）算法。这种计算最优配准的方法通过设置两个点云间的相似度测量来减小它们的距离。对全局目标函数执行最优化算法，使目标函数值减小到收敛域。因为一个点云中的点必须和另一个点云中的所有点进行比较，所以这种方法的算法复杂度很高。

（3）RPM

为了克服ICP算法对初始位置的局限性，基于概率论的方法被研究出来。Gold提出了鲁棒点匹配（Robust Point Matching，简称RPM）算法，以及其改进算法。这种方法应用了退货算法减小穷举搜索时间。RPM算法既可以用于刚性配准，也可以用于非刚性配准。对于RPM算法，在存在噪声点或者某些结构缺失时，配准可能失败。

（4）形状描述符配准

形状描述符配准在初始位置很差的情况下也能大体上很好的实现配准。它配准的前提是假设了一个点云密度，在没有这个特殊假设的情况下，如果将一个系数的点云匹配到一个稠密的点云，这种匹配方法将失败。

（5）UPF/UKF

尽管UPF算法能够精确的配准较小的数据集，但是它需要大量的粒子来实现精确配准。由于存在巨大的计算复杂度，这种方法不能用于大型点云数据的配准。为了解决这个问题，UKF算法被提出来，这种方法收到了状态向量是单峰假设的限制，因此，对于多峰分布的情况，这种方法会配准失败。


## 常用命令

1 查看环境变量命令：echo ROS_PACKAGE_PATH ，命令roscd及其它的ROS命令仅适用于环境变量$ROS_PACKAGE_PATH中存在的目录路径，echo命令为字符串输出命令

2 编译工作空间：在工作空间目录下 cd ~/catkin_ws/ catkin_make。catkin_make是在CMake标准工作流程中依次调用了cmake 和 make。

3 显示当前所在工作目录的绝对路径命令 pwd

4 三个文件命令 rospack rosls roscd

6 包中的Mainifest.xml文件定义了包如何构建，运行和记录，和包的依赖项。

7 常常在发生变化之后，使用rospack profile识别新目录。

8 查看包的直接依赖命令 rospack depends1

9 cat命令 显示文件内容命令。

10 package.xml提供程序包的元信息，含有描述标签、维护者标签、许可标签、依赖项标签

11 Nodes:节点,一个节点即为一个可执行文件，它可以通过ROS与其它节点进行通信。

12 修改该文件夹的用户归属关系 sudo chown -R

13 rosnode显示当前运行的ROS节点信息，rosnode list显示当前活跃的节点

14 运行并改变节点名字 rosrun turtlesim turtlesim_node __name:=my_turtle

15 测试节点命令 rosnode ping [node_name]

16 rqt_graph能够创建一个显示当前系统运行情况的动态图形 rosrun rqt_graph rqt_graph

17 rostopic 话题命令 rostopic echo 显示话题上发布的数据。

18 rostopic list -v 列出当前订阅和发布的话题。

19 rostopic type 查看所发布的话题的消息类型，rosmsg show 查看消息的详细情况。可以结合rostopic type和rosmsg show命令来获
取关于某个话题的更深层次的信息 rostopic type /turtle1/command_velocity | rosmsg show

20 rostopic pub 可以把数据发布到当前某个正在广播的话题上，用法 rostopic pub [topic] [msg_type] [args]

21 rostopic hz 查看数据发布的频率。

22 rqt_plot命令可以实时显示一个发布到某个话题上的数据变化图形。rosrun rqt_plot rqt_plot

23 服务允许节点发送请求（request） 并获得一个响应（response）

24 rosservice list 输出可用服务的信息

25 rosservice type [service] 查看服务的类型,查看服务类型的详细信息 rosservice type [service] | rossrv show

26 rosservice call [service] [args] 调用服务

27 rosparam 能够存储并操作ROS参数服务器上的数据。rosparam set 设置参数。

28 rosparam get 获取参数。rosparam get / 可以显示参数服务器上的所有内容。

29 rosparam load [file_name] [namespace] 从文件中读取参数。

30 rosparam dump [file_name] 将参数写入文件。

31 rqt_console属于ROS日志框架(logging framework)的一部分，用来显示节点的输出信息

32 rqt_logger_level允许我们修改节点运行时输出信息的日志等级（logger levels），logger levels包括 DEBUG、WARN、INFO和ERROR

33 roslaunch可以用来启动定义在launch文件中的多个节点，roslaunch [package] [filename.launch]



## 1. 什么是 `ros_control`？它的主要功能是什么？
- **答案要点**：`ros_control` 是一个 ROS 包，提供用于控制机器人硬件的抽象接口。它提供硬件接口、控制器管理和实时控制框架，可以方便地开发和运行控制系统。

## 2. `ros_control` 中的硬件接口（Hardware Interface）是什么？
- **答案要点**：硬件接口是 `ros_control` 的核心，定义了机器人硬件的控制接口。它允许你与机器人硬件（如关节、传感器等）进行通信，实现读取和写入操作。

## 3. 什么是控制器管理器（Controller Manager），它的作用是什么？
- **答案要点**：控制器管理器是 `ros_control` 的一个重要部分，负责加载、启动、停止和卸载控制器。它管理多个控制器并与硬件接口交互。

## 4. 在 `ros_control` 中，如何使用 `JointStateController` 控制关节的状态？
- **答案要点**：`JointStateController` 用于从硬件接口读取关节的状态并发布到 ROS 网络，通常用于发布关节的位置、速度和力等信息。

## 5. `EffortJointInterface` 和 `PositionJointInterface` 有什么区别？
- **答案要点**：`EffortJointInterface` 用于控制关节的力或扭矩，`PositionJointInterface` 用于控制关节的位置。

## 控制器与接口

## 6. 如何在 `ros_control` 中定义一个自定义控制器？
- **答案要点**：定义一个自定义控制器需要继承 `controller_interface::Controller` 类，重写初始化、更新、启动和停止等方法，并将其加载到控制器管理器中。

## 7. `JointTrajectoryController` 是如何工作的？它的作用是什么？
- **答案要点**：`JointTrajectoryController` 用于控制多个关节的轨迹，能够接受一个关节轨迹并将其转换为关节位置、速度和努力的命令。

## 8. 在 `ros_control` 中，如何实现一个 PID 控制器？
- **答案要点**：`ros_control` 提供了一个 PID 控制器接口，可以通过继承 `controller_interface::Controller` 并使用 `pid` 控制算法来调整控制器输出。

## 9. `ForceTorqueSensorInterface` 的作用是什么？
- **答案要点**：`ForceTorqueSensorInterface` 提供一个接口，用于从力矩传感器或其他传感器中读取力和力矩数据。

## 10. 在 ROS 2 中，如何使用 `ros_control` 进行实时控制？
- **答案要点**：ROS 2 使用实时控制框架，`ros_control` 可以与实时操作系统（如 Xenomai 或 RT-preempt）结合使用，以保证控制器的响应时间。

## 高级问题

## 11. 如何在 `ros_control` 中集成外部传感器数据？
- **答案要点**：可以通过 `SensorInterface` 定义一个接口来集成外部传感器数据，例如通过订阅 ROS 主题并将其传递给控制器。

## 12. `ros_control` 中如何处理多个关节同时控制的问题？
- **答案要点**：`ros_control` 通过关节接口（如 `PositionJointInterface` 和 `EffortJointInterface`）处理多个关节的控制，可以通过多个控制器来控制不同的关节。

## 13. 如何配置和使用 `ros_control` 中的 `TrajectoryExecution`？
- **答案要点**：`TrajectoryExecution` 是一个用于执行关节轨迹的功能，它通常与 `JointTrajectoryController` 配合使用。配置时需要定义轨迹、时间和插值算法。

## 14. 在 `ros_control` 中，如何处理机器人硬件的多种状态（如断电、通信丢失等）？
- **答案要点**：`ros_control` 提供了硬件状态检测和故障恢复机制，可以通过监控硬件接口的状态和异常来应对硬件问题。

## 15. 如何在 `ros_control` 中配置传感器数据过滤？
- **答案要点**：可以使用 `SensorFilter` 或自定义滤波器，通过对传感器数据进行平滑处理来提高控制系统的稳定性和准确性。

## 实际应用

## 16. 在 `ros_control` 中，如何实现机器人的力控制？
- **答案要点**：通过使用 `ForceTorqueSensorInterface` 和适当的控制器（如 `EffortController`），可以实现对机器人的力控制。需要实时读取力矩传感器数据，并根据需求调整控制输入。

## 17. `ros_control` 如何与 Gazebo 仿真集成？
- **答案要点**：可以使用 Gazebo 插件与 `ros_control` 集成，插件提供了硬件接口与仿真环境的通信，使得 ROS 控制器可以在 Gazebo 中执行。

## 18. 如何在 `ros_control` 中实现自适应控制器？
- **答案要点**：自适应控制器可以通过调节控制器参数（如增益）来适应环境变化。可以通过读取传感器数据和控制输出调整控制器的参数。

## 19. 如何通过 `ros_control` 控制机器人进行轨迹规划和跟踪？
- **答案要点**：通过 `JointTrajectoryController` 可以接受轨迹命令并生成相应的控制指令来跟踪轨迹。轨迹可以通过外部规划器生成。

## 20. 如何测试和验证 `ros_control` 控制器的正确性？
- **答案要点**：可以使用 ROS 的单元测试框架（如 `rostest`）对控制器进行自动化测试，验证控制器的响应、精度和性能，确保其在各种情况下都能正常工作。



##  ros命名空间NodeHandle相关问题 

## 4.1 NodeHandle的定义 

NodeHandles节点类，ros::NodeHandle类有两个作用：

 *  首先，在roscpp程序的内部，它提供了[RAII][]方式启动和关闭。
 *  其次，它提供了一个额外的层命令空间解决方案，可以使组件更容易写。

## 4.2 NodeHandle的自动启动节点和关闭节点 

自动启动和关闭

 *  ros::NodeHandle管理内部的引用计数
 *  开始节点：

```java
ros::NodeHandle nh;
```

 *  创建时候，如果内部节点没有开始，ros::NodeHandle会开始节点，ros::NodeHandle实例销毁，节点就会关闭。

## 4.3 NodeHandle的命名空间 

1、句柄可以让你通过构造函数指定命名空间

 *  查阅[ROS命名空间文档][ROS]
 *  NodeHandle可以指定命名空间给构造器：

```java
ros::NodeHandle nh("my_namespace");
```

 *  这使得使用该句柄的任何相对名字都是相对`<node\_namespace>/my\_namespace`，而不只是相对`<node\_namespace>`。
 *  也可以指定父NodeHandle和命名空间：

```java
ros::NodeHandle nh1("ns1");
ros::NodeHandle nh2(nh1, "ns2");
```

 *  这个放nh2 进入<node\_namespace>/ns1/ns2 命名空间。

2、也可以指定全局名字

```java
ros::NodeHandle nh("/my_global_namespace");
```

这种做法并不推荐，因为这样会使得节点无法被放入别的命名空间。只是有时在代码中使用全局名字有用。

 *  你可以指定全局名，如：

```java
ros::NodeHandle nh("/my_global_namespace");
```

 *  这通常不鼓励，因为它阻止了节点被推为命名空间（如roslaunch）。然而，有时在代码中使用全局名称可能是有用的。

3、私有名字

使用私有名字比直接调用有私有名的句柄方法更有技巧，你可以在一个私有命名空间中直接创建一个新的句柄。

```java
ros::NodeHandle nh("~my_private_namespace"); ros::Subscriber sub = nh.subscribe("my_private_topic",....);
```

以上例子会订阅<node\_name>/my\_private\_namespace/my\_private\_topic === （node\_namespace/nodename/my\_private\_namespace/my\_private\_topic）

注意：理解的重点上文中红色标注的部分，node\_namespace和node\_name是两回事！

node\_name = node\_namespace \+ node name

```java
// launch 文件中 ns=="node_namespace" 
ros::init(argc, argv, "node_name"); // node name 

ros::NodeHandle n; //n 命名空间为/node_namespace 


ros::NodeHandle n1("sub"); // n1命名空间为/node_namespace/sub 
ros::NodeHandle n2(n1,"sub2");// n2命名空间为/node_namespace/sub/sub2 


ros::NodeHandle pn1("~"); //pn1 命名空间为/node_namespace/node_name 
ros::NodeHandle pn2("~sub"); //pn2 命名空间为/node_namespace/node_name/sub 
ros::NodeHandle pn3("~/sub"); //pn3 命名空间为/node_namespace/node_name/sub 
ros::NodeHandle gn("/global"); // gn 命名空间为/global
```

综上，只要不是明确指定，就会用节点名node\_name来指定命名空间。


## move_base
### 全局规划器
#### base_global_planner
- ##### NavfnROS
  有bug 老版本
- ##### GlobalPlanner
- ##### CarrotPlanner
  经常被作为自定义规划器的模板



### 局部规划器
#### base_local_planner
  一般设置为odom为基础坐标系

  ##### Trajectory Planner //ROS默认
  内部实际为DWA算法，代码效率低
  ##### DWA Planner  //动态窗口

  ##### TEB Planner 
  加入时间因素考虑，提供代价地图优化插件，运动平滑性和执行效率更高；倾向于倒车转弯，天生适合阿克曼底盘
  ##### Eband Planner  



# 2D SLAM  

## HectorMapping

1. **无里程计依赖**  
   完全依靠激光雷达数据进行定位与建图，不需要轮式里程计，适合里程计数据不可靠或不存在的情况。
2. **多分辨率地图**  
   使用多分辨率栅格地图进行扫描匹配，低分辨率地图用于快速定位，高分辨率地图用于精细优化，提高了效率和精度。

3. **适合静态 2D 环境**  
   主要用于平面 2D 场景，如室内或静态环境，对动态障碍物的处理效果有限。



## Gmapping

1. **自带点云障碍物配准算法**  
   Gmapping 内置障碍物配准算法，可以直接处理激光雷达扫描数据并与地图中的栅格信息进行匹配，提升定位精度。

2. **不依赖里程计**  
   Gmapping 可以在没有里程计（Odometry）数据的情况下运行，通过激光雷达数据直接进行建图与定位，适合不具备高精度里程计的场景。

3. **基于粒子滤波**  
   使用粒子滤波算法，每个粒子表示一种可能的位姿假设。Gmapping 根据传感器数据动态更新粒子权重，确保机器人位姿的精确估计。

4. **自适应粒子数量**  
   Gmapping 具备自适应粒子数量功能，可以根据环境复杂程度动态调整粒子数。简单环境减少粒子数，复杂环境增多粒子数，以在精度与计算效率间平衡。



# 3D SLAM  
## Cartographer
## RTAB-Map
## OctoMap 

# 规划算法
## Dijkstra 
  Dijkstra算法用于求解带权图中单源最短路径的算法，适用于所有边权为非负数的情况。它的核心是逐步扩展到各节点，记录当前已知的最短路径，直到目标节点或所有节点都已找到最短路径。具体流程如下：

        初始化：将起始节点的路径长度设为0，所有其他节点的路径长度设为∞（即不可达），并标记所有节点未访问。
        选择节点：从未访问的节点中选择路径长度最小的节点，将其标记为已访问。
        更新路径：对于当前节点的每个相邻节点，计算从起始节点到该相邻节点的路径长度。如果新路径长度小于当前记录的路径长度，则更新路径长度。
        重复步骤：重复步骤2和步骤3，直到所有节点都被访问，或找到目标节点的最短路径。
        结束：记录起点到各节点的最短路径，若有目标节点则输出最短路径。
## A* 
1. **深度优先**  
