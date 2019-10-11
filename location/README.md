# location


注意 clone时可以指定深度，这样减小下载量。
```
git clone git://xxoo --depth 1
```


总体框架

![总体框架](https://images.gitee.com/uploads/images/2018/1227/153136_52441310_1121244.png "屏幕截图.png")

#### 开发环境

ubuntu16.04 

[ros Kinetic](http://wiki.ros.org/kinetic/Installation)

#### 需要具备的知识（重要！）
1. [tf坐标系转化关系](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF)
2. [eigen矩阵变换](http://eigen.tuxfamily.org/index.php?title=Main_Page)
3. [pcl ndt配准](http://pointclouds.org/documentation/tutorials/normal_distributions_transform.php#normal-distributions-transform)
4. [地图服务器 map_server](http://wiki.ros.org/map_server)

#### 项目介绍
ndt定位程序

基于全局点云地图，与激光雷达的点云数据
1.纯激光模式：给出初始的位姿后进行定位，定位速度10Hz（激光雷达发布的速度）
2.激光+imu模式：给出初始的位姿后进行定位，定位速度10Hz，姿态角400Hz。

#### 地图说明
每个地图由3个文件组成分别是用于定位的***.pcd和用于导航的配置文件***.yaml  静态地图***.pgm。

其中配置文件格式如下。想使用不同的静态地图时，只需要修改里面的名称（image）即可。

```
image: tengxun.pgm
resolution: 0.100000
origin: [-83.999997, -25.400000, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

静态地图由黑白像素组成，黑色是障碍物，白色是可通行区域。

可通过ps修改静态地图来实现虚拟障碍物的功能，也可以让机器人远离一些人为不想让它过去的地方。

注意，pgm格式比较特殊，ubuntu下使用gimp，windows下使用photoshop。

```
sudo apt-get install gimp
```

地图文件在 location.launch 中调用，修改好地图后需要同步修改launch文件中的yaml文件。

```
 <node name="MapServer" pkg="map_server" type="map_server" output="screen" args="$(find location)/map/tengxun.yaml"/>
```

#### 使用说明

确保传感器已经开启

#### 1.普通模式——只使用激光雷达

兼容16线和32线

```
roslaunch velodyne_pointcloud VLP16_points.launch
roslaunch location location.launch 
```
开启地图服务器 map_server和定位程序location。
之后在rviz中按上方的2D Pose Estimate ，在地图中点点击对应的位置进行初始化。


#### 2.imu模式——使用激光雷达、imu、单片机硬件时间

兼容16线和32线

会订阅velodyne的packets数据，发布结构化点云。

会额外发布带有硬件时间戳的odom数据

```
roslaunch receive_stm32 receive_stm32.launch
roslaunch velodyne_pointcloud VLP16_points.launch
roslaunch receive_xsens receive_xsens.launch

roslaunch location location_imuV2.launch 
```
开启地图服务器 map_server和定位程序location。
之后在rviz中按上方的2D Pose Estimate ，在地图中点点击对应的位置进行初始化。


#### 数据包

玉泉地图

```
链接：https://pan.baidu.com/s/1A8WTkl-vLPUJl7TWc2dJBQ 
提取码：zc26 
```

velodyne16线+imu+硬件同步数据包

```
链接：https://pan.baidu.com/s/1tiptHtHLxEPKXopSu5ZpMQ 
提取码：2fil 
```

速腾16线+imu+硬件同步数据包

```
链接：https://pan.baidu.com/s/13fnU6wV8SNiRPZKs_6ExxQ 
提取码：p1ld 
```



