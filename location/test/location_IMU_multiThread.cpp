#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <angles/angles.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <location/location_tool.hpp>
#include <location/tic_toc.h>

#include <lidar_correction/lidarCorrection.hpp>
#include <boost/circular_buffer.hpp>
#include <receive_stm32/stm32_utcData.h>
#include <receive_xsens/Imu_withSync.h>
#include <lidar_correction/correct_vlp_xsens.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;
using namespace LidarCorrection;
using namespace CorrectVlpXsens;

string VLPType, LiDARName, MapPCDFileName, imuSyncDataName, utcDataName;
bool useImuToCorrectPointCloud;//是否使用imu去矫正激光数据，会导致处理速度变慢，16线激光不矫正 2ms CPU2.8G，矫正9.4ms
VPointCloud mapCloud;

struct globalPoseData
{
  boost::mutex mutex;
  ros::Time globalPose_time;
  Eigen::Matrix4f globalPose = Eigen::Isometry3f::Identity().matrix();
  Eigen::Matrix4f globalPoseDelta = Eigen::Isometry3f::Identity().matrix();
};
globalPoseData globalPose;

tf::TransformBroadcaster *odom_broadcaster(NULL);

//unsigned int countGlobal = 0;
unsigned int countLoop = 0;
//Eigen::Matrix4f initGlobalPose = Eigen::Isometry3f::Identity().matrix();
//Eigen::Matrix4f globalPose = Eigen::Isometry3f::Identity().matrix();
//Eigen::Matrix4f globalPoseDelta = Eigen::Isometry3f::Identity().matrix();



//激光雷达回调函数更新的内容
ros::Time current_time;
VPointCloud newScan;
Eigen::Matrix4f firstLidarTransRT = Eigen::Isometry3f::Identity().matrix();

//位置初始化与重定位
bool initPoseFlag = false;
bool resetPoseFlag[2] = {false, false};
Eigen::Matrix4f initGlobalPose = Eigen::Isometry3f::Identity().matrix();

//bool newScanFlag = false;
//bool initFlag = false;
//bool initPoseFlag = false;
//bool resetPoseFlag = false;

//ros::Time current_time;

//ros接口线程获取地图参数后 通知处理线程进行初始化
boost::condition_variable initThreadParam_cv;
boost::mutex initThreadParam_cv_mutex;
bool threadQuitFlag = false;

//ros接口线程获取位置初始化后 通知处理线程进行初始化
boost::condition_variable initPoseParam_cv;
boost::mutex initPoseParam_cv_mutex;

//激光雷达数据获取成功后 通知处理线程进行处理
boost::condition_variable lidarData_cv;
boost::mutex lidarData_cv_mutex;

//处理线程完成后 通知ros接口线程发布数据
boost::condition_variable lidarProcess_cv;
boost::mutex lidarProcess_cv_mutex;
bool lidarProcessFlag = false;


VelodyneCorrection::Ptr correction_ptr(new VelodyneCorrection);
correct_vlp_xsens _correct_vlp_xsens(correction_ptr);
ros::Publisher pubLaserImuCloud;



void VelodyneScanHandler(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
{
  current_time =  scanMsg->header.stamp;

  //当tf发布的时间和这个激光的时间不一致时，rviz 会自动进行tf插值，然后rviz将点云显示出来就会出现和地图不匹配的问题！
  //处理激光雷达数据时间，进行硬件同步校准，输出第一束激光的时间
  boost::unique_lock<boost::mutex> lock(lidarData_cv_mutex);//上锁，更新数据
  newScan.clear();
   _correct_vlp_xsens.VelodyneScanHandler(scanMsg, newScan, firstLidarTransRT, useImuToCorrectPointCloud);

  sensor_msgs::PointCloud2 PonitMap2;
  pcl::toROSMsg(newScan, PonitMap2);
  PonitMap2.header.seq = scanMsg->header.seq;
  PonitMap2.header.stamp = scanMsg->header.stamp;
  //PonitMap2.header.stamp = current_time;
  PonitMap2.header.frame_id = "/velodyne";
  pubLaserImuCloud.publish(PonitMap2);  // 发布基于IMU旋转后的激光点云

//  newScanFlag = true;
  lidarData_cv.notify_one();//分配一个线程去处理数据
  //countGlobal++;
}
/**
 * @brief receivePose
 * 接收其他node发布的初始化位置
 * @param msg 输入的初始化位置，可以是rviz发布的，也可以是其他程序发布的
 */
void receivePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if(!initPoseFlag)
  {
    cout << "received a pose, init" << endl;
    initGlobalPose(0,3) = msg->pose.pose.position.x;
    initGlobalPose(1,3) = msg->pose.pose.position.y;
    initGlobalPose(2,3) = 0;//狗相比于当时采集数据的坐标系原点的z方向的偏差，需要根据实际情况调节。
    Eigen::Quaternionf temp(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);//w+xi+yj+zk
    initGlobalPose.block(0,0,3,3) = temp.matrix();

    //初始化全局位置
    boost::unique_lock<boost::mutex> lock(globalPose.mutex);
    globalPose.globalPose_time = current_time;
    globalPose.globalPose = initGlobalPose;

    initPoseFlag = true;
    initPoseParam_cv.notify_all();//通知所有线程，初始化位置。
  }
  else
  {
    cout << "received a pose, reset" << endl;
    initGlobalPose(0,3) = msg->pose.pose.position.x;
    initGlobalPose(1,3) = msg->pose.pose.position.y;
    initGlobalPose(2,3) = 0;//狗相比于当时采集数据的坐标系原点的z方向的偏差，需要根据实际情况调节。
    Eigen::Quaternionf temp(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);//w+xi+yj+zk
    initGlobalPose.block(0,0,3,3) = temp.matrix();
    resetPoseFlag[0] = true;
    resetPoseFlag[1] = true;

  }
}




/**
 * @brief hardSyncHandler 等待单片机硬件时钟
 * @param hardSyncData_ptr
 */
void hardSyncHandler(const receive_stm32::stm32_utcData::ConstPtr &hardSyncData_ptr)
{
    _correct_vlp_xsens.hardSyncHandler(hardSyncData_ptr);
}




/**
 * @brief imuHandler 填充imu环形buffer
 * @param imuData
 */
void imuHandler(const receive_xsens::Imu_withSync imuData)
{
  _correct_vlp_xsens.imuHandler(imuData);
}
void process_1()
{
  {
    boost::mutex::scoped_lock lock(initThreadParam_cv_mutex);
    initThreadParam_cv.wait(lock);
    cout << "thread 1 map param init." << endl;
  }
  //需要通过作用域限定来自动释放锁
  //https://www.boost.org/doc/libs/1_62_0/libs/fiber/doc/html/fiber/synchronization/conditions.html

  //加载地图
  SubMap submap(mapCloud, Eigen::Isometry3f::Identity().matrix(), 1.0, 1.0);

  {
    boost::mutex::scoped_lock lockPose(initPoseParam_cv_mutex);
    initPoseParam_cv.wait(lockPose);//等待激光数据分配,条件变量没有到达时会释放锁，等条件变量触发后会再次获得锁
    submap.setLocalPose(initGlobalPose);
    cout<< "thread 1 get pose" << endl << initGlobalPose <<endl;
  }

  boost::mutex::scoped_lock lockLidarData(lidarData_cv_mutex);
  while(!threadQuitFlag)
  {
    VPointCloud temp;
    ros::Time _current_time;
    lidarData_cv.wait(lockLidarData);//等待激光数据分配
    {
      _current_time = current_time;//获取当前帧的时间
      temp = newScan;
    }
    static Eigen::Matrix4f firstLidarTransRT_Before = firstLidarTransRT;
    lidarData_cv_mutex.unlock();//数据获取完毕，放弃锁。

    TicToc tt;
    Eigen::Matrix4f _globalPoseDelta = globalPose.globalPose;
    Eigen::Matrix4f _globalPose = submap.getPose(newScan, firstLidarTransRT_Before.inverse() * firstLidarTransRT);
    firstLidarTransRT_Before = firstLidarTransRT;
    _globalPoseDelta = _globalPoseDelta.inverse()*_globalPose;

    //模拟处理时间太长
    //boost::this_thread::sleep(boost::posix_time::milliseconds(200));

    if(_current_time < globalPose.globalPose_time)//当前这一帧计算太慢，新的pose（下一帧激光）已经被另外线程计算获得，就舍弃当前结果
    {
      cout << "thread 1 time out!" << tt.toc() <<" ms"<< endl;
      continue;
    }
    else
    {
      boost::unique_lock<boost::mutex> lock(globalPose.mutex);
      globalPose.globalPose = _globalPose;
      globalPose.globalPoseDelta = _globalPoseDelta;
      globalPose.globalPose_time = _current_time;
    }

    lidarProcessFlag = true;
    cout << "thread 1 process " << tt.toc() <<" ms" <<endl;

    if( resetPoseFlag[0] )
    {
      resetPoseFlag[0] = false;
      submap.setLocalPose(initGlobalPose);
      cout<< "thread 1 get pose" << endl << initGlobalPose <<endl;
    }
  }
}


void process_2()
{
  {
    boost::mutex::scoped_lock lock(initThreadParam_cv_mutex);
    initThreadParam_cv.wait(lock);
    cout << "thread 2 map param init." << endl;
  }
  //需要通过作用域限定来自动释放锁
  //https://www.boost.org/doc/libs/1_62_0/libs/fiber/doc/html/fiber/synchronization/conditions.html

  //加载地图
  SubMap submap(mapCloud, Eigen::Isometry3f::Identity().matrix(), 1.0, 1.0);

  {
    boost::mutex::scoped_lock lockPose(initPoseParam_cv_mutex);
    initPoseParam_cv.wait(lockPose);//等待pose初始化
    submap.setLocalPose(initGlobalPose);
    cout<< "thread 2 get pose" << endl << initGlobalPose <<endl;
  }

  boost::mutex::scoped_lock lockLidarData(lidarData_cv_mutex);
  while(!threadQuitFlag)
  {
    VPointCloud temp;
    ros::Time _current_time;
    lidarData_cv.wait(lockLidarData);//等待激光数据分配,条件变量没有到达时会释放锁，等条件变量触发后会再次获得锁
    {
      _current_time = current_time;//获取当前帧的时间
      temp = newScan;
    }
    static Eigen::Matrix4f firstLidarTransRT_Before = firstLidarTransRT;
    lidarData_cv_mutex.unlock();//数据获取完毕，放弃锁。

    TicToc tt;
    Eigen::Matrix4f _globalPoseDelta = globalPose.globalPose;
    Eigen::Matrix4f _globalPose = submap.getPose(newScan, firstLidarTransRT_Before.inverse() * firstLidarTransRT);
    firstLidarTransRT_Before = firstLidarTransRT;
    _globalPoseDelta = _globalPoseDelta.inverse()*_globalPose;

    if(_current_time < globalPose.globalPose_time)//当前这一帧计算太慢，新的pose（下一帧激光）已经被另外线程计算获得，就舍弃当前结果
    {
      cout << "thread 2 time out!" << tt.toc() <<" ms"<< endl;
      continue;
    }
    else
    {
      boost::unique_lock<boost::mutex> lock(globalPose.mutex);
      globalPose.globalPose = _globalPose;
      globalPose.globalPoseDelta = _globalPoseDelta;
      globalPose.globalPose_time = _current_time;
    }

    lidarProcessFlag = true;
    cout << "thread 2 process " << tt.toc() <<" ms" <<endl;

    if( resetPoseFlag[1] )
    {
      resetPoseFlag[1] = false;
      submap.setLocalPose(initGlobalPose);
      cout<< "thread 2 get pose" << endl << initGlobalPose <<endl;
    }
  }
}

/**
 * @brief process
 * 订阅激光雷达数据，输出全局位置。
 * @param argc
 * @param argv
 * @return
 */
int rosInterface(int argc, char** argv)
{
  using namespace std;
  ros::init(argc, argv, "location_IMU_multiThread");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  ros::Publisher pubMapCloud;
  ros::Publisher odom_pub;

  ros::Publisher pubLaserCloud;

  ros::Subscriber goalSubscriber;
  ros::Subscriber subImuLaserPoint;
  ros::Subscriber subImuString;
  ros::Subscriber subUTC;
  nh_param.param<string>("VLPType", VLPType, "HDL_32E");//VLP16
  nh_param.param<string>("LiDARName", LiDARName, "/velodyne_packets");
  nh_param.param<string>("MapPCDFileName", MapPCDFileName, " ");
  nh_param.param<string>("imuSyncDataName", imuSyncDataName, "/imu/data_withSync");
  nh_param.param<string>("utcDataName", utcDataName, "/stm32/utc_data");
  nh_param.param<bool>("useImuToCorrectPointCloud", useImuToCorrectPointCloud, true);

  if(VLPType == "HDL_32E")
  {
    //注意！一定要初始化！
    correction_ptr->setParameters(LidarCorrection::VelodyneCorrection::ModelType::HDL_32E);
  }
  else if(VLPType == "VLP_16")
  {
    correction_ptr->setParameters(LidarCorrection::VelodyneCorrection::ModelType::VLP_16);
  }
  else
  {
    cout << "VLP Type Wrong. Input is " << VLPType << endl;
    return 0;
  }

  goalSubscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, receivePose);
  subImuLaserPoint = nh.subscribe<velodyne_msgs::VelodyneScan>(LiDARName, 10, VelodyneScanHandler);
  subImuString = nh.subscribe<receive_xsens::Imu_withSync> (imuSyncDataName, 200, imuHandler, ros::TransportHints().tcpNoDelay());
  subUTC = nh.subscribe<receive_stm32::stm32_utcData> (utcDataName, 10, hardSyncHandler, ros::TransportHints().tcpNoDelay());

  pubLaserImuCloud = nh.advertise<sensor_msgs::PointCloud2>  //基于imu旋转后的数据点
      ("/imu_laser_point", 100);
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>  //原始数据点
      ("/laser_point", 100);
  pubMapCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 1);  //变换到地图坐标系中的mappoint
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  tf::TransformBroadcaster odom_broadcaster_;
  ros::Rate rate(100);//100Hz

  //加载地图
  pcl::io::loadPCDFile<VPoint> ( MapPCDFileName, mapCloud );
  cout<<"point cloud loaded, piont size = "<<mapCloud.points.size()<<endl;
  //SubMap submap(mapCloud, globalPose, 1.0, 1.0);
  sensor_msgs::PointCloud2 sensor_msgsPonitMap2;
  pcl::toROSMsg(mapCloud, sensor_msgsPonitMap2);
  cout << "init map" << endl;
  initThreadParam_cv.notify_all();//通知所有线程，初始化地图。

  //等待初始化位置
  while (ros::ok())
  {
    if(initPoseFlag)
    {
      cout<< "get pose" << endl << initGlobalPose <<endl;
      break;
    }

    if(countLoop % 100 == 0)
    {
      sensor_msgs::PointCloud2 PonitMap2;
      pcl::toROSMsg(mapCloud, PonitMap2);
      PonitMap2.header.stamp = ros::Time::now();
      PonitMap2.header.frame_id = "/map";

      pubMapCloud.publish(PonitMap2);
    }
    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时
    countLoop++;
  }
  cout << "location start." << endl;

  // 先获得base_link到velodyne的tf转换关系，以便发布odom到base_link的tf
  tf::StampedTransform tf_base2velodyne;
  tf::TransformListener tf_listener;
  tf_listener.waitForTransform("/base_link", "/velodyne", ros::Time(0), ros::Duration(1.0));
  tf_listener.lookupTransform("/base_link", "/velodyne", ros::Time(0), tf_base2velodyne);

  while (ros::ok())
  {
    if(lidarProcessFlag)
    {
      lidarProcessFlag = false;

      //发布ros的内容
      Eigen::Quaterniond eigen_quat(globalPose.globalPose.block<3,3>(0,0).cast<double>());
      Eigen::Vector3d eigen_trans(globalPose.globalPose.block<3,1>(0,3).cast<double>());
      tf::Quaternion tf_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
      tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), eigen_trans(2));

      //计算map到velodyne的tf关系,current_timed对应于回调函数中激光雷达获取的时间，而不是当前时间。
      tf::StampedTransform tf_odom2velodyne(tf::Transform(tf_quat, tf_trans), globalPose.globalPose_time, "odom", "velodyne");

      //计算odom到base_link的tf关系。
      tf::StampedTransform tf_map2base;
      tf_map2base.mult(tf_odom2velodyne, tf_base2velodyne.inverse());
      tf_map2base.stamp_ = globalPose.globalPose_time;
      tf_map2base.frame_id_ = "odom";
      tf_map2base.child_frame_id_ = "base_link";

      geometry_msgs::TransformStamped tf_msgs;
      tf::transformStampedTFToMsg(tf_map2base, tf_msgs);
      //send the transform
      odom_broadcaster_.sendTransform(tf_msgs);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = globalPose.globalPose_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      //set the position
      odom.pose.pose.position.x = tf_msgs.transform.translation.x;
      odom.pose.pose.position.y = tf_msgs.transform.translation.y;
      odom.pose.pose.position.z = tf_msgs.transform.translation.z;
      odom.pose.pose.orientation = tf_msgs.transform.rotation;

      Eigen::Matrix3f r_matrix = globalPose.globalPoseDelta.block<3,3>(0,0);
      Eigen::Vector3f r_delta = r_matrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
      //set the velocity
      odom.twist.twist.linear.x = globalPose.globalPoseDelta(0,3);
      odom.twist.twist.linear.y = globalPose.globalPoseDelta(1,3);
      odom.twist.twist.linear.z = globalPose.globalPoseDelta(2,3);
      odom.twist.twist.angular.x = r_delta(0);
      odom.twist.twist.angular.y = r_delta(1);
      odom.twist.twist.angular.z = r_delta(2);

      //publish the message
      odom_pub.publish(odom);

      //每隔10秒，发布一次地图点云。rviz中关闭地图点云后再打开如果接受不到map点云，就不会更新。
      countLoop++;
      if(countLoop % 10 == 0)
      {
        sensor_msgsPonitMap2.header.stamp = ros::Time::now();
        sensor_msgsPonitMap2.header.frame_id = "/map";
        pubMapCloud.publish(sensor_msgsPonitMap2);
      }
    }
    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时
  }
  threadQuitFlag = true;
  return 0;
}

int main(int argc, char** argv)
{
  boost::thread thrd1(&rosInterface, argc, argv);
  boost::thread thrd2(&process_1);
  boost::thread thrd3(&process_2);
  thrd1.join();
  thrd2.join();
  thrd3.join();
  return 0;
}
