#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
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
#include <location/tic_toc.h>
#include <location/locationV2.hpp>
#include <location/state_machine_with_imu_V2.hpp>
#include <location/added_function.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace locationV2;
using namespace state_machineV2;
using namespace addedFunction;

ros::Publisher g_pubMapCloud;
ros::Publisher g_pubOdom;
ros::Publisher g_pubOdomWithHardTime;
ros::Publisher g_pubLaserImuCloud;
ros::Publisher g_pubStatus;
ros::Publisher g_pubSwitchPoints;
ros::Publisher g_pubDebugInformation;
ros::Subscriber g_subPose;
ros::Subscriber g_subLidarPoint;
ros::Subscriber g_subImuWithSync;
ros::Subscriber g_subUtc;
state_machine_with_imu::Ptr g_pStateMachine;

tf::StampedTransform g_tf_base2velodyne;

switchPointsTool::Ptr g_pSwitchPointsTool;


void publishOdomAndTF(tf::TransformBroadcaster &odom_broadcaster)
{
  //发布odom
  Eigen::Quaterniond eigen_quat(g_pStateMachine->getNewGlobalPose().block<3,3>(0,0).cast<double>());
  Eigen::Vector3d eigen_trans(g_pStateMachine->getNewGlobalPose().block<3,1>(0,3).cast<double>());

  tf::Quaternion tf_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
  tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), g_tf_base2velodyne.getOrigin()[2]);//确保baselink的z为0
  //tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), eigen_trans(2));



  //计算map到velodyne的tf关系,current_time_lidard对应于回调函数中激光雷达获取的时间，而不是当前时间。
  tf::StampedTransform tf_odom2velodyne(tf::Transform(tf_quat, tf_trans),
                                        pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp),
                                        "odom",
                                        g_pStateMachine->getNewHeader().frame_id);

  //计算odom到base_link的tf关系。
  tf::StampedTransform tf_map2base;
  tf_map2base.mult(tf_odom2velodyne, g_tf_base2velodyne.inverse());
  tf_map2base.stamp_ = pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp);
  tf_map2base.frame_id_ = "odom";
  tf_map2base.child_frame_id_ = "base_link";

  geometry_msgs::TransformStamped tf_msgs;
  tf::transformStampedTFToMsg(tf_map2base, tf_msgs);
  //send the transform
  odom_broadcaster.sendTransform(tf_msgs);



  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp);
  //odom.header.seq = current_seq_lidar;//odom的seq设置无效
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  //set the position
  odom.pose.pose.position.x = tf_msgs.transform.translation.x;
  odom.pose.pose.position.y = tf_msgs.transform.translation.y;
  odom.pose.pose.position.z = tf_msgs.transform.translation.z;
  odom.pose.pose.orientation = tf_msgs.transform.rotation;

  Eigen::Matrix3f r_matrix = g_pStateMachine->getNewGlobalDeltaPose().block<3,3>(0,0);
  Eigen::Vector3f r_delta = r_matrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
  //set the velocity
  odom.twist.twist.linear.x = g_pStateMachine->getNewGlobalDeltaPose()(0,3);
  odom.twist.twist.linear.y = g_pStateMachine->getNewGlobalDeltaPose()(1,3);
  odom.twist.twist.linear.z = g_pStateMachine->getNewGlobalDeltaPose()(2,3);
  odom.twist.twist.angular.x = r_delta(0);
  odom.twist.twist.angular.y = r_delta(1);
  odom.twist.twist.angular.z = r_delta(2);

  //publish the message
  g_pubOdom.publish(odom);


  odom.header.stamp = g_pStateMachine->getLidarHardTime();
  g_pubOdomWithHardTime.publish(odom);

  //cout << "delta time in hard time" << ros::Time::now()-(g_pStateMachine->getLidarHardTime()+g_pStateMachine->getHardSysAndRosSysdeltaTime()) <<endl;
  //cout << "delta time in ros time" << ros::Time::now()-pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp) <<endl;
  cout << "time now " << ros::Time::now()<< " new scan " << pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp) << "delta time in ros time" << ros::Time::now()-pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp) <<endl;
  cout << "imu time " << g_pStateMachine->getCurrentImuRosTime() << endl;
}

//基于imu发布最新的姿态，
void publishOdomAndTFWithImu(tf::TransformBroadcaster &odom_broadcaster)
{
//  static int step=0;
//  if(step++ < 3)
//    return;
//  step = 0;
  if(g_pStateMachine->getCurrentImuRosTime() < pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp))
  {
    cout << "imu is old " << g_pStateMachine->getCurrentImuRosTime() << " "
         << pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp) << endl;
  }

  Eigen::Matrix4f imuDelta = g_pStateMachine->getImuPoseAtLidarPoseMoment().inverse() * g_pStateMachine->getCurrentImuValue();

  Eigen::Matrix4f poseWithImu = g_pStateMachine->getNewGlobalPose() * imuDelta;
  //发布odom
  Eigen::Quaterniond eigen_quat(poseWithImu.block<3,3>(0,0).cast<double>());
  Eigen::Vector3d eigen_trans(poseWithImu.block<3,1>(0,3).cast<double>());

  tf::Quaternion tf_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
  tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), g_tf_base2velodyne.getOrigin()[2]);
  //tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), eigen_trans(2));



  //计算map到velodyne的tf关系,发布的时间是imu在系统中的时间。
  tf::StampedTransform tf_odom2velodyne(tf::Transform(tf_quat, tf_trans),
                                        g_pStateMachine->getCurrentImuRosTime(),
                                        "odom",
                                        g_pStateMachine->getNewHeader().frame_id);

  //计算odom到base_link的tf关系。
  tf::StampedTransform tf_map2base;
  tf_map2base.mult(tf_odom2velodyne, g_tf_base2velodyne.inverse());
  tf_map2base.stamp_ = g_pStateMachine->getCurrentImuRosTime();
  tf_map2base.frame_id_ = "odom";
  tf_map2base.child_frame_id_ = "base_link";

  geometry_msgs::TransformStamped tf_msgs;
  tf::transformStampedTFToMsg(tf_map2base, tf_msgs);
  //send the transform
  odom_broadcaster.sendTransform(tf_msgs);


//odom暂时不发布，因为涉及到速度发布。

//  //next, we'll publish the odometry message over ROS
//  nav_msgs::Odometry odom;
//  odom.header.stamp = g_pStateMachine->getCurrentImuRosTime();
//  //odom.header.seq = current_seq_lidar;//odom的seq设置无效
//  odom.header.frame_id = "odom";
//  odom.child_frame_id = "base_link";
//  //set the position
//  odom.pose.pose.position.x = tf_msgs.transform.translation.x;
//  odom.pose.pose.position.y = tf_msgs.transform.translation.y;
//  odom.pose.pose.position.z = tf_msgs.transform.translation.z;
//  odom.pose.pose.orientation = tf_msgs.transform.rotation;

//  Eigen::Matrix3f r_matrix = g_pStateMachine->getNewGlobalDeltaPose().block<3,3>(0,0);
//  Eigen::Vector3f r_delta = r_matrix.eulerAngles(0,1,2);//roll pitch yaw 顺序
//  //set the velocity
//  odom.twist.twist.linear.x = g_pStateMachine->getNewGlobalDeltaPose()(0,3);
//  odom.twist.twist.linear.y = g_pStateMachine->getNewGlobalDeltaPose()(1,3);
//  odom.twist.twist.linear.z = g_pStateMachine->getNewGlobalDeltaPose()(2,3);
//  odom.twist.twist.angular.x = r_delta(0);
//  odom.twist.twist.angular.y = r_delta(1);
//  odom.twist.twist.angular.z = r_delta(2);

//  //publish the message
//  g_pubOdom.publish(odom);


//  odom.header.stamp = g_pStateMachine->getLidarHardTime();
//  g_pubOdomWithHardTime.publish(odom);

  //cout << "delta time in hard time" << ros::Time::now()-(g_pStateMachine->getLidarHardTime()+g_pStateMachine->getHardSysAndRosSysdeltaTime()) <<endl;
  //cout << "delta time in ros time" << ros::Time::now()-pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp) <<endl;
  cout << "in imu. time now " << ros::Time::now()<< " new scan " << pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp) << "delta time in ros time" << ros::Time::now()-pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp) <<endl;
  cout << "in imu. imu time " << g_pStateMachine->getCurrentImuRosTime() << endl;
}


void publishDebugInformation()
{
  std::vector<float> temp;
  g_pStateMachine->getRecordData(temp);
  std_msgs::Float32MultiArray rosData;
  rosData.data = temp;

  g_pubDebugInformation.publish(rosData);
}

void publishLidarPoints()
{
  sensor_msgs::PointCloud2 PonitMap2;
  pcl::toROSMsg(*(g_pStateMachine->getNewScan()), PonitMap2);

//  PonitMap2.header.seq = g_pStateMachine->getNewHeader().seq;
//  PonitMap2.header.stamp = pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp);
//  PonitMap2.header.frame_id = g_pStateMachine->getNewHeader().frame_id;
  g_pubLaserImuCloud.publish(PonitMap2);
}

void lidarCloudHandler(const velodyne_msgs::VelodyneScan::ConstPtr scanMsg)
{
  g_pStateMachine->updataLidarData(scanMsg);
}

void utcHandler(const receive_stm32::stm32_utcData::ConstPtr hardSyncData_ptr)
{
  g_pStateMachine->updataUtcTimeData(hardSyncData_ptr);
}

void imuHandler(const receive_xsens::Imu_withSync imuData)//要修改为指针
{
  g_pStateMachine->updataImuData(imuData);
}

/**
 * @brief receivePose
 * 接收其他node发布的初始化位置
 * @param msg 输入的初始化位置，可以是rviz发布的，也可以是其他程序发布的
 */
void receivePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  Eigen::Matrix4f pose(Eigen::Isometry3f::Identity().matrix());
  pose(0,3) = msg->pose.pose.position.x;
  pose(1,3) = msg->pose.pose.position.y;
  pose(2,3) = 0;//这个变量要在launch中可调
  Eigen::Quaternionf temp(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);//w+xi+yj+zk
  pose.block<3,3>(0,0) = temp.matrix();

  g_pStateMachine->updataCallBackPose(pose);
  cout<< "get pose" << endl << pose <<endl;
}

void publishStatus()
{
  std_msgs::Int32 temp;
  temp.data = (int)g_pStateMachine->getStatus();
  g_pubStatus.publish(temp);
}

void stateMachineThread()
{

}
/**
 * @brief main
 * 订阅激光雷达数据，输出全局位置。
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{

  using namespace std;
  ros::init(argc, argv, "locationV2");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");

  string VLPType, lidarName, mapPCDFileName, imuWithSyncName, utcName;
  int switchBufferSize;
  bool enableSwitchLidarData;
  bool enablePublishGlobalDenseMap;

  float expectFrequency=10;//hz



  nh_param.param<string>("VLPType", VLPType, "VLP_16");
  nh_param.param<string>("imuWithSyncName", imuWithSyncName, "/imu/data_withSync");
  nh_param.param<string>("utcName", utcName, "/stm32/utc_data");
  nh_param.param<string>("lidarName", lidarName, "/velodyne_points");
  nh_param.param<string>("mapPCDFileName", mapPCDFileName, " ");
  nh_param.param<bool>("enableSwitchLidarData", enableSwitchLidarData, false);
  nh_param.param<int>("switchBufferSize", switchBufferSize, 4);
  nh_param.param<bool>("enablePublishGlobalDenseMap", enablePublishGlobalDenseMap, false);


  g_subPose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, receivePose, ros::TransportHints().tcpNoDelay());
  g_subLidarPoint = nh.subscribe<velodyne_msgs::VelodyneScan>(lidarName, 10, lidarCloudHandler);
  g_subImuWithSync = nh.subscribe<receive_xsens::Imu_withSync>(imuWithSyncName, 1200, imuHandler, ros::TransportHints().tcpNoDelay());
  g_subUtc = nh.subscribe<receive_stm32::stm32_utcData>(utcName, 10, utcHandler);
  g_pubMapCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 1);  //变换到地图坐标系中的mappoint
  g_pubOdom = nh.advertise<nav_msgs::Odometry>("odom", 50);
  g_pubOdomWithHardTime = nh.advertise<nav_msgs::Odometry>("odomWithHardTime", 50);
  g_pubStatus = nh.advertise<std_msgs::Int32>("locationStatus",10);//发布定位状态
  g_pubSwitchPoints = nh.advertise<sensor_msgs::PointCloud2>("switchedCloud", 10);
  g_pubLaserImuCloud = nh.advertise<sensor_msgs::PointCloud2>("LidarPoints", 10);
  g_pubDebugInformation = nh.advertise<std_msgs::Float32MultiArray>("debug_information", 10);
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformListener tf_listener;
  ros::Rate rate(1000);//1000Hz，对应于1ms的延时等待查询。


  //加载地图
  VPointCloud::Ptr p_mapCloud (new VPointCloud());
  pcl::io::loadPCDFile<VPoint> ( mapPCDFileName, *p_mapCloud );
  cout<<"map loaded, piont size = "<<p_mapCloud->points.size()<<endl;
  sensor_msgs::PointCloud2 sensor_msgsPonitMap2;
  pcl::toROSMsg(*p_mapCloud, sensor_msgsPonitMap2);
  sensor_msgsPonitMap2.header.frame_id = "/map";

  cout << "waiting! init map" << endl;
  VelodyneCorrection::ModelType modelType;
  if(VLPType == "VLP_16")
  {
    modelType = VelodyneCorrection::ModelType::VLP_16;
  }
  else if(VLPType == "HDL_32E")
  {
    modelType = VelodyneCorrection::ModelType::HDL_32E;
  }
  else
  {
    modelType = VelodyneCorrection::ModelType::UNINIT;
  }

  g_pStateMachine = boost::make_shared<state_machine_with_imu>(
        p_mapCloud, 1.0, 1.0, 1.0,
        enableSwitchLidarData, enablePublishGlobalDenseMap, modelType);

  g_pSwitchPointsTool = boost::make_shared<switchPointsTool>(switchBufferSize);
  cout << "init done. waitting for input a initial pose..." << endl;

  boost::thread thrd2(&stateMachineThread);
  thrd2.join();

  //等待初始化位置


  while (ros::ok())
  {
    static unsigned int countLoop=0;

    if(g_pStateMachine->initDone())
      break;

    if(countLoop % 1000 == 0)
    {
      sensor_msgsPonitMap2.header.stamp = ros::Time::now();
      g_pubMapCloud.publish(sensor_msgsPonitMap2);
    }
    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时
    countLoop++;
  }

  // 先获得base_link到velodyne的tf转换关系，以便发布odom到base_link的tf
  tf_listener.waitForTransform("/base_link", "/velodyne", ros::Time(0), ros::Duration(1.0));
  tf_listener.lookupTransform("/base_link", "/velodyne", ros::Time(0), g_tf_base2velodyne);

  cout << "location start." << endl;


  while (ros::ok())
  {

    //带返回的等待新的pose



    if(g_pStateMachine->loopOnce())//由新的激光雷达帧触发
    {
      publishStatus();
      //注意，这儿发布的点云是从packets解析的，timeStamp使用的是packet的，
      //因此，之后发布的tf也是基于这个packets的timeStamp
      //rviz中如果使用原始驱动看velodyne的points会和地图不对齐，因为原厂驱动发布的points的timeStamp和packets不同
      //这里发布的点云是结构化velodyne点云。且时间戳和packets对齐。
      publishLidarPoints();

      //发布调试信息
      publishDebugInformation();


      if(g_pStateMachine->getStatus() == state_machine_with_imu::Status::normal)
      {
        publishOdomAndTF(odom_broadcaster);

        if(g_pStateMachine->getEnableSwitchLidarData())
        {
          g_pSwitchPointsTool->update(g_pStateMachine->getNewScan(), g_pStateMachine->getNewGlobalPose());

          sensor_msgs::PointCloud2 PonitMap2;
          pcl::toROSMsg(*g_pSwitchPointsTool->getSwitchedPoints(), PonitMap2);

          PonitMap2.header.seq = g_pStateMachine->getNewHeader().seq;
          PonitMap2.header.stamp = pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp);
          PonitMap2.header.frame_id = g_pStateMachine->getNewHeader().frame_id;
          g_pubSwitchPoints.publish(PonitMap2);
        }

        //发布全局点云地图，注意，会影响性能
        if(g_pStateMachine->getEnablePublishGlobalDenseMap())
        {
          static unsigned int countLoop=0;

          if(countLoop++ % 1000 == 0)
          {
            countLoop=0;
            sensor_msgsPonitMap2.header.stamp = ros::Time::now();
            g_pubMapCloud.publish(sensor_msgsPonitMap2);
          }
        }
      }
    }

    if(g_pStateMachine->getStatus() == state_machine_with_imu::Status::normal)
    {
        //imu有更新再发布
        static Eigen::Matrix4f currentImuValue = Eigen::Isometry3f::Identity().matrix();

        if(currentImuValue != g_pStateMachine->getCurrentImuValue())
        {
            currentImuValue = g_pStateMachine->getCurrentImuValue();
            publishOdomAndTFWithImu(odom_broadcaster);
        }
    }

    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时

    if(rate.cycleTime() > ros::Duration(1.0 / expectFrequency))
      ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",expectFrequency, rate.cycleTime().toSec());

  }
  return 0;
}
