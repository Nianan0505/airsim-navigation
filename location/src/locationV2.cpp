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
#include <location/state_machineV2.hpp>
#include <location/added_function.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace locationV2;
using namespace state_machineV2;
using namespace addedFunction;

ros::Publisher g_pubMapCloud;
ros::Publisher g_pubOdom;
ros::Publisher g_pubStatus;
ros::Publisher g_pubSwitchPoints;
ros::Publisher g_pubDebugInformation;
ros::Subscriber g_subPose;
ros::Subscriber g_subLidarPoint;

state_machine::Ptr g_pStateMachine;

tf::StampedTransform g_tf_base2LidarSensor1;

switchPointsTool::Ptr g_pSwitchPointsTool;


void publishOdomAndTF(tf::TransformBroadcaster &odom_broadcaster)
{
  //发布odom
  Eigen::Quaterniond eigen_quat(g_pStateMachine->getNewGlobalPose().block<3,3>(0,0).cast<double>());
  Eigen::Vector3d eigen_trans(g_pStateMachine->getNewGlobalPose().block<3,1>(0,3).cast<double>());

  tf::Quaternion tf_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
  tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), g_tf_base2LidarSensor1.getOrigin()[2]);
  //tf::Vector3 tf_trans(eigen_trans(0), eigen_trans(1), eigen_trans(2));



  //计算map到LidarSensor1的tf关系,current_time_lidard对应于回调函数中激光雷达获取的时间，而不是当前时间。
  tf::StampedTransform tf_odom2LidarSensor1(tf::Transform(tf_quat, tf_trans),
                                        pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp),
                                        "map",
                                        g_pStateMachine->getNewHeader().frame_id);

  //计算odom到base_link的tf关系。
  tf::StampedTransform tf_map2base;
  tf_map2base.mult(tf_odom2LidarSensor1, g_tf_base2LidarSensor1.inverse());
  tf_map2base.stamp_ = pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp);
  tf_map2base.frame_id_ = "map";
  tf_map2base.child_frame_id_ = "base_link";

  geometry_msgs::TransformStamped tf_msgs;
  tf::transformStampedTFToMsg(tf_map2base, tf_msgs);
  //send the transform
  odom_broadcaster.sendTransform(tf_msgs);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = pcl_conversions::fromPCL(g_pStateMachine->getNewHeader().stamp);
  //odom.header.seq = current_seq_lidar;//odom的seq设置无效
  odom.header.frame_id = "map";
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
}

void publishDebugInformation()
{
  std::vector<float> temp;
  g_pStateMachine->getRecordData(temp);
  std_msgs::Float32MultiArray rosData;
  rosData.data = temp;

  g_pubDebugInformation.publish(rosData);
}

void lidarCloudHandler(const sensor_msgs::PointCloud2ConstPtr& lidarCloudMsg)
{
  VPointCloud::Ptr pNewScan(new VPointCloud());
  pcl::fromROSMsg(*lidarCloudMsg, *pNewScan);//这个转化包含了header与data所有的转化。
  g_pStateMachine->updataLidarData(pNewScan);
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

//恢复模式下会timeout吗？
void publishTimeOutStatus()
{
  std_msgs::Int32 temp;
  temp.data = 100;
  g_pubStatus.publish(temp);
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


  string lidarName, mapPCDFileName;
  int switchBufferSize;
  bool enableSwitchLidarData;
  bool enablePublishGlobalDenseMap;

  float expectFrequency=10;//hz


  nh_param.param<string>("lidarName", lidarName, "/LidarSensor1_points");
  nh_param.param<string>("mapPCDFileName", mapPCDFileName, " ");
  nh_param.param<bool>("enableSwitchLidarData", enableSwitchLidarData, false);
  nh_param.param<int>("switchBufferSize", switchBufferSize, 4);
  nh_param.param<bool>("enablePublishGlobalDenseMap", enablePublishGlobalDenseMap, false);

  g_subPose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, receivePose, ros::TransportHints().tcpNoDelay());
  g_subLidarPoint = nh.subscribe<sensor_msgs::PointCloud2>(lidarName, 10, lidarCloudHandler, ros::TransportHints().tcpNoDelay());
  g_pubMapCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 10);  //变换到地图坐标系中的mappoint
  g_pubOdom = nh.advertise<nav_msgs::Odometry>("odom", 50);
  g_pubStatus = nh.advertise<std_msgs::Int32>("locationStatus",10);//发布定位状态
  g_pubSwitchPoints = nh.advertise<sensor_msgs::PointCloud2>("switchedCloud", 10);
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
  g_pStateMachine = boost::make_shared<state_machine>(p_mapCloud, 1.0, 1.0, 1.0, enableSwitchLidarData, enablePublishGlobalDenseMap);
  g_pSwitchPointsTool = boost::make_shared<switchPointsTool>(switchBufferSize);
  cout << "init done. waitting for input a initial pose..." << endl;

  //size check 不起作用
//  cout << "size of source map(Byte): " << sizeof(*p_mapCloud) <<endl;
//  cout << "size of KDTree(Byte): " << sizeof(g_pStateMachine->m_pLidarLocation->m_currentKDTree) <<endl;
//  cout << "size of NdtMap(Byte): " << sizeof(g_pStateMachine->m_pLidarLocation->m_currentNdtMap) <<endl;

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

  ROS_INFO("----------------------------==============================");
  // 先获得base_link到LidarSensor1的tf转换关系，以便发布odom到base_link的tf
  tf_listener.waitForTransform("/base_link", "/LidarSensor1", ros::Time(0), ros::Duration(1.0));
  tf_listener.lookupTransform("/base_link", "/LidarSensor1", ros::Time(0), g_tf_base2LidarSensor1);

  cout << "location start." << endl;


  while (ros::ok())
  {

    //有新的处理好的激光数据就会返回true
    if(g_pStateMachine->loopOnce())
    {
      publishStatus();

      if(g_pStateMachine->getStatus() == state_machine::Status::normal)
      {
        publishOdomAndTF(odom_broadcaster);

        //发布调试信息
        publishDebugInformation();

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
//    //间隔性检测是否超时
//    {
//      static int step_count=0;
//      if(step_count++ == 10)
//      {
//        step_count = 0;
//        if(g_pStateMachine->isLidarDataTimeOut())
//        {
//          publishTimeOutStatus();
//        }
//      }
//    }


    ros::spinOnce();//查询回调函数中断标志位。执行回调函数。
    rate.sleep();//以ros::Rate来延时

    if(rate.cycleTime() > ros::Duration(1.0 / expectFrequency))
      ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",expectFrequency, rate.cycleTime().toSec());

  }
  return 0;
}
