<<<<<<< HEAD
# airsim仿真导航

# 环境配置

从github下载airsimv1.2.0-Linux，其他的都说是windows版本，编译后的环境可能有问题，下载后:

> cd AirSim
>
> ./setup.sh
>
> ./build.sh

当运行某一个环境时，比如Blocks是从home中`/Documents/AirSim`含有的settings.json中加载的，settings.json中有对于无人车(无人机)的设定，主要有:

- "SettingsVersion": 1.2, //这个是一定要有的
- "SimMode": "Car",  //选择Car还是Multirotor
- “ViewMode”： 选择FlyWithMe（for drones）或者SpringArmChase（for cars)
- "Vehicles"中对car有更加详细的设定，比如"VehicleType": "PhysXCar",  RC(Remote Control)和各种sensors

# settings中sensors的设定

每个传感器都有自己对应的num，相机除外，传感器与num的对应关系如下:

- barometer 气压计：1
- imu ：2
- gps ：3
- magnetometer 磁力计：4
- Distance Sensor ：5
- Lidar ：6

有一些传感器是默认的，对于drone来说是barometer  imu gps magnetometer，对于车来说只有gps

```c++
"DefaultSensors": {
    "Barometer": {
         "SensorType": 1,
         "Enabled" : true
    },
    "Imu": {
         "SensorType": 2,
         "Enabled" : true
    },
    "Gps": {
         "SensorType": 3,
         "Enabled" : true
    },
    "Magnetometer": {
         "SensorType": 4,
         "Enabled" : true
    },
    "Distance": {
         "SensorType": 5,
         "Enabled" : true
    },
    "Lidar2": { 
         "SensorType": 6,
         "Enabled" : true,
         "NumberOfChannels": 4,
         "PointsPerSecond": 10000
    }
},
```

## IMU

ros节点无法正确获取IMU，目前该问题还没解决，但是issue中有提到用C++正确获取IMU，该方法已经得到验证

```c++
msr::airlib::MultirotorRpcLibClient client;  // or msr::airlib::CarRpcLibClient
        msr::airlib::Kinematics::State k_state = client.simGetGroundTruthKinematics();
        msr::airlib::Environment::State e_state = client.simGetGroundTruthEnvironment();

        // timestamp
        auto t = Utils::getTimeSinceEpochSecs();

        // angular velocity of IMU - angular velocity in the body frame
        msr::airlib::Vector3r w_b = k_state.twist.angular;
        // linear acceleration of body in world frame
        msr::airlib::Vector3r la_b_w =  k_state.accelerations.linear;
        // subtract gravity
        la_b_w = la_b_w - e_state.gravity;
        // get acceleration in the body frame - IMU linear acceleration
        msr::airlib::Vector3r la_b = msr::airlib::VectorMath::transformToBodyFrame(la_b_w, k_state.pose.orientation, true);
```

## 相机

相机没有对应的num，不同类型的相机既可以用名称，也可以用ID，对应关系如下：

```C++
front_center : 0
front_left: 1
front_right : 2
fpv : 3
back_center : 4
```

在函数ImageRequest中的第一个参数设定图像类型
图像类型ImageType的对应关系如下：

```C++
  Scene = 0, 
  DepthPlanner = 1, 
  DepthPerspective = 2,
  DepthVis = 3, 
  DisparityNormalized = 4,
  Segmentation = 5,
  SurfaceNormals = 6,
  Infrared = 7
```

同样可以在car_image_raw.py中的ImageRequest中确定，在settings.json中可以指定
关于相机的参数，默认值可通过如下确定：

```C++
"CameraDefaults": {
      "CaptureSettings": [
        {
          "ImageType": 0,
          "Width": 1920,
          "Height": 1080,
          "FOV_Degrees": 90,
          "AutoExposureSpeed": 100,
          "MotionBlurAmount": 0
        }
    ]
  },
```

其中width与height需要与car_image_raw.py中如下对应

```c++
   msg.height = 1080  # resolution should match values in settings.json
   msg.width = 1920
```

### 得到一张单独的图像

从名字为0的相机得到单张图像，返回的是png格式的bytes值，

```C++
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

int getOneImage() 
{
    using namespace std;
    using namespace msr::airlib;
    
    //for car use CarRpcLibClient
    //msr::airlib::MultirotorRpcLibClient client;
    msr::airlib::CarRpcLibClient client;

    vector<uint8_t> png_image = client.simGetImage("0", VehicleCameraBase::ImageType::Scene);
    //do something with images
}
```

### 得到多张图片

从单个API中可以获取多个view，既可以得到未压缩的RGB图像，也可以得到浮点单通道图像

```python
import airsim #pip install airsim

# for car use CarClient() 
client = airsim.MultirotorClient()

responses = client.simGetImages([
    # png format
    airsim.ImageRequest(0, airsim.ImageType.Scene), 
    # uncompressed RGB array bytes
    airsim.ImageRequest(1, airsim.ImageType.Scene, False, False),
    # floating point uncompressed image
    airsim.ImageRequest(1, airsim.ImageType.DepthPlanner, True)])
```

如果用python 的numpy库来处理图像，可以得到未压缩的RGB图像，将其转换成numpy

```python
responses = client.simGetImages([ImageRequest("0", airsim.ImageType.Scene, False, False)])
response = responses[0]

# get numpy array
img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

# reshape array to 4 channel image array H X W X 4
img_rgb = img1d.reshape(response.height, response.width, 3)

# original image is fliped vertically
img_rgb = np.flipud(img_rgb)

# write to png 
airsim.write_png(os.path.normpath(filename + '.png'), img_rgb)

```

`simGetImage`  can accept request for multiple image types from any cameras in single call.可以指定图像是压缩的png，还是未压缩的rgb，还是浮点格式；
压缩的png 对应的是 binary string literal
float array  对应的是python list of float64 ，可以转换为Numpy 2D array 通过 airsim.list_to_2d_float_array(response.image_data_float, response.width, response.height)

#### C++

```c++
int getStereoAndDepthImages() 
{
    using namespace std;
    using namespace msr::airlib;
    
    typedef VehicleCameraBase::ImageRequest ImageRequest;
    typedef VehicleCameraBase::ImageResponse ImageResponse;
    typedef VehicleCameraBase::ImageType ImageType;

    //for car use
    //msr::airlib::CarRpcLibClient client;
    msr::airlib::MultirotorRpcLibClient client;

    //get right, left and depth images. First two as png, second as float16.
    vector<ImageRequest> request = { 
        //png format
        ImageRequest("0", ImageType::Scene),
        //uncompressed RGB array bytes
        ImageRequest("1", ImageType::Scene, false, false),       
        //floating point uncompressed image  
        ImageRequest("1", ImageType::DepthPlanner, true) 
    };

    const vector<ImageResponse>& response = client.simGetImages(request);
    //do something with response which contains image data, pose, timestamp etc
}

```

`simGetCameraInfo`会返回指定相机的位姿pose和视场角FOV，`simGetCameraOrientation`设定指定相机的角度，在NED坐标系下的四元数，`airsim.to_quaternion`函数会将pith roll yaw转为四元数，eg：
`client.simSetCameraOrientation(0, airsim.to_quaternion(0.261799, 0, 0)); `
`#将名字为0的相机的gimbal的pitch设定到0-15度`

### 改变分辨率和相机参数

如果想改变分辨率和视场角FOV，可以在settings.json中改
如果使用立体相机，目前左右之间的距离被设定在25cm；

### 不同的图像类型

#### DepthPlanner and DepthPerspective

图像类型及对应值如下：

```
  Scene = 0, 
  DepthPlanner = 1, 
  DepthPerspective = 2,
  DepthVis = 3, 
  DisparityNormalized = 4,
  Segmentation = 5,
  SurfaceNormals = 6,
  Infrared = 7

```

  如果想以浮点数的形式获取深度图像（例如：设定`pixels_as_float`为true) ，并且在ImageRequest中设定ImageType为DepthPlanner或者DepthPerspective，前者只要与camera平行的都有相同的深度值，后者用投影射线，可以将透视深度反馈到ros中生成点云，

#### Depthvis

根据距离相机平面的距离，像素从黑到白，纯黑即0米，纯白即100m或者更多；

#### DisparityNormalized

ach pixel is `(Xl - Xr)/Xmax`, which is thereby normalized to values between 0 to 1.:

## 雷达

```C++
"Sensors": {
			    "LidarSensor1": { 
					"SensorType": 6,
					"Enabled" : true,  //类似config
					"NumberOfChannels": 16, //16线激光
					"RotationsPerSecond": 10,
					"PointsPerSecond": 100000,
					"X": 0, "Y": 0, "Z": -1,
					"Roll": 0, "Pitch": 0, "Yaw" : 0,  //定义了位姿
					"VerticalFOVUpper": -15,
					"VerticalFOVLower": -25,
					"HorizontalFOVStart": -20,
					"HorizontalFOVEnd": 20, //定义了最下最上角度和最左最右角度
					"DrawDebugPoints": true, //是否在环境中可视
					"DataFrame": "SensorLocalFrame" //垂直惯性坐标系"VehicleInertialFrame" or 传感器坐标系"SensorLocalFrame"
				},
				"LidarSensor2": {  //可定义多个雷达
				   "SensorType": 6,
					"Enabled" : true,
					"NumberOfChannels": 4,
					"RotationsPerSecond": 10,
					"PointsPerSecond": 10000,
					"X": 0, "Y": 0, "Z": -1,
					"Roll": 0, "Pitch": 0, "Yaw" : 0, 
					"VerticalFOVUpper": -15,
					"VerticalFOVLower": -25,
					"DrawDebugPoints": true, 
					"DataFrame": "SensorLocalFrame"
				}
			}

```

getLidarData()函数返回了点云数组、时间戳和雷达位姿，其中：
点云在雷达坐标系中（NED坐标系，以米为单位）
雷达位姿在车的坐标系中（NED坐标系，以米为单位）

# OriginGeopoint

此设置指定放置在环境中的“Player Start“ 的纬度、经度和高度，使用该变换计算车辆的原点。请注意，该API使用的坐标都是国际单位制的NED系统，这意味着每辆车在NED系统中从（0，0，0）开始。计算“origingeopoint”中指定的地理坐标的时间设置。
NED系统即：north (x+) east(y+) down(z+) 与gazebo只有z相反

# APIs for carstate

setCarControls：
可以设置throttle（节气门）/ steering（转向） / handbrake（手刹） / auto/manual gear（自动或手动驾驶）
getCarState：
获取speed current gear 和6个运动学参数（position orientation linear/angular velocity linear/angular acceleration）
除了angular velocity和acceleration 在车坐标系下，其他在世界坐标系 且均为NED系统

```c++
#include <iostream>
#include "vehicles/car/api/CarRpcLibClient.hpp"

int main() 
{
    msr::airlib::CarRpcLibClient client;
    client.enableApiControl(true); //this disables manual control
    CarControllerBase::CarControls controls;

    std::cout << "Press enter to drive forward" << std::endl; std::cin.get();
    controls.throttle = 1;
    client.setCarControls(controls);

    std::cout << "Press Enter to activate handbrake" << std::endl; std::cin.get();
    controls.handbrake = true;
    client.setCarControls(controls);

    std::cout << "Press Enter to take turn and drive backward" << std::endl; std::cin.get();
    controls.handbrake = false;
    controls.throttle = -1;
    controls.steering = 1;
    client.setCarControls(controls);

    std::cout << "Press Enter to stop" << std::endl; std::cin.get();
    client.setCarControls(CarControllerBase::CarControls());

    return 0;
}


```

struct CarControls 如下：

```c++
    struct CarControls {
        float throttle = 0; /* 1 to -1 */
        float steering = 0; /* 1 to -1 */
        float brake = 0;    /* 1 to -1 */
        bool handbrake = false;
        bool is_manual_gear = false;
        int manual_gear = 0;
        bool gear_immediate = true;

        CarControls()
        {
        }
        CarControls(float throttle_val, float steering_val, float brake_val, bool handbrake_val,
            bool is_manual_gear_val, int manual_gear_val, bool gear_immediate_val)
            : throttle(throttle_val), steering(steering_val), brake(brake_val), handbrake(handbrake_val),
            is_manual_gear(is_manual_gear_val), manual_gear(manual_gear_val), gear_immediate(gear_immediate_val)
        {
        }
        //设定油门
        void set_throttle(float throttle_val, bool forward)
        {
        //如果向前
            if (forward) {
                is_manual_gear = false;
                manual_gear = 0;
                throttle = std::abs(throttle_val);
            }
            //如果向后
            else {
                is_manual_gear = false;
                manual_gear = -1;
                throttle = - std::abs(throttle_val); //给的是油门的绝对值，所以有负号
            }
        }
    };

```

struct CarState如下：

```c++
    struct CarState {
        float speed;
        int gear;
        float rpm;
        float maxrpm;
        bool handbrake;
        Kinematics::State kinematics_estimated;
        uint64_t timestamp;

        CarState(float speed_val, int gear_val, float rpm_val, float maxrpm_val, bool handbrake_val, 
            const Kinematics::State& kinematics_estimated_val, uint64_t timestamp_val)
            : speed(speed_val), gear(gear_val), rpm(rpm_val), maxrpm(maxrpm_val), handbrake(handbrake_val), 
              kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
        {
        }
    };

```

# airsim的ros接口

airsim最新提供了ros的无人机接口，主要在airsim_ros_pkgs 和airsim_tutorials

_pkg中，接口可以在airsim中的AirLib中看到

我把ros包放在了单独的工作空间airsim_ws中，编译ros包时需要修改以下几个地方:

- CMakeLists中 `cmake_minimum_required(VERSION 3.10.0)`如果报错可以手动改成3.5.0 好像是编译airsim时会自动安装cmake3.10，然后编译ros包时去调用装好的3.10，但是有可能出错，手动改成3.5目前还没出现问题；
- CMakeLists中 `set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../AirSim-v.1.2.2/)`需要根据实际路径修改；

编译完成后，可以运行`airsim_node`这个节点，airsim_settings_parser.cpp主要是用settings
=======
# airsim-navigation
location and navigation by car in airsim
>>>>>>> 1145f15f31c6561de1d2a2bdd62a81893d970f00
