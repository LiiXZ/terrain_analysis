#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

static std::string POINTS_TOPIC; //将点云数据话题写为参数在launch中加载
static std::string STATE_TOPIC;  //将状态估计信息话题写为参数在launch中加载

const double PI = 3.1415926;
//机器人周围的点云滤波范围
double disX = 1.0;
double disY = 1.0;

double scanVoxelSize = 0.05; // 点云下采样
double decayTime = 2.0;      // 点云时间差阈值 大于的不会被处理
double noDecayDis = 4.0;     // 点云距离阈值 小于该阈值不考虑时间差
double clearingDis = 8.0;    // 该距离外的点会被清除
bool clearingCloud = false;  // 清除距离外的点云
bool useSorting = true;
double quantileZ = 0.25; // 考虑地面附近高程最小值会改变
bool considerDrop = false;
bool limitGroundLift = false;
double maxGroundLift = 0.15;
bool clearDyObs = false;
double minDyObsDis = 0.3;
double minDyObsAngle = 0;
double minDyObsRelZ = -0.5;
double absDyObsRelZThre = 0.2;
double minDyObsVFOV = -16.0;
double maxDyObsVFOV = 16.0;
int minDyObsPointNum = 1;
bool noDataObstacle = false;
int noDataBlockSkipNum = 0;
int minBlockPointNum = 10;        // 计算有效高程的最小点云数量
double vehicleHeight = 0.8;       // 车辆的高度
int voxelPointUpdateThre = 100;   // 需要处理的体素网格点云数量最小值
double voxelTimeUpdateThre = 2.0; // 更新体素网格时间阈值
double minRelZ = -1.5;            // 点云处理的最小高度
double maxRelZ = 0.2;             // 点云处理的最大高度
double disRatioZ = 0.2;           // 点云处理的高度与距离的比例

// terrain voxel parameters
float terrainVoxelSize = 1.0; // 地形的分辨率
int terrainVoxelShiftX = 0;   // 地形中心与车体的偏移量
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21; // 地形的宽度
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;  //10
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth; // 地形体素点云数量 21x21

// planar voxel parameters  平面的体素参数
float planarVoxelSize = 0.2;     // 平面的分辨率
const int planarVoxelWidth = 51; // 平面的宽度
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth; // 每个平面的体素网格数量

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];


int terrainVoxelUpdateNum[terrainVoxelNum] = {0};    // 体素网格的更新次数
float terrainVoxelUpdateTime[terrainVoxelNum] = {0}; // 体素网格的更新时间
float planarVoxelElev[planarVoxelNum] = {0};
int planarVoxelEdge[planarVoxelNum] = {0};
int planarVoxelDyObs[planarVoxelNum] = {0};
vector<float> planarPointElev[planarVoxelNum];

double laserCloudTime = 0;
string laserFrameId = "";
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;
int noDataInited = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0; // odom或map中的机器人三维坐标
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter; // 使用pcl中的体素网格类，实例化出一个存放点云数据的下采样滤波器对象

// state estimation callback function 状态估计回调函数
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)) // 将四元数转化为RPY
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  // vehicleX = odom->pose.pose.position.x;
  // vehicleY = odom->pose.pose.position.y;
  // vehicleZ = odom->pose.pose.position.z;

  vehicleX = 0;
  vehicleY = 0;
  vehicleZ = 0;

  sinVehicleRoll = sin(vehicleRoll);
  cosVehicleRoll = cos(vehicleRoll);
  sinVehiclePitch = sin(vehiclePitch);
  cosVehiclePitch = cos(vehiclePitch);
  sinVehicleYaw = sin(vehicleYaw);
  cosVehicleYaw = cos(vehicleYaw);

  if (noDataInited == 0)
  {
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    noDataInited = 1;
  }
  if (noDataInited == 1)
  {
    float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                     (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
    if (dis >= noDecayDis)
      noDataInited = 2;
  }
}

// registered laser scan callback function 雷达数据回调函数（根据高度阈值和距离阈值裁剪得到局部点云）
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2)
{
  laserCloudTime = laserCloud2->header.stamp.toSec();
  laserFrameId = laserCloud2->header.frame_id;
  // std::cout << "laserCloudTime: " << laserCloudTime << std::endl;
  if (!systemInited)
  {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud); // 格式转换

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  
  for (int i = 0; i < laserCloudSize; i++)
  {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + // 计算每个点云与机器人的距离
                     (pointY - vehicleY) * (pointY - vehicleY));
    if (pointZ - vehicleZ > minRelZ - disRatioZ * dis && // 如果该点云在计算区域内
        pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1) && !(abs(pointX - vehicleX) < disX && abs(pointY - vehicleY) < disY))
    {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime; // 将该点云的反射强度设置为激光数据的时间差
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true;
}

// joystick callback function  手柄的回调函数
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->buttons[5] > 0.5)
  {
    noDataInited = 0;
    clearingCloud = true;
  }
}

// cloud clearing callback function
void clearingHandler(const std_msgs::Float32::ConstPtr &dis)
{
  noDataInited = 0;
  clearingDis = dis->data;
  clearingCloud = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terrainAnalysis");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("disX", disX);
  nhPrivate.getParam("disY", disY);

  nhPrivate.getParam("scanVoxelSize", scanVoxelSize);
  nhPrivate.getParam("decayTime", decayTime);
  nhPrivate.getParam("noDecayDis", noDecayDis);
  nhPrivate.getParam("clearingDis", clearingDis);
  nhPrivate.getParam("useSorting", useSorting);
  nhPrivate.getParam("quantileZ", quantileZ);
  nhPrivate.getParam("considerDrop", considerDrop);
  nhPrivate.getParam("limitGroundLift", limitGroundLift);
  nhPrivate.getParam("maxGroundLift", maxGroundLift);
  nhPrivate.getParam("clearDyObs", clearDyObs);
  nhPrivate.getParam("minDyObsDis", minDyObsDis);
  nhPrivate.getParam("minDyObsAngle", minDyObsAngle);
  nhPrivate.getParam("minDyObsRelZ", minDyObsRelZ);
  nhPrivate.getParam("absDyObsRelZThre", absDyObsRelZThre);
  nhPrivate.getParam("minDyObsVFOV", minDyObsVFOV);
  nhPrivate.getParam("maxDyObsVFOV", maxDyObsVFOV);
  nhPrivate.getParam("minDyObsPointNum", minDyObsPointNum);
  nhPrivate.getParam("noDataObstacle", noDataObstacle);
  nhPrivate.getParam("noDataBlockSkipNum", noDataBlockSkipNum);
  nhPrivate.getParam("minBlockPointNum", minBlockPointNum);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);
  nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("disRatioZ", disRatioZ);

  nhPrivate.getParam("state_topic", STATE_TOPIC);  //状态估计话题
  nhPrivate.getParam("points_topic", POINTS_TOPIC);  //点云数据话题

  ros::Subscriber subOdometry =
      nh.subscribe<nav_msgs::Odometry>(STATE_TOPIC, 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>( // 雷达发布的pcl格式的点云数据，话题为
      POINTS_TOPIC, 5, laserCloudHandler);

  ros::Subscriber subJoystick =
      nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);

  ros::Subscriber subClearing =
      nh.subscribe<std_msgs::Float32>("/map_clearing", 5, clearingHandler); // 清空某范围内的点云地图

  ros::Publisher pubLaserCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/terrain_map", 2);

  for (int i = 0; i < terrainVoxelNum; i++)
  {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  int terraionPresstimes = 0;
  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize); // 设置体素网格的格子大小

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();
    terraionPresstimes++;
    if(terraionPresstimes == 10)
    {
    	for (int i = 0; i < terrainVoxelNum; i++)
      {
          terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      }
	    terraionPresstimes = 0;
    }

    if (newlaserCloud)
    { // 新点云数据传入时
      newlaserCloud = false;

      // terrain voxel roll over 滚动局部地形体素点云
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize)
      { // 当车辆偏移过大时滚动地图
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[indY]->clear();
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleX - terrainVoxelCenX > terrainVoxelSize)
      {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                            indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
              ->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX +
                                (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      while (vehicleY - terrainVoxelCenY > terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX +
                            (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
              ->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // stack registered laser scans
      pcl::PointXYZI point;
      int laserCloudCropSize = laserCloudCrop->points.size(); // 点云数据在call_back中已被裁剪过了
      for (int i = 0; i < laserCloudCropSize; i++)
      {                                    // 地形体素点云地图的构建
        point = laserCloudCrop->points[i]; // 处理裁剪后的局部点云地图

        int indX = int((point.x - vehicleX + terrainVoxelSize / 2) / // 计算该点云属于局部体素网格中的序号
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;
        int indY = int((point.y - vehicleY + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;

        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
            indY < terrainVoxelWidth)
        {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point); // 按照坐标序号插入到地形体素点云中
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
        }
      }
      // 在遍历每一个网格时，仅对格内需要更新的点云数量大于 voxelPointUpdateThre 的网格进行处理。
      // 并且与上一次该网格处理的时间差大于 voxelTimeUpdateThre
      // terrainVoxelNum=441，表明该体素网格为21x21的单层网格
      for (int ind = 0; ind < terrainVoxelNum; ind++)
      {
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
            laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                voxelTimeUpdateThre ||
            clearingCloud)
        {                                                                                     // 仅对体素网格内点云数量大于阈值，且与上一次更新时间差较大的网格进行更新
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind]; // 依次对每个体素网格中的点云进行处理

          laserCloudDwz->clear();
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
          downSizeFilter.filter(*laserCloudDwz); // 降采样

          terrainVoxelCloudPtr->clear();
          int laserCloudDwzSize = laserCloudDwz->points.size();
          for (int i = 0; i < laserCloudDwzSize; i++)
          {
            point = laserCloudDwz->points[i]; // 对每个网格中降采样后的点云进行处理
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));
            if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
                (laserCloudTime - systemInitTime - point.intensity <
                     decayTime ||
                 dis < noDecayDis) &&
                !(dis < clearingDis && clearingCloud)) // 使用多个条件判断是否符合有效点云的条件
            {
              terrainVoxelCloudPtr->push_back(point);
            }
          }

          terrainVoxelUpdateNum[ind] = 0;
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
        }
      }

      terrainCloud->clear();
      for (int indX = terrainVoxelHalfWidth - 5;
           indX <= terrainVoxelHalfWidth + 5; indX++)
      {
        for (int indY = terrainVoxelHalfWidth - 5;
             indY <= terrainVoxelHalfWidth + 5; indY++)
        {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
      }

      // estimate ground and compute elevation for each point  估计地面并计算每个点的高程
      for (int i = 0; i < planarVoxelNum; i++) // 对每个网格进行处理（初始化网格参数）
      {
        planarVoxelElev[i] = 0;
        planarVoxelEdge[i] = 0;
        planarVoxelDyObs[i] = 0;
        planarPointElev[i].clear();
      }

      int terrainCloudSize = terrainCloud->points.size(); // 对机器人有效的局部点云信息
      for (int i = 0; i < terrainCloudSize; i++)
      {
        point = terrainCloud->points[i];

        int indX =
            int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;
        int indY =
            int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;

        if (point.x - vehicleX + planarVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0)
          indY--;

        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ)
        {
          for (int dX = -1; dX <= 1; dX++)
          {
            for (int dY = -1; dY <= 1; dY++)
            {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth)
              {
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY]
                    .push_back(point.z); // 存入高度信息
              }
            }
          }
        }

        if (clearDyObs)
        {
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth)
          {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            if (dis1 > minDyObsDis)
            {
              float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
              if (angle1 > minDyObsAngle)
              {
                float pointX2 =
                    pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                float pointY2 =
                    -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                float pointZ2 = pointZ1;

                float pointX3 =
                    pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                float pointY3 = pointY2;
                float pointZ3 =
                    pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                float pointX4 = pointX3;
                float pointY4 =
                    pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                float pointZ4 =
                    -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;
                if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV || fabs(pointZ4) < absDyObsRelZThre)
                {
                  planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
                }
              }
            }
            else
            {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
                  minDyObsPointNum;
            }
          }
        }
      }

      if (clearDyObs)
      {
        for (int i = 0; i < laserCloudCropSize; i++)
        {
          point = laserCloudCrop->points[i];

          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth)
          {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
            if (angle1 > minDyObsAngle)
            {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
            }
          }
        }
      }

      if (useSorting) // 在每个大体素网格中的平面中求解最小的高程
      {
        for (int i = 0; i < planarVoxelNum; i++)
        {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0)
          {
            sort(planarPointElev[i].begin(), planarPointElev[i].end()); // sort算法默认升序排列，此时最小值为第一个

            int quantileID = int(quantileZ * planarPointElevSize);
            if (quantileID < 0)
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;

            if (planarPointElev[i][quantileID] >
                    planarPointElev[i][0] + maxGroundLift &&
                limitGroundLift)
            {
              planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
            }
            else
            {
              planarVoxelElev[i] = planarPointElev[i][quantileID];
            }
          }
        }
      }
      else
      {
        for (int i = 0; i < planarVoxelNum; i++)
        {
          int planarPointElevSize = planarPointElev[i].size(); // 对2601中的每个体素网格求其中点云的最小高程
          if (planarPointElevSize > 0)
          {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++)
            { // 循环找到该体素网格中的最小高程
              if (planarPointElev[i][j] < minZ)
              {
                minZ = planarPointElev[i][j];
                minID = j;
              }
            }

            if (minID != -1)
            {
              planarVoxelElev[i] = planarPointElev[i][minID];
            }
          }
        }
      }

      terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++) // 根据前面的地形处理结果，在有效点云中可视化分割结果
      {
        point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ)
        {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth)
          {
            // 如果当前点云的高程比附近最小值大，小于车辆的高度，并且计算高程时的点云数量也足够多，就把当前点云放入到地形高程点云中。其中点云的强度为一个相对的高度
            if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                    minDyObsPointNum ||
                !clearDyObs)
            {
              float disZ =
                  point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
              if (considerDrop)
                disZ = fabs(disZ);    //考虑跌落时，地面一下的相对高度被赋值为正值，当做障碍物处理
              int planarPointElevSize =
                  planarPointElev[planarVoxelWidth * indX + indY].size();
              if (disZ >= 0 && disZ < vehicleHeight &&
                  planarPointElevSize >= minBlockPointNum)
              {
                terrainCloudElev->push_back(point);                              // 将分割后的点云存入地形点云中
                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ; // 反射强度为相对于局部最小高程的高度

                terrainCloudElevSize++;
              }
            }
          }
        }
      }

      if (noDataObstacle && noDataInited == 2)
      {
        for (int i = 0; i < planarVoxelNum; i++)
        {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize < minBlockPointNum)
          {
            planarVoxelEdge[i] = 1;
          }
        }

        for (int noDataBlockSkipCount = 0;
             noDataBlockSkipCount < noDataBlockSkipNum;
             noDataBlockSkipCount++)
        {
          for (int i = 0; i < planarVoxelNum; i++)
          {
            if (planarVoxelEdge[i] >= 1)
            {
              int indX = int(i / planarVoxelWidth);
              int indY = i % planarVoxelWidth;
              bool edgeVoxel = false;
              for (int dX = -1; dX <= 1; dX++)
              {
                for (int dY = -1; dY <= 1; dY++)
                {
                  if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                      indY + dY >= 0 && indY + dY < planarVoxelWidth)
                  {
                    if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                        dY] < planarVoxelEdge[i])
                    {
                      edgeVoxel = true;
                    }
                  }
                }
              }

              if (!edgeVoxel)
                planarVoxelEdge[i]++;
            }
          }
        }

        for (int i = 0; i < planarVoxelNum; i++)
        {
          if (planarVoxelEdge[i] > noDataBlockSkipNum)
          {
            int indX = int(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;

            point.x =
                planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
            point.y =
                planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
            point.z = vehicleZ;
            point.intensity = vehicleHeight;

            point.x -= planarVoxelSize / 4.0;
            point.y -= planarVoxelSize / 4.0;
            terrainCloudElev->push_back(point);

            point.x += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.y += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.x -= planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);
          }
        }
      }

      clearingCloud = false;

      // publish points with elevation 发布带有高度信息的点云数据
      sensor_msgs::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
      terrainCloud2.header.frame_id = laserFrameId; //保持frame不变，为lidar_link，其与vehicle为静态变换
      pubLaserCloud.publish(terrainCloud2);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
