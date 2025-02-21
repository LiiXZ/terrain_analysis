# terrain_analysis

功能包简介：

1. 订阅点云话题，根据点云的局部相对高度判断是否为可行区域
2. 将点云的局部相对高度重赋值为点云的反射强度，并发布
3. planeAnalysis使用了RANSIC平面拟合，环境复杂时频率较低

## 使用方法

### 下载并编译功能包

```bash
cd catkin_ws/src
git clone https://github.com/LiiXZ/terrain_analysis.git
cd ..
catkin_make
```

### 启动节点并预览

```bash
roslaunch terrain_analysis terrain_analysis.launch
```

### 使用

修改 `launch/terrain_analysis.launch` 中的点云和定位参数即可：

```xml
<arg name="points_topic" default="/rslidar_points" />
<arg name="state_topic" default="/us_liorf_localization/mapping/base_odometry" />
```
