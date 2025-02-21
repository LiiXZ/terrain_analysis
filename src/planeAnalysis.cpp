#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class PlaneAnalysis
{
public:
    PlaneAnalysis() : nh("~")
    {
        // 设置参数默认值
        nh.param("intensity_threshold", intensity_threshold_, 0.5);  //反射强度阈值
        nh.param("plane_angle_threshold", plane_angle_threshold_, 30.0);  //坡度地形阈值
        nh.param("remain_points_threshold", remain_points_threshold_, 200);  //


        // 订阅点云话题
        cloud_subscriber_ = nh.subscribe("/terrain_map", 1, &PlaneAnalysis::cloudCallback, this);

        // 发布两个点云话题
        obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 1);
        ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 1);

        plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/plane_points", 1);
        no_plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/no_plane_points", 1);

        my_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/terrain_points", 1);

    }

    void get_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int threshold, pcl::PointCloud<pcl::PointXYZI>::Ptr plane, pcl::PointCloud<pcl::PointXYZI>::Ptr no_plane)
    {
        while (cloud->points.size() >= threshold)
        {
            // 创建分割对象 -- 检测平面参数
            // pcl::search::KdTree<pcl::PointXYZ>::Ptr search(new pcl::search::KdTree<pcl::PointXYZ>);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 存储输出的模型的系数
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // 存储内点，使用的点
            pcl::SACSegmentation<pcl::PointXYZI> seg;
            // 可选设置
            seg.setOptimizeCoefficients(false);
            // 必须设置
            seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型类型，检测平面
            seg.setMethodType(pcl::SAC_RANSAC);    // 设置方法【聚类或随机样本一致性】
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.01);
            // seg.setSamplesMaxDist(0.1, search);
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients); // 分割操作
            // search->setInputCloud(cloud);
            if (inliers->indices.size() == 0)
            {
                PCL_ERROR("Could not estimate a planar model for the given dataset.");
                return;
            }

            // 平面参数
            // std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
            //           << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
            // 检查该平面与XY平面的角度（假设Z轴为向上方向）
            double angle = std::acos(coefficients->values[2] / std::sqrt(coefficients->values[0]*coefficients->values[0] +
                                                                        coefficients->values[1]*coefficients->values[1] +
                                                                        coefficients->values[2]*coefficients->values[2]));
            // std::cout << "该平面的夹角angle: " << angle << std::endl;
            for (size_t i = 0; i < inliers->indices.size(); ++i) {
                pcl::PointXYZI point = cloud->points[inliers->indices[i]];
                if (angle < plane_angle_threshold_ * M_PI / 180.0 || (angle > M_PI / 2 && (M_PI - angle) < plane_angle_threshold_ * M_PI / 180.0)) {
                    // std::cout << "该平面的夹角angle: " << angle << std::endl;
                    point.intensity = 0.001;
                    plane->points.push_back(point);
                } else {
                    no_plane->points.push_back(point);
                }
            }

            // 提取平面
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
            extract.filter(*output); // 提取对于索引的点云 内点

            // // 移去平面局内点，提取剩余点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZI>);
            extract.setNegative(true);
            extract.filter(*cloud_other);
            cloud = cloud_other;
        }
        *no_plane = *no_plane + *cloud;
        std::cout << "cloud->points.size(): " << cloud->points.size() << std::endl;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg)
    {
        // 将ROS消息转换为PCL点云格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_cloud_msg, *cloud);

        // 创建条件过滤器，根据反射强度阈值分割点云
        pcl::PassThrough<pcl::PointXYZI> condrem;
        condrem.setInputCloud(cloud);
        condrem.setFilterFieldName("intensity");
        condrem.setKeepOrganized(true);
        condrem.setFilterLimits(0, intensity_threshold_);

        // 分割点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr plane(new pcl::PointCloud<pcl::PointXYZI>);    // 坡度点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_plane(new pcl::PointCloud<pcl::PointXYZI>); // 非坡度点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr my_terrain_analysis(new pcl::PointCloud<pcl::PointXYZI>); // 重赋值后的点云


        condrem.filter(*ground);

        condrem.setNegative(true);
        condrem.filter(*obstacle);

        //--------在障碍物点云中，拟合平面，依据点云数量和地面夹角筛选可通行平面，作为可通行点云并发布----------//
        ros::Time start_time = ros::Time::now();
        get_plane(cloud, remain_points_threshold_, plane, no_plane);
        // 获取结束时间
        ros::Time end_time = ros::Time::now();
        double duration = (end_time - start_time).toSec();
        ROS_INFO("执行时间: %.4f 秒", duration);

        // 将PCL点云再转换为ROS消息并发布
        sensor_msgs::PointCloud2 plane_msg, no_plane_msg, my_terrain_analysis_msg;
        pcl::toROSMsg(*plane, plane_msg);
        pcl::toROSMsg(*no_plane, no_plane_msg);
        plane_msg.header = input_cloud_msg->header;
        no_plane_msg.header = input_cloud_msg->header;

        plane_pub_.publish(plane_msg);
        no_plane_pub_.publish(no_plane_msg);

        // 发布分割后的点云到对应的话题
        obstacle_pub_.publish(obstacle);
        ground_pub_.publish(ground);
        //发布重赋值后的点云
        *my_terrain_analysis = *plane + *no_plane;
        pcl::toROSMsg(*my_terrain_analysis, my_terrain_analysis_msg);
        my_terrain_analysis_msg.header = input_cloud_msg->header;
        my_pub_.publish(my_terrain_analysis_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_subscriber_;
    ros::Publisher obstacle_pub_;
    ros::Publisher ground_pub_;
    ros::Publisher no_plane_pub_;
    ros::Publisher plane_pub_;
    ros::Publisher my_pub_;

    double intensity_threshold_;
    double plane_angle_threshold_;
    int remain_points_threshold_;
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "plane_analysis_node");
    PlaneAnalysis plane_analysis;
    ros::spin();
    return 0;
}
