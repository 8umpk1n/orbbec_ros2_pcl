#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <tuple>
#include <cmath> // 用于数学计算
#include <array>
#include <algorithm>
#include <mutex>
#include <chrono>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace ColorUtils {
    struct HSV {
        float h, s, v;
    };

    HSV rgb_to_hsv(const pcl::PointXYZRGB& point) {
        HSV hsv;
        float r = point.r / 255.0f;
        float g = point.g / 255.0f;
        float b = point.b / 255.0f;

        float cmax = std::max({r, g, b});
        float cmin = std::min({r, g, b});
        float delta = cmax - cmin;

        // 计算色调
        if (delta == 0)
            hsv.h = 0;
        else if (cmax == r)
            hsv.h = 60 * fmod(((g - b) / delta), 6);
        else if (cmax == g)
            hsv.h = 60 * (((b - r) / delta) + 2);
        else
            hsv.h = 60 * (((r - g) / delta) + 4);

        if (hsv.h < 0) hsv.h += 360;

        // 计算饱和度
        hsv.s = (cmax == 0) ? 0 : (delta / cmax);

        // 计算明度
        hsv.v = cmax;

        return hsv;
    }
}

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("pcl_processor") {
        // 声明参数
        this->declare_parameter("axis_filters.z", std::vector<float>{0.0f, 1.0f});
        this->declare_parameter("axis_filters.x", std::vector<float>{-0.5f, 0.5f});
        this->declare_parameter("axis_filters.y", std::vector<float>{-0.5f, 0.5f});
        
        this->declare_parameter("color_thresholds.min_hue", 30.0f);
        this->declare_parameter("color_thresholds.max_hue", 100.0f);
        this->declare_parameter("color_thresholds.min_saturation", 0.2f);
        this->declare_parameter("color_thresholds.min_value", 0.15f);
        
        this->declare_parameter("clustering.tolerance", 0.01f);
        this->declare_parameter("clustering.min_cluster_size", 5000);
        this->declare_parameter("clustering.max_cluster_size", 25000);
        
        this->declare_parameter("process_interval", 5000);

        // 获取初始参数
        update_parameters();

        // 设置参数回调
        param_callback_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) {
                for (const auto &param : params) {
                    if (param.get_name() == "axis_filters.z" ||
                        param.get_name() == "axis_filters.x" ||
                        param.get_name() == "axis_filters.y") {
                        update_parameters();
                    }
                }
                return rcl_interfaces::msg::SetParametersResult().set__successful(true);
            });

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_registered/points", 
            10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                latest_cloud_ = msg; 
            });
        
        // 创建5秒定时器（5000ms）
        process_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            [this]() {
                sensor_msgs::msg::PointCloud2::SharedPtr current_cloud;
                {
                    std::lock_guard<std::mutex> lock(cloud_mutex_);
                    current_cloud = latest_cloud_;
                    latest_cloud_.reset(); // 清空已处理数据
                }
                
                if(current_cloud) {
                    process_pointcloud(current_cloud);
                }
                else {
                    RCLCPP_WARN(this->get_logger(), "No pointcloud received in last 5 seconds");
                }
            });
    }

private:
    void update_parameters() {
        auto z_filter = this->get_parameter("axis_filters.z").as_double_array();
        auto x_filter = this->get_parameter("axis_filters.x").as_double_array();
        auto y_filter = this->get_parameter("axis_filters.y").as_double_array();
        
        axis_filters_ = {
            std::tuple{"z", static_cast<float>(z_filter[0]), static_cast<float>(z_filter[1])},
            std::tuple{"x", static_cast<float>(x_filter[0]), static_cast<float>(x_filter[1])},
            std::tuple{"y", static_cast<float>(y_filter[0]), static_cast<float>(y_filter[1])}
        };

        min_hue_ = this->get_parameter("color_thresholds.min_hue").as_double();
        max_hue_ = this->get_parameter("color_thresholds.max_hue").as_double();
        min_saturation_ = this->get_parameter("color_thresholds.min_saturation").as_double();
        min_value_ = this->get_parameter("color_thresholds.min_value").as_double();
        
        cluster_tolerance_ = this->get_parameter("clustering.tolerance").as_double();
        min_cluster_size_ = this->get_parameter("clustering.min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("clustering.max_cluster_size").as_int();
        
        process_interval_ = this->get_parameter("process_interval").as_int();
    }

    void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto cloud = convert_to_pcl(msg);
        
        // 保存原始点云（可选）
        // save_to_file(cloud, "original.pcd");
        
        apply_axis_filters(cloud);
        save_to_file(cloud, "filtered.pcd");
        
        auto plant_cloud = extract_green_plants(cloud);
        auto clustered_cloud = cluster_filtering(plant_cloud);
        save_to_file(clustered_cloud, "final_result.pcd");
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convert_to_pcl(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
    {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg(*msg, *cloud);
        return cloud;
    }

    void apply_axis_filters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
        for (const auto& [axis, min, max] : axis_filters_) {
            pcl::PassThrough<pcl::PointXYZRGB> filter;
            filter.setInputCloud(cloud);
            filter.setFilterFieldName(axis);
            filter.setFilterLimits(min, max);
            filter.filter(*cloud);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_green_plants(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) 
    {
        auto result = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(*result),
            [this](const auto& point) {
                auto hsv = ColorUtils::rgb_to_hsv(point);
                return hsv.h >= min_hue_ && hsv.h <= max_hue_ &&
                       hsv.s >= min_saturation_ &&
                       hsv.v >= min_value_;
            });
        return result;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_filtering(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) 
    {
        auto result = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        
        // 创建KD树用于快速搜索
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);

        // 执行欧几里得聚类
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 合并有效簇的点云
        for (const auto& cluster : cluster_indices) {
            for (const auto& idx : cluster.indices) {
                result->push_back((*cloud)[idx]);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Found %zu clusters, remaining points: %zu",
                   cluster_indices.size(), result->size());
        return result;
    }

    void save_to_file(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                    const std::string& filename) 
    {
        try {
            // 保存 PCD 文件
            pcl::io::savePCDFileASCII(filename, *cloud);
            RCLCPP_INFO(this->get_logger(), "Successfully saved %zu points to %s",
                       cloud->size(), filename.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save %s: %s",
                        filename.c_str(), e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::mutex cloud_mutex_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_{nullptr};
    rclcpp::TimerBase::SharedPtr process_timer_;
    std::vector<std::tuple<std::string, float, float>> axis_filters_;
    float min_hue_, max_hue_, min_saturation_, min_value_;
    float cluster_tolerance_;
    int min_cluster_size_, max_cluster_size_;
    int process_interval_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}

