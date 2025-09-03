#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>

#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

using namespace cv;
using namespace std;

class HeightmapSpawner : public rclcpp::Node {
public:
    HeightmapSpawner()
    : Node("heightmap_spawner") {
        this->declare_parameter("save_path", "/tmp");
        this->declare_parameter("height", 0.5);
        this->declare_parameter("use_median_filtering", true);
        this->declare_parameter("use_color_inverse", true);
        this->declare_parameter("low_thresh", 0);
        this->declare_parameter("high_thresh", 100);

        // Set QoS to match map topics (transient local)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        // Create a subscription to the /map topic
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", qos, std::bind(&HeightmapSpawner::map_callback, this, std::placeholders::_1));

        // Log the actual topic name after remapping
        RCLCPP_INFO(this->get_logger(), "Heightmap spawner started. Waiting for map on topic '%s'...", map_subscription_->get_topic_name());
    }

    bool is_done() {
        return done_;
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (done_) return; // Process only the first valid map

        // Add a check to ensure the map is not empty
        if (msg->info.width == 0 || msg->info.height == 0) {
            RCLCPP_WARN(this->get_logger(), "Received an empty or invalid map, ignoring.");
            return;
        }


        auto height = this->get_parameter("height").as_double();
        auto low_thresh = this->get_parameter("low_thresh").as_int();
        auto high_thresh = this->get_parameter("high_thresh").as_int();

        RCLCPP_INFO(this->get_logger(), "Received a map");

        Mat map_image = occupancy_grid_to_image(msg, low_thresh, high_thresh);

        float resolution = msg->info.resolution;
        float size_x = resolution * msg->info.width;
        float size_y = resolution * msg->info.height;

        RCLCPP_INFO(this->get_logger(), "Map size: %d x %d", msg->info.width, msg->info.height);
        RCLCPP_INFO(this->get_logger(), "Map resolution: %f", resolution);

        process_image(map_image);

        float origin_x = msg->info.origin.position.x + size_x / 2;
        float origin_y = msg->info.origin.position.y + size_y / 2;

        create_entity(size_x, size_y, origin_x, origin_y, height);

        // Mark as done to avoid processing further maps
        RCLCPP_INFO(this->get_logger(), "Heightmap spawned successfully. Shutting down node.");
        done_ = true;
    }

    Mat occupancy_grid_to_image(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid, int low_thresh, int high_thresh) {
        int width = grid->info.width;
        int height = grid->info.height;
        Mat image(height, width, CV_8UC1);

        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                int index = (height - i - 1) * width + j;
                int value = grid->data[index];
                if (value == -1) {
                    image.at<uchar>(i, j) = 0; // Unknown (black)
                } else if (value >= high_thresh) {
                    image.at<uchar>(i, j) = 0; // Wall (black)
                } else if (value <= low_thresh) {
                    image.at<uchar>(i, j) = 255; // Floor (white)
                } else {
                    // Map linearly from 0-100 to 0-255
                    float scale = 255.0 / 100.0;
                    image.at<uchar>(i, j) = value * scale;
                }
            }
        }
        return image;
    }

    Mat process_image(Mat source_image) {
        auto save_path = this->get_parameter("save_path").as_string();
        auto use_median_filtering = this->get_parameter("use_median_filtering").as_bool();
        auto use_color_inverse = this->get_parameter("use_color_inverse").as_bool();

        if (use_median_filtering) {
            medianBlur(source_image, source_image, 5);
        }

        if (use_color_inverse) {
            bitwise_not(source_image, source_image);
        }

        Mat resized_image;
        int max_size = std::max(source_image.cols, source_image.rows);
        int size = 1;
        while (size <= max_size) {
            size *= 2;
        }
        size += 1; // Ensure size is (2^n) + 1 for Gazebo compatibility
        resize(source_image, resized_image, Size(size, size), 0, 0, INTER_NEAREST);

        save_path += "/prepared_map.png";
        imwrite(save_path, resized_image);
        return resized_image;
    }

    void create_entity(float size_x, float size_y, float origin_x, float origin_y, float height) {
        std::string save_path = this->get_parameter("save_path").as_string();
        std::string entity_str = R"(<?xml version="1.0" ?>
            <sdf version='1.7'>
            <model name="map">
            <static>true</static>
            <link name="link">
                <visual name="visual">
                <geometry>
                    <heightmap>
                    <use_terrain_paging>true</use_terrain_paging>
                    <texture>
                        <diffuse>package://heightmap_spawner/textures/white.png</diffuse>
                        <normal>package://heightmap_spawner/textures/white.png</normal>
                        <size>1</size>
                    </texture>
                    <texture>
                        <diffuse>package://heightmap_spawner/textures/gray.png</diffuse>
                        <normal>package://heightmap_spawner/textures/white.png</normal>
                        <size>1</size>
                    </texture>
                    <blend>
                        <min_height>0</min_height>
                        <fade_dist>0.01</fade_dist>
                    </blend>
                    <blend>
                        <min_height>2</min_height>
                        <fade_dist>0.01</fade_dist>
                    </blend>
                    <uri>file://)";
        entity_str += save_path;
        entity_str += R"(/prepared_map.png</uri>
                    <size>)";
        entity_str += std::to_string(size_x);
        entity_str += " ";
        entity_str += std::to_string(size_y);
        entity_str += " ";
        entity_str += std::to_string(height);
        entity_str += R"(</size>
                    <pos>)";
        entity_str += std::to_string(origin_x);
        entity_str += " ";
        entity_str += std::to_string(origin_y);
        entity_str += R"( 0</pos>
                    <sampling>2</sampling>
                    </heightmap>
                </geometry>
                </visual>
                <collision name="collision">
                <pose>)";
        entity_str += std::to_string(origin_x);
        entity_str += " ";
        entity_str += std::to_string(origin_y);
        entity_str += R"( 0 0 0 0</pose> // bug hack: Gazebo doesn't support pos for heightmap collision
                <geometry>
                    <heightmap>
                    <uri>file://)";
        entity_str += save_path;
        entity_str += R"(/prepared_map.png</uri>
                    <size>)";
        entity_str += std::to_string(size_x);
        entity_str += " ";
        entity_str += std::to_string(size_y);
        entity_str += " ";
        entity_str += std::to_string(height);
        entity_str += R"(</size>
                    <pos>0 0 0</pos>
                    <sampling>2</sampling>
                    </heightmap>
                </geometry>
                </collision>
            </link>
            </model>
            </sdf>)";

        createEntityFromStr(entity_str);

        RCLCPP_INFO(this->get_logger(), "Entity created at (%f, %f)", origin_x, origin_y);
    }

    void createEntityFromStr(const std::string& modelStr, const std::string& worldName = "default", int timeout = 5000) {
        while (true) {
            bool result;
            gz::msgs::EntityFactory req;
            gz::msgs::Boolean res;
            req.set_sdf(modelStr);

            bool executed = gz_node_.Request("/world/" + worldName + "/create", req, timeout, res, result);
            if (executed) {
                if (result) {
                    std::cout << "Entity was created: [" << res.data() << "]" << std::endl;
                    break;
                } else {
                    std::cout << "Service call failed. Retrying..." << std::endl;
                }
            } else {
                std::cerr << "Service call timed out. Retrying..." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    gz::transport::Node gz_node_;
    bool done_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightmapSpawner>();
    // Spin until the node is done or shutdown is requested
    while (rclcpp::ok() && !node->is_done()) {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
