#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/utils.h>
namespace robot_mapping
{
    struct Pose
    {
        Pose()=default;
        Pose(const int px, const int py) : x(px), y(py){}
        int x;
        int y;
    };

    unsigned int poseToCell(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info);

    Pose coordToPose(double px, double py, const nav_msgs::msg::MapMetaData &map_info);

    bool poseOnMap(const Pose &pose, nav_msgs::msg::MapMetaData &map_info);

    class Occupancy_Mapping : public rclcpp::Node
    {
    public:
        Occupancy_Mapping(const std::string &name);

    private:
        void timeCallback();
        //This will need to be removed
        void scanCallback(const sensor_msgs::msg::LaserScan &scan);

        nav_msgs::msg::OccupancyGrid map;
        // rclcpp::Subscription<>
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

        std::unique_ptr<tf2_ros::Buffer>tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener>tf_listener{nullptr};
    };
}