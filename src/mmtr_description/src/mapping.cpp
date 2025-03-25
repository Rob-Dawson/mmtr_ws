#include "mapping.hpp"
using namespace std::placeholders;
namespace robot_mapping
{

Pose coordToPose(double px, double py, const nav_msgs::msg::MapMetaData &map_info)
{
    Pose pose;
    pose.x = std::round((px-map_info.origin.position.x) / map_info.resolution);
    pose.y = std::round((py -map_info.origin.position.y) / map_info.resolution);

    return pose;
}

bool poseOnMap(const Pose &pose, nav_msgs::msg::MapMetaData &map_info)
{
    return pose.x <static_cast<int>(map_info.width) && pose.x >= 0 && 
            pose.y <static_cast<int>(map_info.height) && pose.y >=0;
}

unsigned int poseToCell(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info)
{
    return map_info.width * pose.y + pose.x;
}


Occupancy_Mapping::Occupancy_Mapping(const std::string &name) : Node(name)
{
    declare_parameter<double>("width", 50.0);
    declare_parameter<double>("height", 50.0);
    declare_parameter<double>("resolution", 0.1);

    double width = get_parameter("width").as_double();
    double height = get_parameter("height").as_double();
    map.info.resolution = get_parameter("resolution").as_double();
    map.info.height = std::round(height / map.info.resolution); 
    map.info.width = std::round(width / map.info.resolution);
    map.info.origin.position.x = -std::round(width/2.0);
    map.info.origin.position.y = -std::round(width/2.0);
    map.header.frame_id = "/odometry/filtered";
    map.data = std::vector<int8_t>(map.info.width * map.info.height, -1);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    scan_sub = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Occupancy_Mapping::scanCallback, this, _1));

    mapPub = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
    timer = create_wall_timer(std::chrono::seconds(1), std::bind(&Occupancy_Mapping::timeCallback, this));


}

// void Occupancy_Mapping::odomCallback(const nav_msgs::msg::Odometry &odom)
// {
//     geometry_msgs::msg::TransformStamped t;
//     try 
//     {
//         t = tf_buffer
//     }
// }


void Occupancy_Mapping::scanCallback(const sensor_msgs::msg::LaserScan &scan)
{
    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer->lookupTransform(map.header.frame_id, scan.header.frame_id, tf2::TimePointZero);

    }
    catch(const tf2::TransformException &exec)
    {
        RCLCPP_ERROR(get_logger(), "Unable to transform between /odom and /base_footprint");
        return;
    }
    Pose robotPose = coordToPose(t.transform.translation.x, t.transform.translation.y, map.info);
    if(!poseOnMap(robotPose, map.info))
    {
        RCLCPP_ERROR(get_logger(), "Robot outside map");
        return;
    }
    
    unsigned int robotCell = poseToCell(robotPose, map.info);
    map.data.at(robotCell) = 100;
}

void Occupancy_Mapping::timeCallback()
{
    map.header.stamp = get_clock()->now();
    mapPub->publish(map);
}
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto map = std::make_shared<robot_mapping::Occupancy_Mapping>("occupancy_map");
    rclcpp::spin(map);
    rclcpp::shutdown();
}