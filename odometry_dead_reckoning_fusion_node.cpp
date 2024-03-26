#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "isaac_ros_visual_slam_interfaces/srv/set_odometry_pose.hpp"


class DeadReckoningFusionNode : public rclcpp::Node
{
public:
    DeadReckoningFusionNode()
    : Node("dead_reckoning_fusion_node")
    {
        // Declare parameters for the odometry topic names
        this->declare_parameter<std::string>("global_odometry_topic", "/odometry/global_vo");
        this->declare_parameter<std::string>("gps_odometry_topic", "/odometry/gps");
        // Declare parameter gps_covariance_threshold
        this->declare_parameter<double>("gps_covariance_threshold", 2.0);

        // Get the odometry topic names from the parameters
        this->get_parameter("global_odometry_topic", global_odometry_topic_);
        this->get_parameter("gps_odometry_topic", gps_odometry_topic_);
        this->get_parameter("gps_covariance_threshold", gps_covariance_threshold_);
        // Set odometry service name
        

        // Setup the odometry subscribers
        global_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            global_odometry_topic_, 10, std::bind(&DeadReckoningFusionNode::GlobalOdometryCallback, this, std::placeholders::_1));
        
        gps_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            gps_odometry_topic_, 10, std::bind(&DeadReckoningFusionNode::GpsOdometryCallback, this, std::placeholders::_1));

        // Setup the service client
        client_ = this->create_client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>("visual_slam/set_odometry_pose");
        // Send request until successful
        
        
        // Initialize timer with a dummy callback to avoid uninitialized warnings
        periodic_request_timer_ = this->create_wall_timer(std::chrono::seconds(35), std::bind(&DeadReckoningFusionNode::PeriodicRequestTimerCallback, this));
    }

private:
    std::string global_odometry_topic_;
    std::string gps_odometry_topic_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_odometry_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odometry_;
    rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>::SharedPtr client_;
    geometry_msgs::msg::PoseWithCovariance global_odometry_pose_;
    rclcpp::TimerBase::SharedPtr periodic_request_timer_;
    double gps_covariance_threshold_;
    double gps_covariance_running_avg_ = 0.0;
    double alpha_running_avg_ = 0.3;
    bool dead_reckoning_fusion_enabled_ = false;
    bool first_request_sent_ = false;

    void GlobalOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // get pose from odometry msg
        global_odometry_pose_.pose = msg->pose.pose;
        // global_odometry_pose_.covariance = msg->pose.covariance;

        // Print avg position covariance and heading covariance
        // RCLCPP_INFO(this->get_logger(), "Global Odometry Pose avg: %f", (global_odometry_pose_.covariance[0] + global_odometry_pose_.covariance[7] + global_odometry_pose_.covariance[14])/3.0);
        // RCLCPP_INFO(this->get_logger(), "Global Odometry Pose heading: %f", global_odometry_pose_.covariance[35]);

        // send first request
        if (!first_request_sent_) {
            send_request();
        }
    }

    void GpsOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get covariance from gps odometry msg [double[36]]
        // auto gps_covariance = msg->pose.covariance;

        std::array<double, 36> gps_covariance;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                if(msg->pose.covariance[i*6+j] != 0.0){
                    gps_covariance[i * 6 + j] = msg->pose.covariance[i*6+j];
                }
                else{
                    gps_covariance[i * 6 + j] = i == j ? 1.0 : 0;
                }
            }
        }
        global_odometry_pose_.covariance = gps_covariance;
        // Get average of the first 3 diagonal covariance values
        double gps_covariance_avg = (gps_covariance[0] + gps_covariance[7]) / 2.0;
        gps_covariance_running_avg_ = alpha_running_avg_ * gps_covariance_avg + (1 - alpha_running_avg_) * gps_covariance_running_avg_;
        // Print the average covariance value
        RCLCPP_INFO(this->get_logger(), "Average GPS covariance: %f", gps_covariance_avg);
        RCLCPP_INFO(this->get_logger(), "Running average GPS covariance: %f", gps_covariance_running_avg_);

        if ((gps_covariance_running_avg_ > gps_covariance_threshold_) && (!dead_reckoning_fusion_enabled_)) {

            dead_reckoning_fusion_enabled_ = true;
            // Cancel any existing timer
            periodic_request_timer_->cancel();
            // Send request to set odometry pose
            // send_request();
        } else if ((gps_covariance_running_avg_ <= gps_covariance_threshold_ ) && dead_reckoning_fusion_enabled_)
        {
          dead_reckoning_fusion_enabled_ = false;
          periodic_request_timer_->reset();
        }
        // If dead_reckoning fusion is not enabled, then we send the request every 10 seconds using the timer

        // if (!dead_reckoning_fusion_enabled_){
        //   // In this case, we send request every 20 seconds
        //   periodic_request_timer_->cancel(); // Cancel any existing timer
        //   // Send request to set odometry pose
        //   periodic_request_timer_ = this->create_wall_timer(std::chrono::seconds(20), [this]() { this->send_request(); });
        // }

        
        
    }

    void send_request()
    {

      RCLCPP_INFO(this->get_logger(), "Sending request to set odometry pose");
        // Wait for the service to be available
      while (!client_->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
              return;
          }
          RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
      }

      auto request = std::make_shared<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose::Request>();
      
      request->pose = global_odometry_pose_;
      // LOG THE POSE BEING SENT
      RCLCPP_INFO(this->get_logger(), "Sending pose: x=%f, y=%f, z=%f", request->pose.pose.position.x, request->pose.pose.position.y, request->pose.pose.position.z);

      // Now, send the request
      auto result = client_->async_send_request(request,
          std::bind(&DeadReckoningFusionNode::response_callback, this, std::placeholders::_1));
      
    }

    void response_callback(rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success) {
            first_request_sent_ = true;
            RCLCPP_INFO(this->get_logger(), "Request was successful.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Request failed.");
        }
    }

    void PeriodicRequestTimerCallback() {
      send_request();
    }



    // Existing methods (send_request and response_callback) remain unchanged
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeadReckoningFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}