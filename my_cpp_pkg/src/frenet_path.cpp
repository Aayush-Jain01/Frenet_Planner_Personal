#include "../include/frenet_optimal_trajectory.hpp"
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
//#include <geometry_msgs/Twist.h>
#include <geometry_msgs/msg/Twist.hpp>
#include <time.hpp>
#include <nav_msgs/msg/Path.hpp>
#include <tf2_ros/transform_datatypes.h>
#include <utility>
#include <ros/console.hpp>
#include <vector>
#include<fstream>

class Frenet_Path : public rclcpp::Node {
public:  
    Frenet_Path()
    : Node("frenet_path")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/frenet_path", 1);
        auto path_msg = nav_msgs::msg::Path();

    }
    void publishPath(nav_msgs::msg::Path &path_msg, FrenetPath &path, vecD &rk, vecD &ryaw, double &c_speed,
				 double &c_d, double &c_d_d)
    {
	    geometry_msgs::msg::PoseStamped loc;
	    double delta_theta, yaw;
	    vecD x_vec = path.get_x();
	    vecD y_vec = path.get_y();
	    for (unsigned int i = 0; i < path.get_x().size(); i++)
	    {
		    loc.pose.position.x = x_vec[i];
		    loc.pose.position.y = y_vec[i];
		    delta_theta = atan(c_d_d / ((1 - rk[i] * c_d) * c_speed));
		    yaw = delta_theta + ryaw[i];
		    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw); // roll , pitch = 0
		    q.normalize();
		    quaternionTFToMsg(q, loc.pose.orientation);
		    path_msg.poses.push_back(loc);
    	}
        path_msg.header.frame_id = "map";
        publisher_->publish(path_msg);
    }

private:  
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;            

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Frenet_Path>());
  return 0;
}

