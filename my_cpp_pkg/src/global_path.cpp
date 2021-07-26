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


//DOUBT - HOW TO TAKE IN rx, ry ? SInce they are part of the main code in frenet_ros_obst they are not a problem
//Probably  declare them as vectors in public:



class Global_Path : public rclcpp::Node {
public:  
    Global_Path()
    : Node("global_path") 
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);
        auto global_path_msg = nav_msgs::msg::Path();

    }
    
    //std::vector<> rx, ry;

    global_path_msg.header.frame_id = "map";

	global_path_msg.poses.resize(rx.size());
		// Global path pushed into the message
	for (i = 0; i < rx.size(); i++)
		{
			geometry_msgs::msg::PoseStamped loc;
			loc.pose.position.x = rx[i];
			loc.pose.position.y = ry[i];
			global_path_msg.poses[i] = loc;
		}
	if (false)
		{
			plt::ion();
			plt::show();
			plt::plot(lp.get_x(), lp.get_y());
			plt::pause(0.001);
			plt::plot(rx, ry);
			plt::pause(0.001);
		}
    publisher_->publish(global_path_msg)
private:  
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;            

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Global_Path>());
  return 0;
}
