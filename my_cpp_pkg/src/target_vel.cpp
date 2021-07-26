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


class Target_Vel : public rclcpp::Node {
public:  
    Target_Vel()
    : Node("target_vel")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        auto vel = geometry_msgs::msg::Twist();

    }

	vel.linear.x = bot_v;
	vel.linear.y = 0;
	vel.linear.z = 0;

    publisher_->publish(vel)
private:  
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;            

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Target_Vel>());
  return 0;
}

//Probably this function should be included in the class like in the other two? Not sure
auto calc_bot_v = [min_id, rk](vecD d, vecD s_d, vecD d_d) {
			return sqrt(pow(1 - rk[min_id] * d[d.size() / 2], 2) * pow(s_d[s_d.size() / 2], 2) +
						pow(d_d[d_d.size() / 2], 2));
		};

		// Next velocity along the path
		if (path.get_d().size() <= 1 || path.get_s_d().size() <= 1 || path.get_d_d().size() <= 1)
		{
			bot_v = sqrt(pow(1 - rk[min_id] * c_d, 2) * pow(c_speed, 2) + pow(c_d_d, 2));
		}
		else
		{
			if (STOP_CAR)
			{
				//cerr<< "hi" << endl;
				bot_v = calc_bot_v(path.get_d(), path.get_s_d(), path.get_d_d());
			}
			else
			{
				bot_v = sqrt(pow(1 - rk[min_id] * path.get_d()[1], 2) * pow(path.get_s_d()[1], 2) +
							 pow(path.get_d_d()[1], 2));
			}
		}
		if (STOP_CAR)
		{
			cerr << bot_v << endl;
		}