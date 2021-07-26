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

class FrenetClass : public rclcpp::Node {
public:  
    FrenetClass()
    : Node("frenet_class")
    {
        publisher_frenet_path = this->create_publisher<nav_msgs::msg::Path>("/frenet_path", 1);
        //auto path_msg = nav_msgs::msg::Path();
        publisher_global_path = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);
        //auto global_path_msg = nav_msgs::msg::Path();
        publisher_target_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        //auto vel = geometry_msgs::msg::Twist();
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/base_pose_ground_truth", 10, odom_callback);
        footprint_sub = this->create_subscription<geometry_msgs::msg::PolygonStamped>("/move_base/local_costmap/footprint", 10, footprint_callback);
        costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/move_base/local_costmap/costmap", 10000, costmap_callback);

    }

private:  

   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_frenet_path;            
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_global_path;
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_target_vel;
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
   rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub;
   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;

   void footprint_callback(const geometry_msgs::msg::PolygonStampedConstPtr &p)
   {
	    ::footprint = *p;
   }

// accesses the odometry data
   void odom_callback(const nav_msgs::msg::odometry::ConstPtr &msg)
   {
	    ::odom = *msg;
   }
   void costmap_callback(const nav_msgs::msg::OccupancyGrid::ConstPtr &occupancy_grid)
   {
	    cost_count++;
	    //double startTime = omp_get_wtime();
	    unsigned int height, width;
	    ::cmap = *occupancy_grid;
	    ob_x.clear();
	    ob_y.clear();
	    geometry_msgs::msg::Pose origin = occupancy_grid->info.origin;

	    double startTime1 = omp_get_wtime();
	    vector<pair<double, double>> ob1;
    #pragma omp parallel for collapse(2)
	    for (width = 0; width < occupancy_grid->info.width; ++width)
	    {
		    for (height = 0; height < occupancy_grid->info.height; ++height)
		    {
			    if (occupancy_grid->data[height * occupancy_grid->info.width + width] > 0)
			    {
    #pragma omp critical
	    			{
					    ob1.emplace_back(width * occupancy_grid->info.resolution + occupancy_grid->info.resolution / 2 + origin.position.x, height * occupancy_grid->info.resolution + occupancy_grid->info.resolution / 2 + origin.position.y);
					//     ob_x.emplace_back(width * occupancy_grid->info.resolution + occupancy_grid->info.
					// resolution / 2 + origin.position.x);
					// ob_y.emplace_back(height * occupancy_grid->info.resolution + occupancy_grid->info.resolution
					// / 2 + origin.position.y);
				    }
			    }
		    }
	    }

	sort(ob1.begin(), ob1.end());
	ob_x.resize(ob1.size());
	ob_y.resize(ob1.size());
	for (long i = 0; i < ob1.size(); i++)
	{
		ob_x[i] = ob1[i].first;
		ob_y[i] = ob1[i].second;
	}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrenetClass>());
  return 0;
}

