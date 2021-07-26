# <?xml version='2.0'?>
# <launch>

#  <arg name="base_global_planner" default="navfn/NavfnROS" />
# 	<!-- Frenet Planner -->

# 	<node pkg = "frenet_planner" exec = "frenet_planner" name = "frenet_planner" output = "screen" >
# 		<param from= "$(find frenet_planner)/cfg/frenet_planner.yaml"/>
#   </node>
  
  
#   <node pkg="laser_filters" exec ="scan_to_scan_filter_chain" output="screen" name="laser_filter_right">
#         <remap from="scan" to="prius/front_right_laser/scan" />
#         <remap from="scan_filtered" to="scan_filtered_right" />
        
#         <param from= "$(find frenet_planner)/cfg/laser_filter.yaml"/>
#   </node>

#   <node pkg="laser_filters" exec ="scan_to_scan_filter_chain" output="screen" name="laser_filter_left">
#     <remap from="scan" to="/prius/front_left_laser/scan" />
#     <remap from="scan_filtered" to="/scan_filtered_left" />
#     <param from= "$(find frenet_planner)/cfg/laser_filter.yaml"/>
# </node>
    

# 	<node pkg="move_base" exec ="move_base" name="move_base" output="screen">
#     <remap from="odom" to="base_pose_ground_truth" />
#     <remap from="cmd_vel" to="cmd_vel_with_linear" />
#     <param name="use_sim_time" value="true" />
#     <param name="base_global_planner" value="$(arg base_global_planner)" />
#     <!-- <param name="base_local_planner" value="$(arg base_local_planner)" />   comment these later -->
#     <!-- <rosparam file="$(find car_demo)/cfg/teb_local_planner_params.yaml" command="load"/>   comment these later -->
#     <param from="$(find frenet_planner)/cfg/costmap_common.yaml" namespace="global_costmap" />
#     <param from="$(find frenet_planner)/cfg/costmap_common.yaml" namespace="local_costmap" />
#     <param from="$(find frenet_planner)/cfg/costmap_local.yaml" />
#     <param from="$(find frenet_planner)/cfg/costmap_global.yaml" />
#   </node>

#   <!-- Run the tracker node -->
#   <!-- <node pkg= "tracking_control" name="tracker" type="pure_pursuit.py" output="screen"/> -->
  
#   <!-- Run the controller node --> 
#   <!-- <node pkg= "tracking_control" name="controller" type="PID_MIT_velocity_controller.py" output="screen"/> -->

# </launch>

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='frenet_planner',
      executable='temp_frenet_ros_obst',
      # parameters=[
      #   {'kj': 0.01},
      #   {'kt': 5},
      #   {'kd': 0.5},
      #   {'kd_v': 0.5},
      #   {'klat': 1.0},
      #   {'klon': 2.0},
      #   {'max_speed': 13.89},
      #   {'max_accel': 5.0},
      #   {'max_curvature': 2.0},
      #   {'max_road_width': 10.0},
      #   {'d_road_w': 1.0},
      #   {'dt': 0.2},
      #   {'maxt': 6.0},
      #   {'mint': 2.0},
      #   {'target_speed': 8.33},
      #   {'d_t_s': 1.389},
      #   {'n_s_sample': 1.5},
      #   {'robot_radius': 2.0},
      #   {'min_lat_vel': -1.0},
      #   {'max_lat_vel': 1.0},
      #   {'d_d_ns': 0.5},
      #   {'max_shift_d': 3},
      #   {'W_X': [38, 38, 38, 38, 38, 38, 38 , 38.21, 39.25, 42.56, 48.97, 61]},
      #   {'W_Y': [-57, -45, -32.0, -18.5, -12.0, 0.0, 12, 35, 42.89, 80.41, 131.48, 139.47]}
      # ],
      output='screen'
    )
  ])
 