//#include "../include/frenet_optimal_trajectory.hpp"
//#include <ros/console.h>
#include <bits/stdc++.h>
#include <algorithm>
#include <vector>
//#include <matplotlibcpp.hpp>
//namespace plt = matplotlibcpp;
int transform_count = 0;
#include "../include/frenet_optimal_trajectory.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <time.h>
#include <nav_msgs/msg/path.hpp>
#include <tf2/transform_datatypes.h>
#include <utility>
#include <vector>
#include<fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <omp.h>
#include <matplotlibcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
namespace plt = matplotlibcpp;



// calculates lateral paths using the sampling parameters passed
void FrenetPath::calc_lat_paths(double c_d, double c_d_d, double c_d_dd, double Ti, double di,
								double di_d)
{
	int n = 1 + Ti / DT;
	t.resize(n);
	d.resize(n);
	d_d.resize(n);
	d_dd.resize(n);
	d_ddd.resize(n);

	quintic lat_qp(c_d, c_d_d, c_d_dd, di, di_d, 0.0, Ti);
	for (int te = 0; te < n; te++)
	{
		t[te] = te * DT;
		d[te] = lat_qp.calc_point(te * DT);
		d_d[te] = lat_qp.calc_first_derivative(te * DT);
		d_dd[te] = lat_qp.calc_second_derivative(te * DT);
		d_ddd[te] = lat_qp.calc_third_derivative(te * DT);
	}
}

// calculates longitudnal paths  using quartic polynomial
// void FrenetPath::calc_lon_paths(double c_speed, double s0, double Ti, double tv)
// {
//   // Normal Implementation
//   quartic lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);  // s_dd is set to const. 0 (i.e. not being
//                                                   // sampled)
//   int size = t.size();
//   s.resize(size);
//   s_d.resize(size);
//   s_dd.resize(size);
//   s_ddd.resize(size);
//   int i = 0;
//   for(auto const te : t)
//   {
//     s[i] = (lon_qp.calc_point(te));
//     s_d[i] = (lon_qp.calc_first_derivative(te));
//     s_dd[i] = (lon_qp.calc_second_derivative(te));
//     s_ddd[i] = (lon_qp.calc_third_derivative(te));
//     ++i;
//   }

//   // https://www.geeksforgeeks.org/std-inner_product-in-cpp/
//   Js = inner_product(s_ddd.begin(), s_ddd.end(), s_ddd.begin(), 0);
//   double ds = pow((TARGET_SPEED - s_d.back()), 2);
//   cd = (KJ*Jp + KT*Ti + KD*d.back()*d.back());
//   cv = (KJ*Js + KT*Ti + KD_V*ds);
//   cf = (KLAT*cd + KLON*cv);
// }

// calculates longitudnal paths  using quintic polynomial
void FrenetPath::calc_lon_paths_quintic_poly(double c_speed, double s0, double Ti, double ts,
											 double tv)
{
	quintic lon_qp(s0, c_speed, 0.0, min(s0 + 15, 203.5), tv, 0.0, Ti); // s_dd is not being sampled
	int size = t.size();
	s.resize(size);
	s_d.resize(size);
	s_dd.resize(size);
	s_ddd.resize(size);
	int i = 0;
	for (auto te : t)
	{
		s[i] = (lon_qp.calc_point(te));
		s_d[i] = (lon_qp.calc_first_derivative(te));
		s_dd[i] = (lon_qp.calc_second_derivative(te));
		s_ddd[i] = (lon_qp.calc_third_derivative(te));
		++i;
		if (STOP_CAR and s[i] > 203.5)
		{
			s.resize(i);
			s_d.resize(i);
			s_dd.resize(i);
			s_ddd.resize(i);
			t.resize(i);
			d.resize(i);
			d_d.resize(i);
			d_dd.resize(i);
			d_ddd.resize(i);
			vecD d_ddd_vec = d_ddd;
			Jp = (inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
			break;
		}
	}
	// if(STOP_CAR){
	//   for(int i=0;i<s.size();i++)
	//   {
	//       if(s[i]>203.5)
	//         {
	//           s.resize(i);
	//           s_d.resize(i);
	//           s_dd.resize(i);
	//           s_ddd.resize(i);
	//           t.resize(i);
	//           d.resize(i);
	//           d_d.resize(i);
	//           d_dd.resize(i);
	//           d_ddd.resize(i);
	//           vecD d_ddd_vec = d_ddd;
	//           Jp = ( inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
	//           break;
	//         }
	//   }    iteration
	// }
	// https://www.geeksforgeeks.org/std-inner_product-in-cpp/
	Js = inner_product(s_ddd.begin(), s_ddd.end(), s_ddd.begin(), 0);

	double ds = pow((TARGET_SPEED - s_d.back()), 2);
	if (STOP_CAR && s_d.size() >= 2)
		ds = s_d[1] * s_d[1];
	// calculation of lateral, longitudnal and overall cost of the trajectories
	cd = (KJ * Jp + KT * Ti + KD * d.back() * d.back());
	cv = (KJ * Js + KT * Ti + KD_V * ds);
	cf = (KLAT * cd + KLON * cv);
}

// get sampling limits of d using the previously calculated paths (if present)
void get_limits_d(FrenetPath lp, double *lower_limit_d, double *upper_limit_d)
{
	vecD d_sampling = lp.get_d();
	if (d_sampling.size() != 0)
	{
		*lower_limit_d = d_sampling.back() - MAX_SHIFT_D;
		*upper_limit_d = d_sampling.back() + MAX_SHIFT_D + D_ROAD_W;
	}
}

// generates frenet path parameters
vector<FrenetPath> calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd,
									 double s0, FrenetPath lp)
{
	// trace(c_d_d);
	vector<FrenetPath> frenet_paths;
	double lower_limit_d, upper_limit_d;
	lower_limit_d = -MAX_ROAD_WIDTH; // cerr<<"Time 1 : "<<endTime1-startTime1<<endl;
									 // cerr<<"Time 2 : "<<endTime2-startTime2<<endl;
									 //cerr<<"Time 3 : "<<endTime3-startTime3<<endl;

	upper_limit_d = MAX_ROAD_WIDTH + D_ROAD_W;
	get_limits_d(lp, &lower_limit_d, &upper_limit_d); // IF not required to sample around previous
													  // sampled d(th) then comment this line.
	if (STOP_CAR)
	{
#pragma omp parallel for collapse(2)
		for (int Di = int(lower_limit_d / D_ROAD_W); Di <= int(upper_limit_d / D_ROAD_W); Di += 1) // sampling for lateral
																								   // offset
		{
			for (int Ti = 0; Ti <= int((MAXT + DT) / DT); Ti += 1) // Sampling for prediction time
			{
				double di = double(Di) / D_ROAD_W;
				double ti = double(Ti) / D_T_S;
				FrenetPath fp;
				FrenetPath tfp;
				fp.calc_lat_paths(c_d, c_d_d, c_d_dd, ti, di, TARGET_SPEED);
				vecD d_ddd_vec = fp.get_d_ddd();
				fp.set_Jp(inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
				tfp = fp;
				tfp.calc_lon_paths_quintic_poly(c_speed, s0, ti, 15, TARGET_SPEED);
#pragma omp critical
				frenet_paths.push_back(tfp);
			}
		}
	}
	else
	{

#pragma omp parallel for collapse(3)
		for (int Di = int(lower_limit_d / D_ROAD_W); Di <= int(upper_limit_d / D_ROAD_W); Di += 1) // sampling for lateral
																								   // offset
		{
			for (int Ti = int(MINT / DT); Ti <= int((MAXT + DT) / DT); Ti += 1) // Sampling for prediction time
			{

				for (int Di_d = int(-MAX_LAT_VEL / D_D_NS); Di_d <= int((MAX_LAT_VEL + D_D_NS) / D_D_NS); Di_d += 1)
				{
					double di = double(Di) * D_ROAD_W;
					double ti = double(Ti) * DT;
					double di_d = double(Di_d) * D_D_NS;

					FrenetPath fp;
					FrenetPath tfp;
					fp.calc_lat_paths(c_d, c_d_d, c_d_dd, ti, di, di_d);
					vecD d_ddd_vec = fp.get_d_ddd();
					fp.set_Jp(inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
					double minV = TARGET_SPEED - D_T_S * N_S_SAMPLE;
					double maxV = TARGET_SPEED + D_T_S * N_S_SAMPLE;

					// sampling for longitudnal velocity
					for (double tv = minV; tv <= maxV + D_T_S; tv += D_T_S)
					{
						tfp = fp;
						tfp.calc_lon_paths_quintic_poly(c_speed, s0, ti, 15, tv);
#pragma omp critical
						frenet_paths.push_back(tfp);
					}
				}
			}
		}
	}
	return frenet_paths;
}

void FrenetPath::adding_global_path(Spline2D csp)
{
	double startTime1 = omp_get_wtime();
	int n = s.size();
	x.resize(n);
	y.resize(n);
	for (int i = 0; i < n; i++)
	{
		double ix, iy;
		csp.calc_position(ix, iy, s[i]);
		if (ix == NONE)
		{
			return;
		}
		double iyaw = csp.calc_yaw(s[i]);
		double fx = ix - d[i] * sin(iyaw);
		double fy = iy + d[i] * cos(iyaw);
		x[i] = (fx);
		y[i] = (fy);
	}
	yaw.resize(n - 1);
	ds.resize(n - 1);

	for (int i = 0; i < n - 1; i++)
	{
		double dx = x[i + 1] - x[i];
		double dy = y[i + 1] - y[i];
		if (abs(dx) > 0.0001)
		{
			yaw[i] = (atan2(dy, dx));
		}
		else
		{
			yaw[i] = 0;
		}

		ds[i] = (sqrt(dx * dx + dy * dy));
	}
	// TO remove paths whose predicted s goes out of bounds of global path.
	if (s.size() == x.size())
	{
		return;
	}
	c.resize((n - 1) - 1);
	for (int i = 0; i < (n - 1) - 1; i++)
	{
		if (ds[i] != 0)
			c[i] = ((yaw[i + 1] - yaw[i]) / ds[i]);
		else
		{
			// c[i]=0; //why zero why not FLOAT_MAX
			c[i] = FLT_MAX;
		}
	}
	double endTime1 = omp_get_wtime();
}

// convert the frenet paths to global frame
vector<FrenetPath> calc_global_paths(vector<FrenetPath> fplist, Spline2D csp)
{
	int n = fplist.size();
#pragma omp parallel for collapse(1)
	for (int i = 0; i < n; i++)
	{
		//FrenetPath fp = fplist[i];
		fplist[i].adding_global_path(csp);
	}
	return fplist;
}

// transforms robot's footprint
vector<geometry_msgs::msg::Point32> transformation(vector<geometry_msgs::msg::Point32> fp,
											  geometry_msgs::msg::Pose cp, double px, double py, double pyaw)
{
	vector<geometry_msgs::msg::Point32> new_fp(fp.size());
	tf2::Quaternion qb(cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w);
	tf2::Matrix3x3 mb(qb);
	double broll, bpitch, byaw;
	mb.getRPY(broll, bpitch, byaw);
	double bx, by;
	bx = cp.position.x;
	by = cp.position.y;
	double x, y, theta;
	theta = pyaw - byaw;
	x = px - bx;
	y = py - by;
	int n = new_fp.size();
	for (int i = 0; i < n; i++)
	{
		new_fp[i].x = (fp[i].x - bx) * cos(theta) + (fp[i].y - by) * sin(theta) + x + bx;
		new_fp[i].y = -(fp[i].x - bx) * sin(theta) + (fp[i].y - by) * cos(theta) + y + by;
	}
	return new_fp;
}

// returns distance between two points
#pragma omp declare simd
double dist(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool point_obcheck(geometry_msgs::msg::Point32 p, double obst_r)
{
	//cout<<ob_x<<endl;
	//cout<<ob_y<<endl;
	//printf("Doing obcheck");
	int xlower, ylower, xupper, yupper;
	auto it = lower_bound(ob_x.begin(), ob_x.end(), p.x);
	if (ob_x.size() == 0)
	{
		return 0;	
	}
	if (it == ob_x.begin())
	{
		xlower = xupper = it - ob_x.begin(); // no smaller value than val in vector
	}
	else if (it == ob_x.end())
	{
		xupper = xlower = (it - 1) - ob_x.begin(); // no bigger value than val in vector
	}
	else
	{
		xlower = (it - 1) - ob_x.begin();
		xupper = it - ob_x.begin();
	}
	double dist1 = dist(p.x, p.y, ob_x[xlower], ob_y[xlower]);
	double dist2 = dist(p.x, p.y, ob_x[xupper], ob_y[xupper]);
	if (min(dist1, dist2) < obst_r)
	{
		return 1;
	}
	it = lower_bound(ob_y.begin(), ob_y.end(), p.y);
	if (it == ob_y.begin())
	{
		ylower = yupper = it - ob_y.begin(); // no smaller value  than val in vector
	}
	else if (it == ob_y.end())
	{
		yupper = ylower = (it - 1) - ob_y.begin(); // no bigger value than val in vector
	}
	else
	{
		ylower = (it - 1) - ob_y.begin();
		yupper = it - ob_y.begin();
	}
	dist1 = dist(p.x, p.y, ob_x[ylower], ob_y[ylower]);
	dist2 = dist(p.x, p.y, ob_x[yupper], ob_y[yupper]);
	if (min(dist1, dist2) < obst_r)
	{
		return 1;
	}
	return 0;
}

// checks for collision of the bot
bool FrenetPath::check_collision(double obst_r)
{
	//printf("checking collision/n");
	if (s.size() != x.size())
	{
		//printf("condition 1\n");
		return 1;
	}
	//printf("condition 2\n");
	for (unsigned int i = 0; i < min(x.size(), yaw.size()); i++)
	{
		//printf("entered loop");
		vector<geometry_msgs::msg::Point32> trans_footprint = transformation(footprint.polygon.points,odom.pose.pose, x[i], y[i], yaw[i]);
		for (unsigned int j = 0; j < trans_footprint.size(); j++)
		{
			if (point_obcheck(trans_footprint[j], obst_r) == 1)
			{
				printf("Returning 1");
				return 1;
			}
		}
	}
	return 0;
}

inline bool sortByCost(FrenetPath a, FrenetPath b)
{
	if (a.get_cf() != b.get_cf())
	{
		return a.get_cf() < b.get_cf();
	}
	else
	{
		double jerkCost1, jerkCost2;
		jerkCost1 = KLAT * a.get_Jp() + KLON * a.get_Js();
		jerkCost2 = KLAT * b.get_Jp() + KLON * b.get_Js();
		return jerkCost1 < jerkCost2;
	}
}

// check for specified velocity, acceleration, curvature constraints and collisions
vector<FrenetPath> check_path(vector<FrenetPath> &fplist, double bot_yaw, double yaw_error,
							  double obst_r)
{
	vector<FrenetPath> fplist_final;

#pragma omp parallel for collapse(1)
	for (unsigned int i = 0; i < fplist.size(); i++)
	{
		FrenetPath fp = fplist[i];
		int flag = 0;
		vecD path_yaw = fplist[i].get_yaw();
		if (path_yaw.size() == 0)
			continue;
		//cout<<"path yaww -- -- -- -- -- --"<<path_yaw[0]<<endl;	
		if ((path_yaw[0] - bot_yaw) > yaw_error || (path_yaw[0] - bot_yaw) < -yaw_error) // 20 deg
		{
		
			flag = 1;
		}
		if (flag == 1)
		{
			continue;
		}
		else if (fp.check_collision(obst_r) == 0)
		{
#pragma omp critical
			fplist_final.push_back(fplist[i]);
		}
	}
	return fplist_final;
}

void FrenetPath::plot_path()
{
	plt::plot(x, y);
	plt::pause(0.001);
}
void FrenetPath::plot_velocity_profile()
{
	plt::plot(t, s_d);
	plt::pause(0.001);
}

static int flag_for_display_paths = 0;

void display_paths(vector<FrenetPath> fplist)
{
	plt::ion();
	plt::show();
	int count = 0;
	for (auto &fp : fplist)
	{
		if (count % 50 == 0 && flag_for_display_paths)
		{
			fp.plot_path();
		}
		count++;
	}
	flag_for_display_paths = 1;
}

// generates the path and returns the bestpath
FrenetPath frenet_optimal_planning(Spline2D csp, double s0, double c_speed, double c_d,
								   double c_d_d, double c_d_dd, FrenetPath lp, double bot_yaw)
{
	trace("start");
	double startTime1 = omp_get_wtime();
	vector<FrenetPath> fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, lp);
	double endTime1 = omp_get_wtime();

	trace("calc_global_paths");
	double startTime2 = omp_get_wtime();
	fplist = calc_global_paths(fplist, csp);
	double endTime2 = omp_get_wtime();
	trace("check_path");
	transform_count = 0;
	// for now maximum possilble paths are taken into list
	double startTime3 = omp_get_wtime();
	fplist = check_path(fplist, bot_yaw, 0.5, 2.0);
	//if (fplist.size() == 0)
	//{
//		fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0, lp);
//		fplist = calc_global_paths(fplist, csp);
		
//	}
	
	double endTime3 = omp_get_wtime();
	trace("done checking ");
	if (false)
	{
		display_paths(fplist);
	}

	double min_cost = FLT_MAX;
	double cf;
	FrenetPath bestpath;
	for (auto &fp : fplist)
	{
		cf = fp.get_cf();
		if (min_cost >= cf)
		{
			min_cost = cf;
			bestpath = fp;
		}
	}
	if (false)
	{
		plt::ion();
		plt::show();
		bestpath.plot_path();
	}
	// For plotting velocity profile (x,y) = (t,s_d)
	if (false)
	{
		plt::ion();
		plt::show();
		bestpath.plot_velocity_profile();
	}
	trace("DONE");
	// if (STOP_CAR)
	// {
	// 	vector<double> chalja = bestpath.get_s_d();
	// 	if (chalja.size() >= 2)
	// 		cerr << "BEST PATH S_D[1] = " << chalja[1] << endl;
	// }
	return bestpath;
}

int cost_count = 0, footprint_count = 0, odom_count = 0;

// accesses the costmap and updates the obstacle coordinates
// 	double endTime1 = omp_get_wtime();

// 	/*cerr<<"parallel X is"<<endl;
//   for(auto i :ob_x){
// 	cerr<<i<<"  ";
//   }

//   cerr<<"paralle Y is"<<endl;
//   for(auto i :ob_y){
// 	cerr<<i<<"  ";
//   }*/

// 	//   ob_x.clear();
// 	//   ob_y.clear();
// 	//   double startTime2 = omp_get_wtime();
// 	//geometry_msgs::Pose origin = occupancy_grid->info.origin;
// 	// vector<pair<int,int>> ob;
// 	// #pragma omp parallel for collapse(2)
// 	//   for (width=0; width < occupancy_grid->info.width; ++width)
// 	//   {
// 	// 	for (height=0; height < occupancy_grid->info.height; ++height)
// 	// 	{
// 	// 	  if(occupancy_grid->data[height*occupancy_grid->info.width + width] > 0)
// 	// 	  {
// 	//       //#pragma omp critical
// 	//       {
// 	//     //     ob.emplace_back({width * occupancy_grid->info.resolution + occupancy_grid->info.
// 	// 		// resolution / 2 + origin.position.x,height * occupancy_grid->info.resolution + occupancy_grid->info.resolution
// 	// 		// / 2 + origin.position.y});
// 	//         ob_x.emplace_back(width * occupancy_grid->info.resolution + occupancy_grid->info.
// 	// 		resolution / 2 + origin.position.x);
// 	// 		ob_y.emplace_back(height * occupancy_grid->info.resolution + occupancy_grid->info.resolution
// 	// 		/ 2 + origin.position.y);

// 	//       }

// 	// 	  }
// 	// 	}
// 	//   }
// 	//   double endTime2 = omp_get_wtime();

// 	/*cerr<<"simple X is"<<endl;
//   for(auto i :ob_x){
// 	cerr<<i<<"  ";
//   }

//   cerr<<"simple Y is"<<endl;
//   for(auto i :ob_y){
// 	cerr<<i<<"  ";
//   }*/
// 	//ob.resize(ob_x.)

// 	// cerr<<"costmap parallel time = "<< endTime1 -startTime1<<endl;
// 	// cerr<<"costmap simple time = "<< endTime2 -startTime2<<endl;
// }


using std::placeholders::_1;

class FrenetClass : public rclcpp::Node {
public:  
	vecD wx, wy;
	double target_speed;
        FrenetClass()  : Node("frenet_class")
        {
            publisher_frenet_path = this->create_publisher<nav_msgs::msg::Path>("/frenet_path", 1);
            publisher_global_path = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);
            publisher_target_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        		
	    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/base_pose_ground_truth", 10, std::bind(&FrenetClass::odom_callback,this, _1));
            footprint_sub = this->create_subscription<geometry_msgs::msg::PolygonStamped>("/move_base/local_costmap/footprint", 10, std::bind(&FrenetClass::footprint_callback,this, _1));
            costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/move_base/local_costmap/costmap", 10, std::bind(&FrenetClass::costmap_callback, this, _1));
		
	    wx = W_X;
	    wy = W_Y;
        }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_frenet_path;            
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_global_path;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_target_vel;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;

    void footprint_callback(const geometry_msgs::msg::PolygonStamped::ConstPtr p)
    {
    	     //printf("FOOT PRINT CALLBACK\n");
	     ::footprint = *p;
	     //printf("footprint size = %d\n",footprint.polygon.points.size());
    }

// accesses the odometry data
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr msg)
    {
	     ::odom = *msg;
    }
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::ConstPtr occupancy_grid)
    {

	    cost_count++;
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
   

};

// calculates the distance between two points (x1,y1) and (x2,y2)
inline double calc_dis(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

// finds the point in the global path which is nearest to the bot
void find_nearest_in_global_path(vecD &global_x, vecD &global_y, double &min_x, double &min_y,
								 double &min_dis, int &min_id, int flag, FrenetPath &path)
{
	double bot_x, bot_y;
	if (flag == 0)
	{
	
		bot_x = odom.pose.pose.position.x;
		bot_y = odom.pose.pose.position.y;
		//bot_x =  35;
		//bot_y =  -53;
		//printf("%lf --- %lf\n", odom.pose.pose.position.x, odom.pose.pose.position.y);
	}
	else
	{
		bot_x = path.get_x()[1];
		bot_y = path.get_y()[1];
	}
	min_dis = FLT_MAX;
	for (unsigned int i = 0; i < global_x.size(); i++)
	{
		double dis = calc_dis(global_x[i], global_y[i], bot_x, bot_y);
		if (dis < min_dis)
		{
			min_dis = dis;
			min_x = global_x[i];
			min_y = global_y[i];
			min_id = i;
		}
	}
}

inline double get_bot_yaw()
{
	geometry_msgs::msg::Pose p = odom.pose.pose;
	//printf("\ngeometry_msgs::msg::Pose p = odom.pose.pose -- -- --  -- -- -- %lf %lf %lf\n",odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
	tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	trace(yaw, pitch, roll); // is -nan
	//printf("print the yaw - - - - - - - %lf %lf %lf\n",yaw ,pitch ,roll);
	return yaw;
}

void initial_conditions_path(Spline2D &csp, double &s0, double &c_speed, double &c_d, double &c_d_d,
							 double &c_d_dd, double &bot_yaw, FrenetPath &path)
{
	// Approach 1
	//printf("Line number:%d\n", __LINE__);
	//cout<<"Frenet Path"<< path.get_s() <<endl;
	vecD d = path.get_d();
	vecD s_d = path.get_s_d();
	vecD d_d = path.get_d_d();
	vecD d_dd = path.get_d_dd();
	//printf("Line number:%d\n", __LINE__);
	vecD s = path.get_s();
	//printf("Line number:%d\n", __LINE__);
	//cout << "VecdD s =:" << s << endl;
	s0 = s[1];
	//printf("Line number:%d\n", __LINE__);
	c_speed = s_d[1];
	c_d = d[1];
	//printf("Line number:%d\n", __LINE__);
	c_d_d = d_d[1];
	//printf("Line number:%d\n", __LINE__);
	c_d_dd = 0;
	//printf("Line number:%d\n", __LINE__);
	
	bot_yaw = get_bot_yaw();
}

int initial_conditions_new(Spline2D &csp, vecD &global_s, vecD &global_x, vecD &global_y,
						   vecD &global_R, vecD &global_yaw, double &s0, double &c_speed, double &c_d, double &c_d_d,
						   double &c_d_dd, double &bot_yaw, FrenetPath &path)
{
	double vx = odom.twist.twist.linear.x;
	//printf("Now printing vx vy ----------------------------------------------------------------------------------------\n");
	double vy = odom.twist.twist.linear.y;
	//printf("%lf %lf",vx,vy);
	double v = sqrt(vx * vx + vy * vy);
	double min_x, min_y;
	int min_id;

	// getting d
	find_nearest_in_global_path(global_x, global_y, min_x, min_y, c_d, min_id, 0, path);

	// deciding the sign for d
	pair<double, double> vec1, vec2;
	vec1.first = odom.pose.pose.position.x - global_x[min_id];
	//cout<<"vec1.first = odom.pose.pose.position.x - global_x[min_id];"<<vec1.first<<endl;
	vec1.second = odom.pose.pose.position.y - global_y[min_id];
	//cout<<"vec1.second = odom.pose.pose.position.y - global_y[min_id];"<<vec1.second<<endl;
	vec2.first = global_x[min_id] - global_x[min_id + 1];
	vec2.second = global_y[min_id] - global_y[min_id + 1];
	double curl2D = vec1.first * vec2.second - vec2.first * vec1.second;
	if (curl2D < 0)
		c_d *= -1;
	//printf("c_d = %lf\n",c_d);
	s0 = global_s[min_id];
	bot_yaw = get_bot_yaw();
	double g_path_yaw = global_yaw[min_id];
	trace(bot_yaw, g_path_yaw);
	double delta_theta = bot_yaw - g_path_yaw;
	trace(delta_theta);
	c_d_d = v * sin(delta_theta); // Equation 5
	double k_r = global_R[min_id];
	c_speed = v * cos(delta_theta) / (1 - k_r * c_d); // s_dot (Equation 7)
	c_d_dd = 0;										  // For the time being. Need to be updated
	return min_id;
} 

// publishes path as ros messages
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
		//tf2::Quaternion q = tf2_ros::createQuaternionFromRPY(0, 0, yaw); // roll , pitch = 0
		tf2::Quaternion q;
		q.setRPY(0, 0, yaw);
		q = q.normalize();
		tf2::convert(loc.pose.orientation, q);
		//printf("CONVERTED QUATERNION __________________________________________________________________\n");
		//cout<<"geo values are:  "<<loc.pose.orientation.x<<"  "<<loc.pose.orientation.y<<"  "<<loc.pose.orientation.z<<"  "<<loc.pose.orientation.w<<endl;
		path_msg.poses.push_back(loc);
	}
}

int main(int argc, char **argv)
{
	bool gotOdom = false;
	rclcpp::init(argc, argv);
	//rclcpp::spin(std::make_shared<FrenetClass>());
	//auto node = rclcpp::Node::make_shared("frenet_planner");
	auto node = std::make_shared<FrenetClass>();
	

	//W_X = FrenetClass.wx;
	W_X = node->wx;
	//W_Y = FrenetClass.wy;
	W_Y = node->wy;
	//TARGET_SPEED = FrenetClass.target_speed;
	TARGET_SPEED = node->target_speed;
	RCLCPP_INFO(node->get_logger(), "running main");

	/*	vecD wx = {38, 38, 38, 38, 38, 38, 38, 38}; 	//38,-57 -> starting point of the bot
  vecD wy = {-57, -45,  -32.0,  -18.5,  -12.0, 0.0, 12, 35};*/
	vecD rx, ry, ryaw, rk;
	double ds = 0.1; // ds represents the step size for cubic_spline
	double bot_yaw, bot_v;

	// Global path is made using the waypoints
	Spline2D csp = calc_spline_course(W_X, W_Y, rx, ry, ryaw, rk, ds);

	FrenetPath path;
	FrenetPath lp;
	double s0, c_d, c_d_d, c_d_dd, c_speed;
	unsigned int ctr = 0, i;
	//printf("printing rx see if it works - %d\n", rx.size()); 
	vector<double> global_s(rx.size());
	//printf("vector defined");
	double s = 0;
	global_s[0] = 0;
	//printf("global 0 value set");
	for (unsigned int i = 1; i < rx.size(); i++)
	{
		double dis = calc_dis(rx[i], ry[i], rx[i - 1], ry[i - 1]);
		s = s + dis;
		//printf("dist = %f", s); 
		global_s[i] = s;
	}
	//printf("s set");
	s_dest = global_s.back();
	//printf("s_dest = %lf", s_dest);
	bool run_frenet = true;
	//printf("plt\n");
	plt::ion();
		plt::show();
	vector<double> plts0,pltcspeed,plttime;
	double pltbasetime = omp_get_wtime();
	//printf("plt done\n");
	int init_flag = true;
	int iteration_count = 1;
	while (rclcpp::ok())
	//while(true)
	{
		printf("Starting loop %d iteration\n", iteration_count);
		rclcpp::spin_some(node);
		ofstream fout;
		fout.open("/home/animesh/try_ws/src/frenet_planner_agv/src/log.txt",ios::app);
		int min_id = 0;
		// Specifing initial conditions for the frenet planner using odometry
		double startTime1 = omp_get_wtime();
		//	cout<<"Frenet Path"<< path.get_s() <<endl;
		
		if (init_flag)
		{
		//printf("Line number:%d\n", __LINE__);
			min_id = initial_conditions_new(csp, global_s, rx, ry, rk, ryaw, s0, c_speed, c_d, c_d_d,c_d_dd, bot_yaw, path);
			init_flag = false;
											
		}
		else
		{
		  //      printf("Line number:%d\n", __LINE__);
		 	initial_conditions_path(csp, s0, c_speed, c_d, c_d_d, c_d_dd, bot_yaw, path);
		}
		//printf("Line number:%d\n", __LINE__);
		double endTime1 = omp_get_wtime();
		//plts0.push_back(s0);
		//pltcspeed.push_back(c_speed);
		//plttime.push_back(abs(endTime1-pltbasetime));
		//plt::plot(plttime,pltcspeed);
		//plt::pause(0.001);
		//fout<<"a run"<<endl;
		//fout<<plts0<<endl;
		//fout<<plttime<<endl;
		//fout<<pltcspeed<<endl;
		//fout<<"end"<<endl;
		trace("frenet optimal planning", s0, c_speed, c_d, c_d_d, c_d_dd, lp, bot_yaw);
		//if (abs(s_dest - s0) <= 15) // convert 203.5 to a variable in terms of s_dest
		//{
			//STOP_CAR = true;
		//	TARGET_SPEED = 0;
		//	cerr << "STOP\n";
		//	cerr << s_dest << endl;
		//} else{
	//  STOP_CAR = false;
	//}
	//	 if(abs(s0-s_dest) <= 5)
	//	 {
	//	c_speed /= 2;
	//	 }
		//printf("Line number:%d\n", __LINE__);
		// Getting the optimal frenet path
		//if(run_frenet){
	  //double startTime2 = omp_get_wtime();
	//path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, lp, bot_yaw);
	//double endTime2 = omp_get_wtime();
	//}

		double startTime2 = omp_get_wtime();
		//cout<<"Frenet Path Main"<< path.get_s() <<endl;
		path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, lp, bot_yaw);
		//cout<<"Frenet Path Main"<< path.get_s() <<endl;
		double endTime2 = omp_get_wtime();

		if (false)
		{
			cerr << endl
				 << " s_d" << endl;
			for (auto i : path.get_s_d())
				cerr << i << "  ";
			cerr << endl
				 << " d_d" << endl;
			for (auto i : path.get_d_d())
				cerr << i << "  ";
			//run_frenet=false;
		}

		//printf("Line number:%d\n", __LINE__);
		lp = path;
		nav_msgs::msg::Path path_msg;
		nav_msgs::msg::Path global_path_msg;

		// paths are published in map frame
		path_msg.header.frame_id = "map";
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
		//printf("Line number:%d\n", __LINE__);
		if (true)
		{
			plt::ion();
			plt::show();
			plt::plot(lp.get_x(), lp.get_y());
			//plt::plot(lp.get_s(), lp.get_d());
			//vecD a = lp.get_s();
			//vecD a_f = {a[0]};
			//vecD b = lp.get_d();
			//vecD b_f = {b[0]};
			//plt::scatter(a_f,b_f);
			plt::pause(0.001);
			plt::plot(rx, ry);
			plt::scatter(ob_x,ob_y);
			plt::pause(0.001);
		}
		//printf("\nx and y coordinate - %lf,%lf\n",lp.get_x(),lp.get_y());
		// Required tranformations on the Frenet path are made and pushed into message
		double startTime3 = omp_get_wtime();
		publishPath(path_msg, path, rk, ryaw, c_d, c_speed, c_d_d);
		double endTime3 = omp_get_wtime();

		/********************* have to find a proper justification for taking the midpoint******/
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
		 //cerr<<"Time 1 : "<<endTime1-startTime1<<endl;
		 //cerr<<"Time 2 : "<<endTime2-startTime2<<endl;
		 //cerr<<"Time 3 : "<<endTime3-startTime3<<endl;
		geometry_msgs::msg::Twist vel;
		vel.linear.x = bot_v;
		vel.linear.y = 0;
		vel.linear.z = 0;

		//FrenetClass.publisher_global_path->publish(path_msg);
		//FrenetClass.publisher_frenet_path->publish(global_path_msg);
		//FrenetClass.publisher_target_vel->publish(vel);
		
		node->publisher_global_path->publish(path_msg);
		node->publisher_frenet_path->publish(global_path_msg);
		node->publisher_target_vel->publish(vel);
		
		
		ctr++;
		iteration_count += 1;
		 //if(!run_frenet){
		   //cerr<<"ending frenet"<<endl;
		   //break;
		 //}
		fout.close();
//		printf("Ending loop %d iteration\n", iteration_count++);
	}
	return 0;
}
