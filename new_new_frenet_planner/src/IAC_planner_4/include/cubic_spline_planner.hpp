#ifndef CUBIC_SPLINE_PLANNER_HPP_
#define CUBIC_SPLINE_PLANNER_HPP_
#define EIGEN_NO_DEBUG
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

#define NONE -1e9
using namespace std;
using namespace Eigen;

using vecD =  vector<double>;
class Spline
{
  private:
    vecD a, b, c, d, w;
    vecD x, y;
    int nx;

    // Functions
    MatrixXd& calc_A(vecD h);
    MatrixXd& calc_B(vecD h);
    int search_index(double p);

  public:
    void init(vecD x_in, vecD y_in);
    double calc(double t);  // complete
    double calcd(double t);  // complete
    double calcdd(double t);  // complete
};

class Spline2D
{
  private:
  vecD x, y, s, ds;
  Spline sx, sy;

  public:
    Spline2D(vecD x_in, vecD y_in)
    {
      x = x_in;
      y = y_in;
      //printf("mid  Spline2D\n");
      s = calc_s(x, y);
      //printf("mid before init 1 Spline2D\n");
      sx.init(s, x);
      //printf("mid3 before init 2 Spline2D\n");
      sy.init(s, y);
      //printf("mid4  Spline2D\n");
    }


    vecD calc_s(vecD x, vecD y);
    void calc_position(double &x, double &y, double t);
    double calc_curvature(double t);
    double calc_yaw(double t);
    double get_s_last();
};  // end of class

Spline2D& calc_spline_course(vecD x, vecD y, vecD &rx, vecD &ry, vecD &ryaw, vecD &rk, double ds);
void printVecD(vecD A);

#endif  // CUBIC_SPLINE_PLANNER_HPP_
