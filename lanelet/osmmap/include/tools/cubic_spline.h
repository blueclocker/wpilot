#ifndef CUBIC_SPLINE_H_
#define CUBIC_SPLINE_H_

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include "hdmap/centerway.h"

namespace plan{

inline std::vector<double> vec_diff(std::vector<double> input){
  std::vector<double> output;
  for(unsigned int i=1; i<input.size(); i++){
    output.push_back(input[i] - input[i-1]);
  }
  return output;
};

inline std::vector<double> cum_sum(std::vector<double> input){
  std::vector<double> output;
  double temp = 0;
  for(unsigned int i=0; i<input.size(); i++){
    temp += input[i];
    output.push_back(temp);
  }
  return output;
};

class Spline{
public:
  std::vector<double> x;
  std::vector<double> y;
  int nx;
  std::vector<double> h;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  //Eigen::VectorXf c;
  std::vector<double> d;

  Spline(){};
  // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
  Spline(std::vector<double> x_, std::vector<double> y_):x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_){
    Eigen::MatrixXd A = calc_A();
    Eigen::VectorXd B = calc_B();
    Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
    double * c_pointer = c_eigen.data();
    //Eigen::Map<Eigen::VectorXf>(c, c_eigen.rows(), 1) = c_eigen;
    c.assign(c_pointer, c_pointer+c_eigen.rows());

    for(int i=0; i<nx-1; i++){
      d.push_back((c[i+1]-c[i])/(3.0*h[i]));
      b.push_back((a[i+1] - a[i])/h[i] - h[i] * (c[i+1] + 2*c[i])/3.0);
    }
  };

  double calc(double t){
    // if(t<x.front() || t>x.back()){
    //   throw std::invalid_argument( "received value out of the pre-defined range" );
    // }
    if(t < x.front()) t = x.front();
    if(t > x.back()) t = x.back();
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
  };

  double calc_d(double t){
    // if(t<x.front() || t>x.back()){
    //   throw std::invalid_argument( "received value out of the pre-defined range" );
    // }
    if(t < x.front()) t = x.front();
    if(t > x.back()) t =  x.back();
    int seg_id = bisect(t, 0, nx-1);
    double dx = t - x[seg_id];
    return b[seg_id]  + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  double calc_dd(double t){
    // if(t<x.front() || t>x.back()){
    //   throw std::invalid_argument( "received value out of the pre-defined range" );
    // }
    if(t < x.front()) t = x.front();
    if(t > x.back()) t = x.back();
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

  double calc_ddd(double t){
    // if(t<x.front() || t>x.back()){
    //   throw std::invalid_argument( "received value out of the pre-defined range" );
    // }
    if(t < x.front()) t = x.front();
    if(t > x.back()) t = x.back();
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 6 * d[seg_id];
  }

private:
  Eigen::MatrixXd calc_A(){
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
    A(0, 0) = 1;
    for(int i=0; i<nx-1; i++){
      if (i != nx-2){
        A(i+1, i+1) = 2 * (h[i] + h[i+1]);
      }
      A(i+1, i) = h[i];
      A(i, i+1) = h[i];
    }
    A(0, 1) = 0.0;
    A(nx-1, nx-2) = 0.0;
    A(nx-1, nx-1) = 1.0;
    return A;
  };
  Eigen::VectorXd calc_B(){
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
    for(int i=0; i<nx-2; i++){
      B(i+1) = 3.0*(a[i+2]-a[i+1])/h[i+1] - 3.0*(a[i+1]-a[i])/h[i];
    }
    return B;
  };

  int bisect(double t, int start, int end){
    int mid = (start+end)/2;
    if (t==x[mid] || end-start<=1){
      return mid;
    }else if (t>x[mid]){
      return bisect(t, mid, end);
    }else{
      return bisect(t, start, mid);
    }
  }
};

class Spline2D{
public:
  Spline sx;
  Spline sy;
  Spline sz;
  std::vector<double> s;

  Spline2D(std::vector<map::centerway::CenterPoint3D> points_){
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    for(int i = 0; i < points_.size(); ++i)
    {
      x.push_back(points_[i].x_);
      y.push_back(points_[i].y_);
      z.push_back(points_[i].ele_);
    }
    s = calc_s(x, y, z);
    sx = Spline(s, x);
    sy = Spline(s, y);
    sz = Spline(s, z);
  };

  std::array<double, 3> calc_postion(double s_t){
    double x = sx.calc(s_t);
    double y = sy.calc(s_t);
    double z = sz.calc(s_t);
    return {{x, y, z}};
  };

  double calc_curvature(double s_t){//曲率
    double dx = sx.calc_d(s_t);
    double ddx = sx.calc_dd(s_t);
    double dy = sy.calc_d(s_t);
    double ddy = sy.calc_dd(s_t);
    return (ddy * dx - ddx * dy)/(dx * dx + dy * dy) + 1e-6;//原本是std::pow(..., 1)
  };

  double calc_yaw(double s_t){//朝向角
    double dx = sx.calc_d(s_t);
    double dy = sy.calc_d(s_t);
    return std::atan2(dy, dx);
  };

  double calc_rdkappa(double s_t){//曲率对弧长s的一阶导数
    double dx = sx.calc_d(s_t);
    double ddx = sx.calc_dd(s_t);
    double dddx = sx.calc_ddd(s_t);
    double dy = sy.calc_d(s_t);
    double ddy = sy.calc_dd(s_t);
    double dddy = sy.calc_ddd(s_t);
    return ((dddy*dx - dddx*dy)*(dx*dx + dy*dy) - 3*(dx*ddx + dy*ddy)*(ddy*dx - ddx*dy))/std::pow((dx * dx + dy * dy), 3.0);
  };


private:
  std::vector<double> calc_s(std::vector<double> x, std::vector<double> y, std::vector<double> z){
    std::vector<double> ds;
    std::vector<double> out_s{0};
    std::vector<double> dx = vec_diff(x);
    std::vector<double> dy = vec_diff(y);
    std::vector<double> dz = vec_diff(z);

    for(unsigned int i=0; i<dx.size(); i++){
      ds.push_back(std::sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]));
    }

    std::vector<double> cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  };
};
}//namespace plan
#endif