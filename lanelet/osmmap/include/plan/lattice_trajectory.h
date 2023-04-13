/*
 * @Author: wpbit
 * @Date: 2023-03-15 19:25:25
 * @LastEditors: wpbit
 * @LastEditTime: 2023-03-21 21:10:10
 * @Description: 
 */
#ifndef LATTICE_TRAJECTORY_H_
#define LATTICE_TRAJECTORY_H_

#include <limits>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include "tools/cpprobotics_types.h"
#include "tools/polynomial_quartic_curve1d.h"
#include "tools/polynomial_quintic_curve1d.h"
#include "tools/cubic_spline.h"

namespace plan
{
struct FrenetPath
{
    double cd = 0.0;
    double cv = 0.0;
    double cf = 0.0;

    cpprobotics::Vec_f t;       //time
    cpprobotics::Vec_f d;       //lateral offset
    cpprobotics::Vec_f d_d;     //lateral speed
    cpprobotics::Vec_f d_dd;    //lateral acceleration
    cpprobotics::Vec_f d_ddd;   //lateral jerk
    cpprobotics::Vec_f s;       //s position along spline
    cpprobotics::Vec_f s_d;     //s speed
    cpprobotics::Vec_f s_dd;    //s acceleration
    cpprobotics::Vec_f s_ddd;   //s jerk;

    cpprobotics::Vec_f x;       //x position
    cpprobotics::Vec_f y;       //y position
    cpprobotics::Vec_f yaw;     //yaw /rad
    cpprobotics::Vec_f v;       //speed
    cpprobotics::Vec_f ds;      //radian length
    cpprobotics::Vec_f c;       //curvature

    double max_speed;
    double max_accel;
    double max_curvature;

    void clear()
    {
        cd = 0.0;
        cv = 0.0;
        cf = 0.0;
        max_speed = 0.0;
        max_accel = 0.0;
        max_curvature = 0.0;
        t.clear();
        d.clear();
        d_d.clear();
        d_dd.clear();
        d_ddd.clear();
        s.clear();
        s_d.clear();
        s_dd.clear();
        s_ddd.clear();
        x.clear();
        y.clear();
        yaw.clear();
        v.clear();
        ds.clear();
        c.clear();
    }
};

using Vec_Path = std::vector<FrenetPath>;

class FrenetOptimalTrajectory
{
private:
    
public:
    FrenetOptimalTrajectory();
    ~FrenetOptimalTrajectory();
    double SumofPower(std::vector<double> value_list);
    double NormalizeAngle(const double angle);
    Vec_Path CalculateFrenetPaths(double c_speed, double c_d, double c_dd, double c_ddd, double s0);
    void CalculateTrajectory(Vec_Path &path_list, Spline2D csp);
    bool CheckCollision(FrenetPath path, const cpprobotics::Vec_Poi ob);
    Vec_Path CheckPaths(Vec_Path path_list, const cpprobotics::Vec_Poi ob);
    FrenetPath FrenetOptimalPlanning(Spline2D csp, double s0, double c_speed,
                                     double c_d, double c_dd, double c_ddd, cpprobotics::Vec_Poi ob);
    // FrenetPath FrenetOptimalPlanning(Spline2D csp, const initpos, cpprobotics::Vec_Poi ob);
    void VisualizationSamplePoint(Spline2D csp, double s0, double c_speed,
                                 double c_d, double c_dd, double c_ddd,visualization_msgs::Marker &samplepoints);
};




}//namespace plan


#endif