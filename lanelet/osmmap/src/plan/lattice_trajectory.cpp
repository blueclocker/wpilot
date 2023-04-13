/*
 * @Author: wpbit
 * @Date: 2023-03-15 19:25:37
 * @LastEditors: wpbit
 * @LastEditTime: 2023-03-21 21:09:56
 * @Description: 
 */
#include "plan/lattice_trajectory.h"

namespace plan
{
#define MAX_SPEED 20.0 / 3.6        //max speed [m/s]
#define MAX_ACCEL 4.0               //max acceleration [m/ss]
#define MAX_CURVATURE 0.2           //max curvature [1/m]
#define MAX_ROAD_WIDTH 3.0          //max road width [m]
#define D_ROAD_W 0.2                //road width sampling length [m]
#define DT 0.2                      //time tick [s]
#define MAXT 5.0                    //max prediction time [m]
#define MINT 4.0                    //min prediction time [m]
#define TARGET_SPEED 15.0 / 3.6     //target speed [m/s]
#define D_T_S 2.5 / 3.6             //target speed sampling length [m/s]
#define N_S_SAMPLE 1                //sampling number of target speed
#define ROBOT_RADIUS 1.5            //robot radius [m]

#define KJ 0.5
#define KT 1.0
#define KD 1.0
#define KLAT 1.0
#define KLON 1.0

struct compare
{
    double key;
    compare(double const &i) : key(i) {}

    bool operator()(double const &i)
    {
        return (std::abs(i) > key);
    }
};


FrenetOptimalTrajectory::FrenetOptimalTrajectory()
{
}

FrenetOptimalTrajectory::~FrenetOptimalTrajectory()
{
}

double FrenetOptimalTrajectory::SumofPower(std::vector<double> value_list)
{
    double sum = 0.0;
    for(auto v : value_list) sum += v * v;
    return sum;
}

double FrenetOptimalTrajectory::NormalizeAngle(const double angle)
{
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if(a < 0.0) a += (2.0 * M_PI);
    return a - M_PI;
}

//获取采样轨迹
Vec_Path FrenetOptimalTrajectory::CalculateFrenetPaths(double c_speed, double c_d, double c_dd, double c_ddd, double s0)
{
    std::vector<FrenetPath> fp_list;
    for(double wi = -MAX_ROAD_WIDTH; wi <= MAX_ROAD_WIDTH; wi += D_ROAD_W)
    {
        for(double ti = MINT; ti <= MAXT; ti += DT)
        {
            //当前点状态 X3, 采样点 X3, 时间
            math::PolynomialQuinticCurve1d latqp(c_d, c_dd, c_ddd, wi, 0.0, 0.0, ti);
            cpprobotics::Vec_f time;
            cpprobotics::Vec_f d0;
            cpprobotics::Vec_f d1;
            cpprobotics::Vec_f d2;
            cpprobotics::Vec_f d3;

            //对每一时刻计算d
            for(double dt = 0.0; dt <= ti; dt += DT)
            {
                time.push_back(dt);
                d0.push_back(latqp.Evaluate(0, dt));
                d1.push_back(latqp.Evaluate(1, dt));
                d2.push_back(latqp.Evaluate(2, dt));
                d3.push_back(latqp.Evaluate(3, dt));
            }

            //根据速度s采样
            for(double vi = TARGET_SPEED - N_S_SAMPLE * D_T_S; vi <= TARGET_SPEED + N_S_SAMPLE * D_T_S; vi += D_T_S)
            {
                math::PolynomialQuarticCurve1d lon_qp(s0, c_speed, 0.0, vi, 0.0, ti);
                FrenetPath one_trajectory;
                cpprobotics::Vec_f s0;
                cpprobotics::Vec_f s1;
                cpprobotics::Vec_f s2;
                cpprobotics::Vec_f s3;

                for(double dt = 0.0; dt <= ti; dt += DT)
                {
                    s0.push_back(lon_qp.Evaluate(0, dt));
                    s1.push_back(lon_qp.Evaluate(1, dt));
                    s2.push_back(lon_qp.Evaluate(2, dt));
                    s3.push_back(lon_qp.Evaluate(3, dt));
                }

                double js = 0.0;
                double jd = 0.0;
                double j_la = 0.0;
                for(auto &elem : s3) js += elem * elem;
                for(auto &elem : d3) jd += elem * elem;
                for(auto &elem : d0) j_la += elem * elem;

                double diff_v = TARGET_SPEED - s1.back();
                double diff_l = 0 - d0.back();

                one_trajectory.t = time;
                one_trajectory.d = d0;
                one_trajectory.d_d = d1;
                one_trajectory.d_dd = d2;
                one_trajectory.d_ddd = d3;
                one_trajectory.s = s0;
                one_trajectory.s_d = s1;
                one_trajectory.s_dd = s2;
                one_trajectory.s_ddd = s3;
                double cd = KJ * jd + KT * ti + KD * diff_l * diff_l + 10.0 * j_la;
                double cv = KJ * js + KT * ti + KD * diff_v * diff_v;
                one_trajectory.cd = cd;
                one_trajectory.cv = cv;
                one_trajectory.cf = KLAT * cd + KLON * cv;

                fp_list.push_back(one_trajectory); 
            }
        }
    }
    //完成轨迹采样
    // std::cout << "total sample path: " << fp_list.size() << std::endl;
    return fp_list;
}

//根据参考轨迹与采样的轨迹，计算frenet中其他曲线参数，如航向角，曲率，ds等
void FrenetOptimalTrajectory::CalculateTrajectory(Vec_Path &path_list, Spline2D csp)
{
    //计算采样轨迹其他参数
    for(int num = 0; num < path_list.size(); ++num)
    {
        for(int s_num = 0; s_num < (path_list[num].s).size(); ++s_num)
        {
            double sr = path_list[num].s[s_num];
            std::array<double, 3> position = csp.calc_postion(sr);
            double xr = position[0];
            double yr = position[1];
            double theta_r = csp.calc_yaw(sr);
            double l = path_list[num].d[s_num];

            path_list[num].x.push_back(xr - l * sin(theta_r));
            path_list[num].y.push_back(yr + l * cos(theta_r));
            double kappa_r = csp.calc_curvature(sr);
            double ll = path_list[num].d_d[s_num];

            double ssr = path_list[num].s_d[s_num];
            path_list[num].v.push_back(std::pow(std::pow((ssr * (1.0 - kappa_r * l)), 2) + std::pow(ll, 2), 0.5));
        }

        for(int cu_num = 0; cu_num < (path_list[num].s).size(); ++cu_num)
        {
            if(cu_num == (int)(path_list[num].s.size()) - 1)
            {
                path_list[num].ds.push_back(path_list[num].ds[cu_num - 1]);
                path_list[num].yaw.push_back(path_list[num].yaw[cu_num - 1]);
            }else{
                double deltax = path_list[num].x[cu_num + 1] - path_list[num].x[cu_num];
                double deltay = path_list[num].y[cu_num + 1] - path_list[num].y[cu_num];
                path_list[num].ds.push_back(std::sqrt(deltax * deltax + deltay * deltay));
                path_list[num].yaw.push_back(std::atan2(deltay, deltax));
            }
        }

        for(int cu_num = 0; cu_num < (path_list[num].s).size(); ++cu_num)
        {
            if(cu_num == (int)(path_list[num].s.size()) - 1)
            {
                path_list[num].c.push_back(path_list[num].c[cu_num - 1]);
            }else{
                path_list[num].c.push_back((path_list[num].yaw[cu_num + 1] - path_list[num].yaw[cu_num]) / path_list[num].ds[cu_num]);
            }
        }
    }
    // std::cout << "finish calculate path information" << std::endl;
}

//碰撞检测
bool FrenetOptimalTrajectory::CheckCollision(FrenetPath path, const cpprobotics::Vec_Poi ob)
{
    for(auto point : ob)
    {
        for(int i = 0; i < path.x.size(); ++i)
        {
            double dist = std::pow((path.x[i] - point[0]), 2) + std::pow((path.y[i] - point[1]), 2);
            if(dist <= ROBOT_RADIUS * ROBOT_RADIUS)
            {
                return false;
            }
        }
    }
    return true;
}

//检查路径，通过限制最大速度，最大加速度，最大曲率和避障，选取可用的轨迹
Vec_Path FrenetOptimalTrajectory::CheckPaths(Vec_Path path_list, const cpprobotics::Vec_Poi ob)
{
    Vec_Path output_fp_list;
    for(int i = 0; i < path_list.size(); ++i)
    {
        if(std::any_of(path_list[i].s_d.begin(), path_list[i].s_d.end(), compare(MAX_SPEED))) continue;

        if(std::any_of(path_list[i].s_dd.begin(), path_list[i].s_dd.end(), compare(MAX_ACCEL))) continue;

        if(std::any_of(path_list[i].c.begin(), path_list[i].c.end(), compare(MAX_CURVATURE))) continue;

        if(!CheckCollision(path_list[i], ob)) continue;

        output_fp_list.push_back(path_list[i]);
    }

    // std::cout << "select path size is: " << output_fp_list.size() << std::endl;
    return output_fp_list;
}

FrenetPath FrenetOptimalTrajectory::FrenetOptimalPlanning(Spline2D csp, double s0, double c_speed,
                                     double c_d, double c_dd, double c_ddd, cpprobotics::Vec_Poi ob)
{
    //轨迹采样
    Vec_Path fp_list = CalculateFrenetPaths(c_speed, c_d, c_dd, c_ddd, s0);
    //补全轨迹信息，比如航向角，曲率，ds
    CalculateTrajectory(fp_list, csp);
    //路径筛选
    Vec_Path save_paths = CheckPaths(fp_list, ob);

    double min_cost = std::numeric_limits<double>::max();
    FrenetPath final_path;
    for(auto path : save_paths)
    {
        if(min_cost >= path.cf)
        {
            min_cost = path.cf;
            final_path = path;
        }
    }
    // std::cout << "FrenetOptimalPlanning final path size is: " << final_path.s.size() << std::endl;
    return final_path;
}

void FrenetOptimalTrajectory::VisualizationSamplePoint(Spline2D csp, double s0, double c_speed,
                                     double c_d, double c_dd, double c_ddd, visualization_msgs::Marker &samplepoints)
{
    Vec_Path fp_list = CalculateFrenetPaths(c_speed, c_d, c_dd, c_ddd, s0);
    CalculateTrajectory(fp_list, csp);
    for(int i = 0; i < fp_list.size(); ++i)
    {
        for(int j = 0; j < fp_list[i].x.size(); ++j)
        {
            geometry_msgs::Point p;
            p.x = fp_list[i].x[j];
            p.y = fp_list[i].y[j];
            samplepoints.points.push_back(p);
        }
    }
}


}//namespace plan
