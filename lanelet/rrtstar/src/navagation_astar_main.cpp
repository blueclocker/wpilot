/*
 * @Author: wpbit
 * @Date: 2023-04-04 15:34:55
 * @LastEditors: wpbit
 * @LastEditTime: 2023-04-07 16:30:36
 * @Description: 
 */
#include "rrtstar/rrtstar.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <opencv4/opencv2/opencv.hpp>
#include <osmmap/CarState.h>
#include <tf/tf.h>

namespace RRT
{
class NavagationAstar
{
struct Astarnode
{
    int data_;//0可走，1不可走
    vector2i gridxy_;
    double g_;
    double h_;
    double f_;
    Astarnode *parent_;
    bool isclose_;
    bool isopen_;
    Astarnode() = default;
    explicit Astarnode(const vector2i &p)
    {
        data_ = 0;
        gridxy_ = p;
        g_ = std::numeric_limits<double>::max();
        h_ = std::numeric_limits<double>::max();
        f_ = std::numeric_limits<double>::max();
        parent_ = nullptr;
        isclose_ = false;
        isopen_ = false;
    }
};

struct cmp
{
    bool operator() (const Astarnode *a, const Astarnode *b)
    {
        return a->f_ > b->f_;//小顶堆
    }
};


private:
    ros::NodeHandle nh_;
    vector2d start_point_;
    vector2d goal_point_;
    Astarnode *goal_node_;
    geometry_msgs::Pose endPose_;
    nav_msgs::OccupancyGrid map_;
    std::vector<std::vector<Astarnode*>> map_data_;
    std::string map_path_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    double origin_z_;
    int isobs_;
    vector2d pxy_lower_, pxy_upper_;
    vector2i gridsize_;
    double wightw_ = 1.0;//水平移动代价
    double wighth_ = 1.4;//斜向移动代价
    std::priority_queue<Astarnode*, std::vector<Astarnode*>, cmp> openlist_;
    nav_msgs::Path gpspath_;
    ros::Publisher path_pub_;
    ros::Publisher map_pub_;
    ros::Publisher carstate_pub_;
    ros::Publisher gpspath_pub_;
    ros::Subscriber goal_point_sub_;
    ros::Subscriber localization_sub_;
    std::vector<vector2d> path_;
    bool start_valid_, goal_valid_, map_valid_;
public:
    NavagationAstar(ros::NodeHandle &nh);
    ~NavagationAstar() 
    {
        for(int i = 0; i < map_data_.size(); ++i)
        {
            for(int j = 0; j < map_data_[i].size(); ++j)
            {
                if(map_data_[i][j] != nullptr)
                {
                    delete map_data_[i][j];
                    map_data_[i][j] = nullptr;
                }
            }
        }
    }
    void goalPointCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void localizationCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    void readMap(const std::string &map_path);
    vector2d grid2pxy(const vector2i &p);
    vector2i pxy2grid(const vector2d &p);
    bool isObstacle(const vector2d &p);
    bool isPointValid(const vector2d &p);
    void setObstacle(const vector2d &p);
    bool search(vector2d start_point, vector2d goal_point);
    void visualizationPath();
    void reset();
    std::vector<vector2d> getPath();
    double calcG(const Astarnode* last_node, const Astarnode* current_node);
    double calcH(const Astarnode* current_node);
    double calcF(const Astarnode* current_node);
    void run();
};

NavagationAstar::NavagationAstar(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.getParam("map_path", map_path_);
    nh_.getParam("resolution", resolution_);
    nh_.getParam("origin_x", origin_x_);
    nh_.getParam("origin_y", origin_y_);
    nh_.getParam("origin_z", origin_z_);
    nh_.getParam("isobs", isobs_);
    map_valid_ = false;
    start_valid_ = false;
    //test
    start_valid_ = true;
    start_point_.x_ = 0;
    start_point_.y_ = 0;
    //test
    goal_valid_ = false;
    readMap(map_path_);
    goal_point_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &NavagationAstar::goalPointCallBack, this);
    localization_sub_ = nh_.subscribe("/mapping_odometry", 1, &NavagationAstar::localizationCallBack, this);
    path_pub_ = nh_.advertise<visualization_msgs::Marker>("/navagation_node/golbalpath_info", 1);
    carstate_pub_ = nh_.advertise<osmmap::CarState>("/navagation_node/carstate_info", 1);
    gpspath_pub_ = nh_.advertise<nav_msgs::Path>("/navagation_node/gpspath_info", 1);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/navagation_node/OccupancyGridMap", 1);
}

void NavagationAstar::run()
{
    ros::Rate r(10);
    while(nh_.ok())
    {
        map_.header.stamp = ros::Time::now();
        map_pub_.publish(map_);
        visualizationPath();
        ros::spinOnce();
        r.sleep();
    }
}

void NavagationAstar::reset()
{
    for(int i = 0; i < map_data_.size(); ++i)
    {
        for(int j = 0; j < map_data_[i].size(); ++j)
        {
            map_data_[i][j]->g_ = std::numeric_limits<double>::max();
            map_data_[i][j]->h_ = std::numeric_limits<double>::max();
            map_data_[i][j]->f_ = std::numeric_limits<double>::max();
            map_data_[i][j]->parent_ = nullptr;
            map_data_[i][j]->isclose_ = false;
            map_data_[i][j]->isopen_ = false;
        }
    }
    while(!openlist_.empty()) openlist_.pop();
}

std::vector<vector2d> NavagationAstar::getPath()
{
    std::vector<vector2d> path;
    Astarnode *ptr = goal_node_;
    while(ptr != nullptr)
    {
        path.emplace_back(grid2pxy(ptr->gridxy_));
        ptr = ptr->parent_;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

double NavagationAstar::calcG(const Astarnode* last_node, const Astarnode* current_node)
{
    double incleaseg = (std::abs(current_node->gridxy_.gridx_ - last_node->gridxy_.gridx_) + 
                     std::abs(current_node->gridxy_.gridy_ - last_node->gridxy_.gridy_)) 
                     == 1 ? wightw_ : wighth_;
    double parentg = last_node->parent_ == nullptr ? 0 : last_node->parent_->g_;
    return incleaseg + parentg;
}

double NavagationAstar::calcH(const Astarnode* current_node)
{
    double tempx = 1.0*(goal_node_->gridxy_.gridx_ - current_node->gridxy_.gridx_);
    double tempy = 1.0*(goal_node_->gridxy_.gridy_ - current_node->gridxy_.gridy_);
    return std::sqrt(tempx * tempx + tempy * tempy) * wightw_;
}

double NavagationAstar::calcF(const Astarnode* current_node)
{
    return current_node->g_ + current_node->h_;
}

void NavagationAstar::goalPointCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_point_.x_ = msg->pose.position.x;
    goal_point_.y_ = msg->pose.position.y;
    endPose_ = msg->pose;
    if(isPointValid(goal_point_)) 
    {
        goal_valid_ = true;
        ROS_INFO("get goal point x: %f, y: %f", goal_point_.x_, goal_point_.y_);

        if(!start_valid_ || !map_valid_) 
        {
            ROS_WARN("start point invalid or map invalid!");
            return;
        }

        ROS_INFO("astar searching");
        if(search(start_point_, goal_point_))
        {
            path_ = getPath();
            // reset();
            ROS_INFO("astar find!");
        }else{
            ROS_INFO("astar failed!");
        }
    }else{
        ROS_WARN("invalid goal point!");
    }
}

void NavagationAstar::localizationCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    start_point_.x_ = msg->pose.pose.position.x;
    start_point_.y_ = msg->pose.pose.position.y;
    if(isPointValid(start_point_)) start_valid_ = true;
    ROS_INFO("get start point x: %f, y: %f", start_point_.x_, start_point_.y_);

    tf::Quaternion qenu2base;
    qenu2base.setW(msg->pose.pose.orientation.w);
    qenu2base.setX(msg->pose.pose.orientation.x);
    qenu2base.setY(msg->pose.pose.orientation.y);
    qenu2base.setZ(msg->pose.pose.orientation.z);

    //carstate
    osmmap::CarState hdmapstate;
    hdmapstate.header.frame_id = "map";
    hdmapstate.header.stamp = ros::Time::now();
    hdmapstate.inMap = false;
    hdmapstate.isEndExist = false;
    hdmapstate.isFindPath = false;
    hdmapstate.laneletid = 0;
    hdmapstate.heading = tf::getYaw(qenu2base);
    hdmapstate.carPose.position.x = start_point_.x_;
    hdmapstate.carPose.position.y = start_point_.y_;
    hdmapstate.carPose.position.z = origin_z_;
    hdmapstate.carPose.orientation.x = qenu2base.x();
    hdmapstate.carPose.orientation.y = qenu2base.y();
    hdmapstate.carPose.orientation.z = qenu2base.z();
    hdmapstate.carPose.orientation.w = qenu2base.w();
    hdmapstate.endPose = endPose_;
    hdmapstate.nextpoint.x = 0;
    hdmapstate.nextpoint.y = 0;
    hdmapstate.nextpoint.z = 0;
    //角速度、线速度
    hdmapstate.linear.x = msg->twist.twist.linear.x;
    hdmapstate.linear.y = msg->twist.twist.linear.y;
    hdmapstate.linear.z = msg->twist.twist.linear.z;
    hdmapstate.angular.x = msg->twist.twist.angular.x;
    hdmapstate.angular.y = msg->twist.twist.angular.y;
    hdmapstate.angular.z = msg->twist.twist.angular.z;
    //暂时加速度均给0，规划未用到
    hdmapstate.Accell.x = 0;
    hdmapstate.Accell.y = 0;
    hdmapstate.Accell.z = 0;
    carstate_pub_.publish(hdmapstate);

    //gps path
    gpspath_.header.frame_id = "map";
    gpspath_.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = start_point_.x_;
    pose_stamped.pose.position.y = start_point_.y_;
    pose_stamped.pose.position.z = origin_z_;
    pose_stamped.pose.orientation.x = qenu2base.x();
    pose_stamped.pose.orientation.y = qenu2base.y();
    pose_stamped.pose.orientation.z = qenu2base.z();
    pose_stamped.pose.orientation.w = qenu2base.w();
    gpspath_.poses.push_back(pose_stamped);
    gpspath_pub_.publish(gpspath_);
}

vector2d NavagationAstar::grid2pxy(const vector2i &p)
{
    vector2d point;
    point.x_ = ((double)p.gridx_ + 0.5) * resolution_ + pxy_lower_.x_; //origin_.x_;
    point.y_ = ((double)p.gridy_ + 0.5) * resolution_ + pxy_lower_.y_; //origin_.y_;
    return point;
}

vector2i NavagationAstar::pxy2grid(const vector2d &p)
{
    vector2i point;
    // point.gridx_ = std::min(std::max(int((p.x_ - origin_.x_) / resolution_), 0), grid_size_.gridx_ - 1);
    // point.gridy_ = std::min(std::max(int((p.y_ - origin_.y_) / resolution_), 0), grid_size_.gridy_ - 1);
    point.gridx_ = std::min(std::max(int((p.x_ - pxy_lower_.x_) / resolution_), 0), gridsize_.gridx_ - 1);
    point.gridy_ = std::min(std::max(int((p.y_ - pxy_lower_.y_) / resolution_), 0), gridsize_.gridy_ - 1);
    return point;
}

bool NavagationAstar::isObstacle(const vector2d &p)
{
    vector2i gridxy = pxy2grid(p);
    //std::cout << "x: " << gridxy.gridx_ << ", y: " << gridxy.gridy_ << " ";
    //std::cout << map_data_[gridxy.gridx_][gridxy.gridy_]->data_ << std::endl;
    return map_data_[gridxy.gridx_][gridxy.gridy_]->data_;
}

bool NavagationAstar::isPointValid(const vector2d &p)
{
    if(p.x_ < pxy_lower_.x_ || p.x_ >= pxy_upper_.x_ ||
       p.y_ < pxy_lower_.y_ || p.y_ >= pxy_upper_.y_) return false;

    return !isObstacle(p);
}

void NavagationAstar::setObstacle(const vector2d &p)
{
    if(p.x_ < pxy_lower_.x_ || p.x_ >= pxy_upper_.x_ ||
       p.y_ < pxy_lower_.y_ || p.y_ >= pxy_upper_.y_) return;

    vector2i gridxy = pxy2grid(p);
    map_data_[gridxy.gridx_][gridxy.gridy_]->data_ = 1;
}

bool NavagationAstar::search(vector2d start_point, vector2d goal_point)
{
    reset();
    // goal_node_ = new Astarnode(pxy2grid(goal_point));
    goal_node_ = map_data_[pxy2grid(goal_point).gridx_][pxy2grid(goal_point).gridy_];
    // Astarnode *start_node = new Astarnode(pxy2grid(start_point));
    Astarnode *start_node = map_data_[pxy2grid(start_point).gridx_][pxy2grid(start_point).gridy_];
    start_node->g_ = 0;
    start_node->h_ = calcH(start_node);
    start_node->f_ = calcF(start_node);
    openlist_.push(start_node);
    start_node->isopen_ = true;
    while(!openlist_.empty())
    {
        Astarnode *curnode = openlist_.top();
        openlist_.pop();
        curnode->isclose_ = true;
        // std::cout << curnode->h_ << std::endl;
        for(int startx = curnode->gridxy_.gridx_ - 1; startx <= curnode->gridxy_.gridx_ + 1; ++startx)
        {
            for(int starty = curnode->gridxy_.gridy_ - 1; starty <= curnode->gridxy_.gridy_ + 1; ++starty)
            {
                if(startx < 0 || startx >= gridsize_.gridx_ ||
                   starty < 0 || starty >= gridsize_.gridy_ ||
                   map_data_[startx][starty]->data_ == 1||
                   (startx == curnode->gridxy_.gridx_ && starty == curnode->gridxy_.gridy_) ||
                   map_data_[startx][starty]->isclose_)
                {
                    continue;
                }
                if(!map_data_[startx][starty]->isopen_)
                {
                    map_data_[startx][starty]->parent_ = curnode;
                    map_data_[startx][starty]->g_ = calcG(curnode, map_data_[startx][starty]);
                    map_data_[startx][starty]->h_ = calcH(map_data_[startx][starty]);
                    map_data_[startx][starty]->f_ = calcF(map_data_[startx][starty]);
                    map_data_[startx][starty]->isopen_ = true;
                    openlist_.push(map_data_[startx][starty]);
                }else{
                    double tempg = calcG(curnode, map_data_[startx][starty]);
                    if(tempg > map_data_[startx][starty]->g_)
                    {
                        continue;
                    }else{
                        map_data_[startx][starty]->parent_ = curnode;
                        map_data_[startx][starty]->g_ = tempg;
                        map_data_[startx][starty]->f_ = calcF(map_data_[startx][starty]);
                    }
                }
                //find
                if(startx == goal_node_->gridxy_.gridx_ && starty == goal_node_->gridxy_.gridy_)
                {
                    // std::cout << "Find!" << std::endl;
                    goal_node_->parent_ = curnode;
                    return true;
                }
            }
        }
    }
    // std::cout << "Not Find!" << std::endl;
    return false;
}

void NavagationAstar::readMap(const std::string &map_path)
{
    cv::Mat image = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    int rows = image.rows;
    int cols = image.cols;
    std::cout << "rows: " << rows << ", cols: " << cols << std::endl;
    if(image.data == nullptr) 
    {
        ROS_WARN("read map failed!");
        return;
    }
    map_.header.frame_id = "map";
    map_.info.origin.position.x = origin_x_;
    map_.info.origin.position.y = origin_y_;
    map_.info.origin.position.z = origin_z_;
    map_.info.origin.orientation.w = 1.0;
    map_.info.resolution = resolution_;
    map_.info.width = rows;
    map_.info.height = cols;
    map_.data.resize(rows * cols);
    
    pxy_lower_.x_ = origin_x_;
    pxy_lower_.y_ = origin_y_;
    pxy_upper_.x_ = pxy_lower_.x_ + resolution_ * rows;
    pxy_upper_.y_ = pxy_lower_.y_ + resolution_ * cols;
    gridsize_.gridx_ = rows;
    gridsize_.gridy_ = cols;
    // ROS_INFO("map ld point x: %f, y: %f", originxylow.x_, originxylow.y_);
    // ROS_INFO("map ru point x: %f, y: %f", originxyup.x_, originxyup.y_);
    
//     索引号沿着x轴方向逐行累加,取值如下图所示:
//                                ^
//     ---------------------------| x
//     |   |        ...       |w-1|  
//     |---|----------------------|
//     |            ...           |  
//     |---|--------------|---|---|
//     |   |        ...   |w+1| 1 |
//     |---|--------------|---|---|
//     |   |        ...   | w | 0 |
//  <-----------------------------|---
//     y                     OccupacyGrid_origin

    map_data_.resize(gridsize_.gridx_);
    for(int i = 0; i < map_data_.size(); ++i)
    {
        map_data_[i].resize(gridsize_.gridy_);
    }
    // memset(map_data_, 0, gridsize_.gridx_ * gridsize_.gridy_*sizeof(Astarnode));
    for(int x = 0; x < rows; ++x)
    {
        unsigned char *data = image.ptr<uchar>(rows - 1 - x);
        for(int y = 0; y < cols; ++y)
        {
            // map_.data[y * rows + x] = 100 - *(data + cols - 1 - y) * 100 / 255;
            // map_data_[cols - 1 - y][x] = new Astarnode(vector2i(cols - 1 - y, x));
            int t = 100 - *(data + cols - 1 - y) * 100 / 255;
            map_data_[x][y] = new Astarnode(vector2i(x, y));
            // if(map_.data[y * rows + x] >= isobs_)
            if(t <= isobs_)
            {
                // vector2d obsp;
                // obsp.x_ = x * resolution_ + origin_x_;
                // obsp.y_ = y * resolution_ + origin_y_;
                // setObstacle(obsp);
                // map_data_[cols - 1 - y][x]->data_ = 1;
                map_data_[x][y]->data_ = 1;
                map_.data[y * rows + x] = 100;
            }
        }
    }
    map_valid_ = true;
    ROS_INFO("map init successful!");
}

void NavagationAstar::visualizationPath()
{
    visualization_msgs::Marker maker;
    maker.header.frame_id = "map";
    maker.header.stamp = ros::Time::now();
    maker.type = visualization_msgs::Marker::LINE_STRIP;
    maker.action = visualization_msgs::Marker::ADD;
    maker.ns = "astarpath";
    maker.color.a = 1.0;
    maker.color.g = 1.0;
    maker.pose.orientation.w = 1.0;
    maker.scale.x = 0.25;
    maker.id = 1;
    for(int i = 0; i < path_.size(); ++i)
    {
        geometry_msgs::Point pos;
        pos.x = path_[i].x_;
        pos.y = path_[i].y_;
        pos.z = origin_z_;
        maker.points.push_back(pos);
    }
    path_pub_.publish(maker);
}


}//namespace RRT


int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n("~");
    RRT::NavagationAstar navagationastar(n);
    navagationastar.run();
    return 0;
}