/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:36
 * @LastEditTime: 2022-04-09 20:13:15
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_io.h
 */
#ifndef MAP_IO_H_
#define MAP_IO_H_

#include "map_node.h"
#include "map_way.h"
#include "map_relation.h"
#include "centerway.h"
#include "visualization.h"
#include "map_plan.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <osmmap/navigation.h>
#include <osmmap/regulatoryelement.h>

/*地图读取/可视化模块
* 调用node, way, relation, centerway四大类实现地图解析
* 调用visualization实现可视化
*/

namespace map
{
class Map
{
private:
    ros::NodeHandle n;
    TiXmlElement *node_pin;
    TiXmlElement *way_pin;
    TiXmlElement *relation_pin;
    node::Node *nodes;
    way::Way *ways;
    relation::Relation *relations;
    centerway::CenterWay *centerways;
    MapVisualization *visualmap;
    plan::Globalplan *globalplans;

    //params
    std::string file_path_;
    std::string file_name_;
    //起点所在路段id
    int start_path_;
    //终点所在路段id
    int end_path_;
    //起点的前一个中心线点id
    int start_centerpoint_id_;
    //终点的下一个中心线点id
    int end_centerpoint_id_;
    //是否存在有效起点
    bool isstart_path_exist;
    //是否存在有效终点
    bool isend_path_exist;
    //规划结果，得到的道路中心线id
    std::vector<int> paths;
    static constexpr double origin_lat = 40.0383159;
    static constexpr double origin_lon = 116.2530763;
    static constexpr double origin_ele = 50.89;//50.8932991028
    node::Point3D *basicpoint;//基准点绝对坐标
    node::Point3D *atnowpoint;//当前点相对坐标
    std::vector<centerway::CenterPoint3D> smoothpathnode;

    //ros
    ros::Subscriber startpoint_sub;
    ros::Subscriber gps_pub;
    ros::Subscriber goalpoint_sub;
    osmmap::navigation laneletinfo;
    ros::Publisher navigation_pub;
    tf::TransformBroadcaster broadcaster;
    tf::Transform baselink2map;
    void fullNavigationInfo();
public:
    Map(ros::NodeHandle n_);
    ~Map();
    void Smoothpath(std::vector<int> pathid_);
    void startpoint_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void goalpoint_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
};



};//namespace map

#endif