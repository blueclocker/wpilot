#include "pure_pursuit/pure_pursuit.h"
#include <tf/tf.h>
#include <Eigen/Dense>

vec_control::PurePursuit::PurePursuit()
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private("~");
  // Node paramters
  nh_private.param<double>("ld_gain", ld_gain_, 1.0);
  nh_private.param<double>("min_ld", min_ld_, 0.22);//0.5
  nh_private.param<double>("car_wheel_base", car_wheel_base_, 2.835);//0.44
  nh_private.param<int>("controller_freq", controller_freq_, 10);
  nh_private.param<std::string>("map_frame", map_frame_, "map");
  nh_private.param<std::string>("base_frame", base_frame_, "base_link");
  nh_private.param<double>("target_speed", target_speed_, 5.0);
  ld_ = min_ld_;
  heading_ = 101.0;
  ros_rate_ = new ros::Rate(controller_freq_);
  // Publishers and subscribers
  control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/pure_pursuit/control", 1);
  ros::Subscriber odom_sub_ = nh_.subscribe("/odom", 1, &PurePursuit::odom_clk_, this);
  ros::Subscriber path_sub_ = nh_.subscribe("/planning/frenet_path", 1, &PurePursuit::path_clk_, this);
  // tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
  l_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/pure_pursuit/lookahead_point", 1);
  lgsvl_pub_ = nh_.advertise<lgsvl_msgs::VehicleControlData>("/vehicle_cmd", 1);

  // main loop
  control_loop_();
}

void vec_control::PurePursuit::odom_clk_(const nav_msgs::Odometry::ConstPtr &msg)
{
  car_state_mutex_.lock();
  car_speed_ = msg->twist.twist.linear.x;
  Eigen::Quaterniond q_q_a = Eigen::Quaterniond(0.609886348247528, 0.0135536137968302, 0.00340890442021191, -0.792365729808807).normalized();
  tf::Quaternion qenu2base;
  qenu2base.setW(msg->pose.pose.orientation.w);
  qenu2base.setX(msg->pose.pose.orientation.x);
  qenu2base.setY(msg->pose.pose.orientation.y);
  qenu2base.setZ(msg->pose.pose.orientation.z);
  Eigen::Quaterniond q1 = Eigen::Quaterniond(qenu2base.w(), qenu2base.x(), qenu2base.y(), qenu2base.z()).normalized();
  Eigen::Quaterniond q = (q1 * q_q_a.inverse()).normalized();
  qenu2base.setW(q.w());
  qenu2base.setX(q.x());
  qenu2base.setY(q.y());
  qenu2base.setZ(q.z());
  heading_ = tf::getYaw(qenu2base);
  car_state_mutex_.unlock();
  ld_ = std::max(ld_gain_ * car_speed_, min_ld_);
}

void vec_control::PurePursuit::path_clk_(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
  if(msg->markers.empty()) 
  {
    ROS_INFO("path is empty!");
    return;
  }
  ROS_INFO("New path is received.");
  // path_ = msg->poses;
  path_ = msg->markers.front().points;
  // path_.push_back(msg->poses[0]);
  got_path_ = true;
  path_done_ = false;
  point_idx_ = 0;
  // double start_end_dist = distance(path_[0].pose.position, path_.back().pose.position);
  double start_end_dist = distance(path_[0], path_.back());
  ROS_INFO("Start to End Distance: %f", start_end_dist);
  ROS_INFO("Min lookup distance: %f", min_ld_);
  if (start_end_dist <= min_ld_)
  {
    loop_ = true;
    ROS_INFO("Is Loop: True");
  }
}

void vec_control::PurePursuit::control_loop_()
{
  double y_t = 0, ld_2 = 0, delta = 0;
  double distance_ = 0;
  while (ros::ok())
  {
    car_state_mutex_.lock();
    if (got_path_ && heading_ < 100.0)
    {
      // get the current robot location by tf base_link -> map
      // iterate over the path points
      // if the distance between a point and robot > lookahead break and take
      // this point transform this point to the robot base_link the y component
      // of this point is y_t delta can be computed as atan2(2 * yt * L_, ld_2)
      // try
      // {
        // base_location_ = tfBuffer_.lookupTransform(
            // map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));

        for (; point_idx_ < path_.size(); point_idx_++)
        {
          // distance_ = distance(path_[point_idx_].pose.position, base_location_.transform.translation);
          distance_ = distance(path_[point_idx_], path_[0]);
          // ROS_INFO("Point ID: %d, Distance %f", point_idx_, distance_);
          if (distance_ >= ld_)
          {
            // path_[point_idx_].header.stamp = ros::Time::now(); 
                                  // Set the timestamp to now for the transform
                                  // to work, because it tries to transform the
                                  // point at the time stamp of the input point
            // tfBuffer_.transform(path_[point_idx_], target_point_, base_frame_,
            //                     ros::Duration(0.1));
            break;
          }
        }
        // Calculate the steering angle
        ld_2 = ld_ * ld_;
        // y_t = target_point_.pose.position.y;
        y_t = path_[point_idx_].y - path_[0].y;
        delta = atan2(2 * car_wheel_base_ * y_t, ld_2) - heading_;
        control_msg_.drive.steering_angle = delta;
        control_msg_.drive.speed = 2;
        control_msg_.header.stamp = ros::Time::now();
        control_pub_.publish(control_msg_);
        //lgsvl
        lgsvl_msg_.target_gear = lgsvl_msgs::VehicleControlData::GEAR_DRIVE;
        lgsvl_msg_.header = control_msg_.header;
        double acc = (target_speed_ * target_speed_ - car_speed_ * car_speed_) / (2 * distance_);
        if(acc > 0)
        {
          lgsvl_msg_.acceleration_pct = std::min(acc, 1.0);
          lgsvl_msg_.braking_pct = 0;
        }else{
          lgsvl_msg_.acceleration_pct = 0;
          lgsvl_msg_.braking_pct = std::min(fabs(acc), 1.0);
        }
        lgsvl_msg_.target_wheel_angular_rate = 0.4;
        double target_wheel_angle = std::min(0.8, std::fabs(delta));
        lgsvl_msg_.target_wheel_angle = delta < 0 ? target_wheel_angle : -target_wheel_angle;
        std::cout << "wheel angle: " << lgsvl_msg_.target_wheel_angle  << ", heading: " << heading_ << std::endl;

        last_p_idx_ = point_idx_;
        last_dist_ = distance_;
        if (point_idx_ == path_.size() && loop_)
        {
          point_idx_ = 0;
        }
        else if (point_idx_ == path_.size())
        {
          ROS_INFO("Reached final point");
          control_msg_.drive.steering_angle = 0;
          control_msg_.drive.speed = 0;
          control_msg_.header.stamp = ros::Time::now();
          control_pub_.publish(control_msg_);
          got_path_ = false;
          point_idx_ = 0;

          //lgsvl
          double brake = (- car_speed_ * car_speed_) / (2 * distance_);
          lgsvl_msg_.acceleration_pct = 0;
          lgsvl_msg_.braking_pct = std::min(fabs(brake), 2.0);
          lgsvl_msg_.target_wheel_angle = 0;
          if(std::fabs(car_speed_) < 0.5) lgsvl_msg_.target_gear = lgsvl_msgs::VehicleControlData::GEAR_PARKING;
        }
        lookahead_p.point = path_[point_idx_];
        lookahead_p.header.frame_id = "map";
        lookahead_p.header.stamp = ros::Time::now();
        l_point_pub_.publish(lookahead_p); // Publish the lookahead point
        lgsvl_pub_.publish(lgsvl_msg_);
      // }
      // catch (tf2::TransformException &ex)
      // {
      //   ROS_WARN("%s", ex.what());
      // }
    }else{
      // lgsvl_msg_.target_gear = lgsvl_msgs::VehicleControlData::GEAR_PARKING;
      // lgsvl_msg_.header.frame_id = "map";
      // lgsvl_msg_.header.stamp = ros::Time::now();
      // lgsvl_msg_.acceleration_pct = 0.0;
      // lgsvl_msg_.braking_pct = 1.0;
    }
    car_state_mutex_.unlock();

    ros::spinOnce();
    ros_rate_->sleep();
  }
}

vec_control::PurePursuit::~PurePursuit()
{
  // delete tfListener_;
  delete ros_rate_;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  vec_control::PurePursuit pp_node;

  return 0;
}