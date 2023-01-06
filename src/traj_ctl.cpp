#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ros/callback_queue.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"

using namespace std;

#define SIM 0

enum MISSION_STATE
{
    INIT,
    POSITION,  // TAKEOFF
    LAND
};

MISSION_STATE mission_state;

mavros_msgs::State px4_state, px4_state_prev;

#define DEAD_ZONE 0.25
#define MAX_MANUAL_VEL 1.0
#define RC_REVERSE_PITCH 0
#define RC_REVERSE_ROLL 0
#define RC_REVERSE_THROTTLE 0

double last_set_hover_pose_time;

ros::Publisher     target_pose_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

//ros::Publisher traj_repub;

//  W: World;V: View; B: Body;
Eigen::Affine3f     T_W_Bt, T_V_Bt_set, T_W_V, T_B_C;
geometry_msgs::Pose T_W_B_set;

bool take_off = false;
bool send_tra = false;
int set_init = 0;
double tra_start_x = 1.049;  //0.894
double tra_start_y = -1.664; //-0.393
double tra_start_z = 0.800;  //1.300

void pose_set_callback(const geometry_msgs::PoseStampedConstPtr &pose_set_msg) {
  double z_take_off = 0.800;
  if (set_init == 1 && fabs(z_take_off - pose_set_msg->pose.position.z) < 0.100) {
    T_W_B_set.position.x = tra_start_x;
    T_W_B_set.position.y = tra_start_y;
    T_W_B_set.position.z = tra_start_z;

    take_off = true;
    set_init++;
  }

if(set_init == 2 
&& fabs(tra_start_x - pose_set_msg->pose.position.x) < 0.150
&& fabs(tra_start_y - pose_set_msg->pose.position.y) < 0.150
&& fabs(tra_start_z - pose_set_msg->pose.position.z) < 0.150){
    ROS_INFO("You can now send the trajectory message!");

    send_tra = true;
    set_init++;
}

  if (set_init == 0 && mission_state != LAND && take_off == false) {
    T_W_B_set.position.x = pose_set_msg->pose.position.x;
    T_W_B_set.position.y = pose_set_msg->pose.position.y;
    T_W_B_set.position.z = z_take_off;
    T_W_B_set.orientation.w = pose_set_msg->pose.orientation.w;
    T_W_B_set.orientation.x = pose_set_msg->pose.orientation.x;
    T_W_B_set.orientation.y = pose_set_msg->pose.orientation.y;
    T_W_B_set.orientation.z = pose_set_msg->pose.orientation.z;

    set_init++;
  }
}

std::deque<ros::Duration> command_waiting_times_;
ros::Timer command_timer_;
double tra_x[10000];
double tra_y[10000];
double tra_z[10000];
double tra_ox[10000];
double tra_oy[10000];
double tra_oz[10000];
double tra_ow[10000];
int j = -1;

void trajectory_set_callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg) 
{
  //traj_repub.publish(msg);
  //ROS_INFO("got traj msg!!!");

  if (send_tra && mission_state == POSITION) {
    const size_t n_commands = msg->points.size();
    if (n_commands < 1) {
      ROS_WARN_STREAM("Got trajectory message, but message has no points.");
      return;
    }

    command_waiting_times_.clear();
    command_timer_.stop();

    for (size_t i = 0; i < n_commands - 1; i++) {
      const trajectory_msgs::MultiDOFJointTrajectoryPoint &current_reference =
          msg->points[i];
      const trajectory_msgs::MultiDOFJointTrajectoryPoint &reference_after =
          msg->points[i + 1];
      tra_x[i] = current_reference.transforms[0].translation.x;
      tra_y[i] = current_reference.transforms[0].translation.y;
      tra_z[i] = current_reference.transforms[0].translation.z;
      tra_ox[i] = current_reference.transforms[0].rotation.x;
      tra_oy[i] = current_reference.transforms[0].rotation.y;
      tra_oz[i] = current_reference.transforms[0].rotation.z;
      tra_ow[i] = current_reference.transforms[0].rotation.w;
      command_waiting_times_.push_back(reference_after.time_from_start -
                                       current_reference.time_from_start);
    }

    if (n_commands > 1) {
      command_timer_.setPeriod(command_waiting_times_.front());
      command_waiting_times_.pop_front();
      command_timer_.start();
    }
    }
}

void time_command_callback(const ros::TimerEvent &e) 
{
    if(send_tra && mission_state == POSITION)
    {
      j++;
      cout << "heading to point " << j << endl;
      T_W_B_set.position.x = tra_x[j];
      T_W_B_set.position.y = tra_y[j];
      T_W_B_set.position.z = tra_z[j];
      T_W_B_set.orientation.x = tra_ox[j];
      T_W_B_set.orientation.y = tra_oy[j];
      T_W_B_set.orientation.z = tra_oz[j];
      T_W_B_set.orientation.w = tra_ow[j];
      command_timer_.stop();

      if(command_waiting_times_.empty()){
        ROS_INFO("The trajectory tracking is over and all points have been passed");
      }
      if (!command_waiting_times_.empty()) {
        command_timer_.setPeriod(command_waiting_times_.front());
        command_waiting_times_.pop_front();
        command_timer_.start();
      }
    }
}

#define BAUDRATE 57600
#define ID 1
#define INIT_RAD -1.57233

int  goal_position;
void pitch_set_callback(const std_msgs::Float32ConstPtr &pitch_set_msg)
{
    float pitch_set = pitch_set_msg->data;
    float radian    = INIT_RAD + pitch_set;
}

void rc_callback(const mavros_msgs::RCInConstPtr &rc_msg)
{
    double rc_ch[4];
    for (int i = 0; i < 4; i++)
    {
        // 归一化遥控器输入
        rc_ch[i] = ((double)rc_msg->channels[i] - 1500.0) / 500.0;
        if (rc_ch[i] > DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (rc_ch[i] < -DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            rc_ch[i] = 0.0;
    }

    if (rc_msg->channels[4] < 1250)
    {
        if (px4_state.mode != "STABILIZED")
        {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "STABILIZED";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Switch to STABILIZED!");
                px4_state_prev      = px4_state;
                px4_state_prev.mode = "STABILIZED";
            }
            else
            {
                ROS_WARN("Failed to enter STABILIZED!");
                return;
            }
        }
        mission_state = INIT;
        cout << "px4 state mode is " << px4_state.mode << endl;
    }
    else if (rc_msg->channels[4] > 1250 && rc_msg->channels[4] < 1750)  // heading to POSITION
    {
        if (mission_state == INIT)
        {
            if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[3] == 0.0)
            {
                mission_state            = POSITION;
                ROS_INFO("Switch to POSITION succeed!");
            }
            else
            {
                ROS_WARN("Switch to POSITION failed! Rockers are not in reset middle!");
                return;
            }
        }
    }

    if (!SIM)
    {
        if (rc_msg->channels[5] > 1750)
        {
            if (mission_state == POSITION)
            {
                if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[3] == 0.0 && !px4_state.armed)
                {
                    if (px4_state.mode != "OFFBOARD")
                    {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        {
                            ROS_INFO("Offboard enabled");
                            px4_state_prev      = px4_state;
                            px4_state_prev.mode = "OFFBOARD";
                        }
                        else
                        {
                            ROS_WARN("Failed to enter OFFBOARD!");
                            return;
                        }
                    }
                    else if (px4_state.mode == "OFFBOARD")
                    {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;

                        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                        }
                        else
                        {
                            ROS_ERROR("Failed to armed");
                            return;
                       }
                    }
                }
                else if (!px4_state.armed)
                {
                    ROS_WARN("Arm denied! Rockers are not in reset middle!");
                    return;
                }
            }
        } else if (rc_msg->channels[5] > 1250 && rc_msg->channels[5] < 1750) {
          if (px4_state_prev.mode == "OFFBOARD") {
            mission_state = LAND;
          }
        } else if (rc_msg->channels[5] < 1250) {
          if (px4_state.armed) {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = false;

            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
              ROS_INFO("Vehicle disarmed");
            } else {
              ROS_ERROR("Failed to disarmed");
              return;
            }
            mission_state = INIT;
            ROS_INFO("Swith to INIT state!");
          }
        }
    }

    if (mission_state != INIT)
    {
        double now               = ros::Time::now().toSec();
        double delta_t           = now - last_set_hover_pose_time;
        last_set_hover_pose_time = now;
        if (mission_state == LAND)
        {
            T_W_B_set.position.z -= 0.3 * delta_t;
        }

        if (T_W_B_set.position.z < -0.3)
            T_W_B_set.position.z = -0.3;
        else if (T_W_B_set.position.z > 1.8)
            T_W_B_set.position.z = 1.8;
    }
}
void px4_state_callback(const mavros_msgs::StateConstPtr &state_msg)
{
    px4_state = *state_msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "nbv_ctl");
    ros::NodeHandle nh("~");
    ros::Rate       rate(30);

   // traj_repub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
     //   "/aaa_traj", 100);

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, px4_state_callback,
                                         ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, rc_callback, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber pose_set_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pose_set_callback);

    ros::Subscriber trajectory_sub = 
        nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/hummingbird/whole_command_trajectory", 1, trajectory_set_callback);



    target_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);

    arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    command_timer_ = nh.createTimer(ros::Duration(0), time_command_callback, true, false);

    mission_state = INIT;

    const char *log;

    while (ros::ok())
    {
        if (mission_state != INIT)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose            = T_W_B_set;
            target_pose_pub.publish(pose);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
