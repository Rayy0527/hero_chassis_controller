//
// Created by rmray on 24-1-14.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H


#define ENCODER_RESOLUTION      1440.0   //编码器分辨率, 轮子转一圈，编码器产生的脉冲数
#define WHEEL_DIAMETER          0.1525    //轮子直径,单位：米
#define D_X                     0.4     //底盘Y轴上两轮中心的间距
#define D_Y                     0.4     //底盘X轴上两轮中心的间距
#define PID_RATE                50       //PID调节PWM值的频率
#define ENCODER_MAX 32767
#define ENCODER_MIN -32768
#define ENCODER_LOW_WRAP  ((ENCODER_MAX - ENCODER_MIN)*0.3+ENCODER_MIN)
#define ENCODER_HIGH_WRAP ((ENCODER_MAX - ENCODER_MIN)*0.7+ENCODER_MIN)
#define PI 3.1415926


#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_msgs/JointControllerState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


namespace Controller {
    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        HeroChassisController() = default;

        ~HeroChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

        ros::Subscriber cmd_vel_sub;
        ros::Publisher odom_pub;
        tf::TransformBroadcaster odom_broadcaster;
        ros::Time current_time;
        ros::Time last_time;

        double x = 0.0;
        double y = 0.0;
        double w = 0.0;

        void Kinematics_Init(void);


        static void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr &msg);

//        static tf::TransformBroadcaster vel_broadcaster;
//        static tf::TransformListener vel_listener;
//        std::string vel_mode;
//        static ros::Subscriber vel_odom_sub;


        double Kinematics_Inverse(double *input);

    private:
        int state_{};
        double pulse_per_meter = 0;
        double rx_plus_ry_cali = 0.3;
        double angular_correction_factor = 1.0;
        double linear_correction_factor = 1.0;

        ros::Time last_change_;
        geometry_msgs::Twist vel_msgs;

    };
}
#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H