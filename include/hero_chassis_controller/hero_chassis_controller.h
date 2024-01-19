//
// Created by rmray on 24-1-14.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H


#define ENCODER_RESOLUTION      1440.0   //编码器分辨率, 轮子转一圈，编码器产生的脉冲数
#define WHEEL_DIAMETER          0.1525    //轮子直径,单位：米
#define D_X                     0.18     //底盘Y轴上两轮中心的间距
#define D_Y                     0.25     //底盘X轴上两轮中心的间距
#define PID_RATE                50       //PID调节PWM值的频率

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_msgs/JointControllerState.h>


//    class KinematicsInverse
//    {
//    private:
//        double pulse_per_meter = 0;
//        double rx_plus_ry_cali = 0.3;
//        double angular_correction_factor = 1.0;
//        double linear_correction_factor = 1.0;
//
//    public:
//        /**
//      * @函数作用：运动学解析参数初始化
//      */
//        void Kinematics_Init(void)
//        {
//            pulse_per_meter =
//                    (double) (ENCODER_RESOLUTION / (WHEEL_DIAMETER * 3.1415926)) / linear_correction_factor;
//
//            double r_x = D_X / 2;
//            double r_y = D_Y / 2;
//            rx_plus_ry_cali = (r_x + r_y) / angular_correction_factor;
//        }
///**
//  * @函数作用：逆向运动学解析，底盘三轴速度-->轮子速度
//  * @输入：机器人三轴速度 m/s
//  * @输出：电机应达到的目标速度（一个PID控制周期内，电机编码器计数值的变化）
//  */
//        void Kinematics_Inverse(int16_t* input, int16_t* output)
//        {
//            double r_x = D_X / 2;
//            double r_y = D_Y / 2;
//            double v_tx = (double) input[0];
//            double v_ty = (double) input[1];
//            double omega = (double) input[2];
//
//            static double v_w[4] = {0};
//
//            v_w[0] = v_tx - v_ty - (r_x + r_y) * omega;
//            v_w[1] = v_tx + v_ty + (r_x + r_y) * omega;
//            v_w[2] = v_tx + v_ty - (r_x + r_y) * omega;
//            v_w[3] = v_tx - v_ty + (r_x + r_y) * omega;
//
//            output[0] = (int16_t) (v_w[0] * pulse_per_meter / PID_RATE);
//            output[1] = (int16_t) (v_w[1] * pulse_per_meter / PID_RATE);
//            output[2] = (int16_t) (v_w[2] * pulse_per_meter / PID_RATE);
//            output[3] = (int16_t) (v_w[3] * pulse_per_meter / PID_RATE);
//        }
//    };
//
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
//        std::unique_ptr <
//                realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
//        > controller_state_publisher;

        void Kinematics_Init(void);
//        {
//            pulse_per_meter =
//                    (double) (ENCODER_RESOLUTION / (WHEEL_DIAMETER * 3.1415926)) / linear_correction_factor;
//
//            double r_x = D_X / 2;
//            double r_y = D_Y / 2;
//            rx_plus_ry_cali = (r_x + r_y) / angular_correction_factor;
//        }

        static void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr &msg);


        double Kinematics_Inverse(double *input);
//        {
//            double r_x = D_X / 2;
//            double r_y = D_Y / 2;
//            double v_tx = (double) input[0];
//            double v_ty = (double) input[1];
//            double omega = (double) input[2];
//
//            static double v_w[4] = {0};
//
//            v_w[0] = v_tx - v_ty - (r_x + r_y) * omega;
//            v_w[1] = v_tx + v_ty + (r_x + r_y) * omega;
//            v_w[2] = v_tx + v_ty - (r_x + r_y) * omega;
//            v_w[3] = v_tx - v_ty + (r_x + r_y) * omega;
//
//            output[0] = (int16_t) (v_w[0] * pulse_per_meter / PID_RATE);
//            output[1] = (int16_t) (v_w[1] * pulse_per_meter / PID_RATE);
//            output[2] = (int16_t) (v_w[2] * pulse_per_meter / PID_RATE);
//            output[3] = (int16_t) (v_w[3] * pulse_per_meter / PID_RATE);
//        }

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