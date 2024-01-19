//
// Created by rmray on 24-1-15.
//
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace Controller {

    control_toolbox::Pid pid_front_left;
    control_toolbox::Pid pid_front_right;
    control_toolbox::Pid pid_back_left;
    control_toolbox::Pid pid_back_right;

    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {

        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        double p, i, d, i_max, i_min;

        controller_nh.getParam("pid/i_min", i_min);
        controller_nh.getParam("pid/i_max", i_max);
        if (!controller_nh.getParam("/controller/hero_chassis_controller/pid/p", p) ||
            !controller_nh.getParam("/controller/hero_chassis_controller/pid/i", i) ||
            !controller_nh.getParam("/controller/hero_chassis_controller/pid/d", d))
            {
            ROS_ERROR("Failed to get parameters from the parameter server");
            return false;
            }


        cmd_vel_sub = root_nh.subscribe("/cmd_vel", 1, cmdVelCallBack);

        pid_front_left.initPid(p, i, d, i_max, i_min);
        pid_front_right.initPid(p, i, d, i_max, i_min);
        pid_back_left.initPid(p, i, d, i_max, i_min);
        pid_back_right.initPid(p, i, d, i_max, i_min);

        Kinematics_Init();

        return true;
    }



    void HeroChassisController::Kinematics_Init() {
        pulse_per_meter =
                (double) (ENCODER_RESOLUTION / (WHEEL_DIAMETER * 3.1415926)) / linear_correction_factor;

        double r_x = D_X / 2;
        double r_y = D_Y / 2;
        rx_plus_ry_cali = (r_x + r_y) / angular_correction_factor;
    }

    double output[4];

    double HeroChassisController::Kinematics_Inverse(double *input) {
        double r_x = D_X / 2;
        double r_y = D_Y / 2;
        double v_tx = (double) input[0];
        double v_ty = (double) input[1];
        double omega = (double) input[2];

        static double v_w[4] = {0};

        v_w[0] = v_tx - v_ty - (r_x + r_y) * omega;
        v_w[1] = v_tx + v_ty + (r_x + r_y) * omega;
        v_w[2] = v_tx + v_ty - (r_x + r_y) * omega;
        v_w[3] = v_tx - v_ty + (r_x + r_y) * omega;

        output[0] = (v_w[0] * pulse_per_meter / PID_RATE);
        output[1] = (v_w[1] * pulse_per_meter / PID_RATE);
        output[2] = (v_w[2] * pulse_per_meter / PID_RATE);
        output[3] = (v_w[3] * pulse_per_meter / PID_RATE);

        return *output;
    }



    double linear_vx;
    double linear_vy;
    double angular_w;
    double vel[3];


    void HeroChassisController::cmdVelCallBack(const geometry_msgs::Twist::ConstPtr &msg)
    {
        linear_vx = msg->linear.x;
        linear_vy = msg->linear.y;
        angular_w = msg->angular.z;
        vel[0] = linear_vx;
        vel[1] = linear_vy;
        vel[2] = angular_w;
    }


    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
        double realtime_front_left = front_left_joint_.getVelocity();
        double realtime_front_right = front_right_joint_.getVelocity();
        double realtime_back_left = back_left_joint_.getVelocity();
        double realtime_back_right = back_right_joint_.getVelocity();

        double desired_vel[4];
        Kinematics_Inverse(vel);
        desired_vel[0] = output[0];
        desired_vel[1] = output[1];
        desired_vel[2] = output[2];
        desired_vel[3] = output[3];

        double error_front_left = desired_vel[0] - realtime_front_left;
        double error_front_right = desired_vel[1] - realtime_front_right;
        double error_back_left = desired_vel[2] - realtime_back_left;
        double error_back_right = desired_vel[3] - realtime_back_right;

        double command_front_left = pid_front_left.computeCommand(error_front_left, period);
        double command_front_right = pid_front_right.computeCommand(error_front_right, period);
        double command_back_left = pid_back_left.computeCommand(error_back_left, period);
        double command_back_right = pid_back_right.computeCommand(error_back_right, period);

        if ((time - last_change_).toSec() > 2) {
            state_ = (state_ + 1) % 6;
            last_change_ = time;
        }

        front_left_joint_.setCommand(command_front_left);
        front_right_joint_.setCommand(command_front_right);
        back_left_joint_.setCommand(command_back_left);
        back_right_joint_.setCommand(command_back_right);


    }



    PLUGINLIB_EXPORT_CLASS(Controller::HeroChassisController, controller_interface::ControllerBase)
}