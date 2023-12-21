//
// Created by rmray on 23-12-20.
//
#include "simple_chassis_controller/simple_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <control_msgs/JointControllerState.h>
#include <control_msgs/PidState.h>
#include <std_msgs/Float64.h>

class PIDController {
private:
    double kp_; // Proportional gain
    double ki_; // Integral gain
    double kd_; // Derivative gain
    double target_velocity_; // Target velocity
    double current_velocity_; // Current velocity
    double error_sum_; // Sum of errors
    double previous_error_; // Previous error

    ros::NodeHandle nh_;
    ros::Publisher control_pub_;

public:
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {
        control_pub_ = nh_.advertise<std_msgs::Float64>("/control_command", 10);
    }

    void velocityCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_velocity_ = msg->data;
        double error = target_velocity_ - current_velocity_;

        // Proportional term
        double p_term = kp_ * error;

        // Integral term
        error_sum_ += error;
        double i_term = ki_ * error_sum_;

        // Derivative term
        double d_term = kd_ * (error - previous_error_);
        previous_error_ = error;

        // Total control command
        double control_command = p_term + i_term + d_term;

        // Publish control command
        std_msgs::Float64 control_msgs;
        control_msgs.data = control_command;
        control_pub_.publish(control_msgs);
    }

    void setTargetVelocity(double target_velocity) {
        target_velocity_ = target_velocity;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pid_controller_node");

    control_toolbox::Pid pid;
    pid.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
    double position_desi_ = 0.5;

    // Create a PIDController object with desired gains
    PIDController pid_controller(0.5, 0.1, 0.2);

    // Set the target velocity
    pid_controller.setTargetVelocity(1.0);

    // Subscribe to the current velocity topic
    ros::NodeHandle nh;
    ros::Subscriber velocity_sub = nh.subscribe("/current_velocity", 10, &PIDController::velocityCallback, &pid_controller);

    // Spin the node
    ros::spin();

    return 0;
}