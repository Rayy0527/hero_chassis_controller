//
// Created by rmray on 24-1-15.
//
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

control_toolbox::Pid pid_front_left;
control_toolbox::Pid pid_front_right;
control_toolbox::Pid pid_back_left;
control_toolbox::Pid pid_back_right;

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                 ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
    front_left_joint_ =
            effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_ =
            effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_ =
            effort_joint_interface->getHandle("right_back_wheel_joint");

    double p, i, d, i_max, i_min;
    controller_nh.getParam("pid/p", p);
    controller_nh.getParam("pid/i", i);
    controller_nh.getParam("pid/d", d);
    controller_nh.getParam("pid/i_min", i_min);
    controller_nh.getParam("pid/i_max", i_max);

    pid_front_left.initPid(p, i, d, i_max, i_min);
    pid_front_right.initPid(p, i, d, i_max, i_min);
    pid_back_left.initPid(p, i, d, i_max, i_min);
    pid_back_right.initPid(p, i, d, i_max, i_min);

    return true;
}



//
//void control_toolbox::Pid::initPid(double p, double i, double d, double i_max, double i_min,
//                  const ros::NodeHandle& /*node*/)
//{
//    initPid(p, i, d, i_max, i_min);
//
//    // Create node handle for dynamic reconfigure
//    ros::NodeHandle nh(DEFAULT_NAMESPACE);
//    initDynamicReconfig(nh);
//}
//
//
//double control_toolbox::Pid::computeCommand(double error, ros::Duration dt)
//{
//
//    if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
//        return 0.0;
//
//    double error_dot = d_error_;
//
//    // Calculate the derivative error
//    if (dt.toSec() > 0.0)
//    {
//        if (valid_p_error_last_) {
//            error_dot = (error - p_error_last_) / dt.toSec();
//        }
//        p_error_last_ = error;
//        valid_p_error_last_ = true;
//    }
//
//    return computeCommand(error, error_dot, dt);
//}
//

void HeroChassisController::Kinematics_Init()
{
    pulse_per_meter =
            (float) (ENCODER_RESOLUTION / (WHEEL_DIAMETER * 3.1415926)) / linear_correction_factor;

    float r_x = D_X / 2;
    float r_y = D_Y / 2;
    rx_plus_ry_cali = (r_x + r_y) / angular_correction_factor;
}

void HeroChassisController::Kinematics_Inverse(float *input, float *output)
{
    float r_x = D_X / 2;
    float r_y = D_Y / 2;
    float v_tx = (float) input[0];
    float v_ty = (float) input[1];
    float omega = (float) input[2];

    static float v_w[4] = {0};

    v_w[0] = v_tx - v_ty - (r_x + r_y) * omega;
    v_w[1] = v_tx + v_ty + (r_x + r_y) * omega;
    v_w[2] = v_tx + v_ty - (r_x + r_y) * omega;
    v_w[3] = v_tx - v_ty + (r_x + r_y) * omega;

    output[0] =  (v_w[0] * pulse_per_meter / PID_RATE);
    output[1] =  (v_w[1] * pulse_per_meter / PID_RATE);
    output[2] =  (v_w[2] * pulse_per_meter / PID_RATE);
    output[3] =  (v_w[3] * pulse_per_meter / PID_RATE);
}

geometry_msgs::Twist CmdVel;

float cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    CmdVel = *msg;
}

float linear_vx = CmdVel.linear.x;
float linear_vy = CmdVel.linear.y;
float angular_w = CmdVel.angular.z;
float vel[] = {linear_vx, linear_vy, angular_w};
float *v = vel;

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period)
{
    double realtime_front_left = front_left_joint_.getVelocity();
    double realtime_front_right = front_right_joint_.getVelocity();
    double realtime_back_left = back_left_joint_.getVelocity();
    double realtime_back_right = back_right_joint_.getVelocity();

    float desired_vel[4];
    Kinematics_Inverse(v, &desired_vel[0]);

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

PLUGINLIB_EXPORT_CLASS(HeroChassisController, controller_interface::ControllerBase)