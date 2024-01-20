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
        if (!controller_nh.getParam("/Controller/hero_chassis_controller/pid/p", p) ||
            !controller_nh.getParam("/Controller/hero_chassis_controller/pid/i", i) ||
            !controller_nh.getParam("/Controller/hero_chassis_controller/pid/d", d) ||
            !controller_nh.getParam("/Controller/hero_chassis_controller/vel_mode", vel_mode))
            {
            ROS_ERROR("Failed to get parameters from the parameter server");
            return false;
            }

        cmd_vel_sub = root_nh.subscribe("/cmd_vel", 1, cmdVelCallBack);
        odom_pub = root_nh.advertise<nav_msgs::Odometry>("/odom", 50);

        pid_front_left.initPid(p, i, d, i_max, i_min);
        pid_front_right.initPid(p, i, d, i_max, i_min);
        pid_back_left.initPid(p, i, d, i_max, i_min);
        pid_back_right.initPid(p, i, d, i_max, i_min);

        Kinematics_Init();

//        if (vel_mode == "odom")
//        {
//            vel_odom_sub = controller_nh.subscribe<geometry_msgs::Vector3Stamped>("/")
//        }
//
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
    double linear_vz;

    double angular_vx;
    double angular_vy;
    double angular_vz;

    double angular_w;
    double vel[3];


    void HeroChassisController::cmdVelCallBack(const geometry_msgs::Twist::ConstPtr &msg)
    {
        linear_vx = msg->linear.x;
        linear_vy = msg->linear.y;
        linear_vz = msg->linear.z;

        angular_vx = msg->angular.x;
        angular_vy = msg->angular.y;
        angular_vz = msg->angular.z;

        geometry_msgs::Vector3Stamped vel_linear;
        geometry_msgs::Vector3Stamped vel_linear_base_link;
        geometry_msgs::Vector3Stamped vel_angular;
        geometry_msgs::Vector3Stamped vel_angular_base_link;

        if (vel_mode == "base_link")
        {
            try
            {
                vel_listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));

                vel_linear.header.stamp = ros::Time::now();
                vel_linear.header.frame_id = "odom";
                vel_linear_base_link.header.frame_id = "base_link";
                vel_linear.vector.x = linear_vx;
                vel_linear.vector.y = linear_vy;
                vel_linear.vector.z = linear_vz;
                vel_listener.transformVector("base_link", vel_linear, vel_linear_base_link);

                vel_angular.header.stamp = ros::Time::now();
                vel_angular.header.frame_id = "odom";
                vel_angular_base_link.header.frame_id = "base_link";
                vel_angular.vector.x = angular_vx;
                vel_angular.vector.y = angular_vy;
                vel_angular.vector.z = angular_vz;
                vel_listener.transformVector("base_link", vel_angular, vel_angular_base_link);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("Transform exception: %s", ex.what());
            }
            vel[0] = vel_linear_base_link.vector.x;
            vel[1] = vel_linear_base_link.vector.y;
            vel[2] = vel_angular_base_link.vector.z;
        }
        else if (vel_mode == "odom")
        {
            vel[0] = linear_vx;
            vel[1] = linear_vy;
            vel[2] = angular_vz;
        }
        else
        {
            ROS_ERROR("Neither 'odom' nor 'base_link'");
        }
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

        double x = 0.0;
        double y = 0.0;
        double w = 0.0;

        double vx = 0.25 * 0.5 * WHEEL_DIAMETER * (realtime_front_left + realtime_front_right + realtime_back_left + realtime_back_right);
        double vy = 0.25 * 0.5 * WHEEL_DIAMETER * (realtime_front_left + realtime_front_right + realtime_back_left + realtime_back_right);
        double vw = 0.25 * 0.5 * WHEEL_DIAMETER * 0.5 * (D_X + D_Y) * (realtime_front_left + realtime_front_right + realtime_back_left + realtime_back_right);

        current_time = ros::Time::now();
        last_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        ros::Rate r(1.0);

        double dx = (vx * cos(w) - vy * sin(w)) * dt;
        double dy = (vx * sin(w) + vy * cos(w)) * dt;
        double dw = vw * dt;

        x += dx;
        y += dy;
        w += dw;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(w);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vw;

        odom_pub.publish(odom);

        last_time = current_time;
    }
    PLUGINLIB_EXPORT_CLASS(Controller::HeroChassisController, controller_interface::ControllerBase)
}