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
#include <ros/rostime_decl.h>

/*Kinematics Inverse逆运动学*/

#define ENCODER_RESOLUTION      1440.0   //编码器分辨率, 轮子转一圈，编码器产生的脉冲数
#define WHEEL_DIAMETER          0.1525    //轮子直径,单位：米
#define D_X                     0.18     //底盘Y轴上两轮中心的间距
#define D_Y                     0.25     //底盘X轴上两轮中心的间距
#define PID_RATE                50       //PID调节PWM值的频率

class KinematicsInverse
{
private:
    double pulse_per_meter = 0;
    float rx_plus_ry_cali = 0.3;
    double angular_correction_factor = 1.0;
    double linear_correction_factor = 1.0;

public:
    /**
  * @函数作用：运动学解析参数初始化
  */
    void Kinematics_Init(void)
    {
        pulse_per_meter = (float)(ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926))/linear_correction_factor;

        float r_x = D_X/2;
        float r_y = D_Y/2;
        rx_plus_ry_cali = (r_x + r_y)/angular_correction_factor;
    }

/**
  * @函数作用：逆向运动学解析，底盘三轴速度-->轮子速度
  * @输入：机器人三轴速度 m/s
  * @输出：电机应达到的目标速度（一个PID控制周期内，电机编码器计数值的变化）
  */
    void Kinematics_Inverse(int16_t* input, int16_t* output)
    {
        float r_x = D_X/2;
        float r_y = D_Y/2;
        float v_tx   = (float)input[0];
        float v_ty   = (float)input[1];
        float omega = (float)input[2];
        static float v_w[4] = {0};

        v_w[0] = v_tx - v_ty - (r_x + r_y)*omega;
        v_w[1] = v_tx + v_ty + (r_x + r_y)*omega;
        v_w[2] = v_tx + v_ty - (r_x + r_y)*omega;
        v_w[3] = v_tx - v_ty + (r_x + r_y)*omega;

        output[0] = (int16_t)(v_w[0] * pulse_per_meter/PID_RATE);
        output[1] = (int16_t)(v_w[1] * pulse_per_meter/PID_RATE);
        output[2] = (int16_t)(v_w[2] * pulse_per_meter/PID_RATE);
        output[3] = (int16_t)(v_w[3] * pulse_per_meter/PID_RATE);
    }
};

/*Kinematics Forward正运动学*/

#define ENCODER_MAX 32767
#define ENCODER_MIN -32768
#define ENCODER_LOW_WRAP  ((ENCODER_MAX - ENCODER_MIN)*0.3+ENCODER_MIN)
#define ENCODER_HIGH_WRAP ((ENCODER_MAX - ENCODER_MIN)*0.7+ENCODER_MIN)
#define PI 3.1415926

class KinematicsForward
{
private:
    int32_t  wheel_turns[4] = {0};
    int32_t  encoder_sum_current[4] = {0};
    double pulse_per_meter = 0;

public:
    void Kinematics_Forward(int16_t* input, int16_t* output)
    {
        static double dv_w_times_dt[4];//轮子瞬时变化量dxw=dvw*dt
        static double dv_t_times_dt[3];//底盘瞬时变化量dxt=dvt*dt
        static int16_t encoder_sum[4];

        encoder_sum[0] = -input[0];
        encoder_sum[1] = input[1];
        encoder_sum[2] = -input[2];
        encoder_sum[3] = input[3];

        for(int i=0;i<4;i++)
        {
            if(encoder_sum[i] < ENCODER_LOW_WRAP && encoder_sum_current[i] > ENCODER_HIGH_WRAP)
                wheel_turns[i]++;
            else if(encoder_sum[i] > ENCODER_HIGH_WRAP && encoder_sum_current[i] < ENCODER_LOW_WRAP)
                wheel_turns[i]--;
            else
                wheel_turns[i]=0;
        }

        //将编码器数值转化为前进的距离，单位m
        for(int i=0;i<4;i++)
        {
            dv_w_times_dt[i] = 1.0*(encoder_sum[i] + wheel_turns[i]*(ENCODER_MAX-ENCODER_MIN)-encoder_sum_current[i])/pulse_per_meter;
            encoder_sum_current[i] = encoder_sum[i];
        }

        //要计算坐标所以变回来
        dv_w_times_dt[0] = -dv_w_times_dt[0];
        dv_w_times_dt[1] =  dv_w_times_dt[1];
        dv_w_times_dt[2] = -dv_w_times_dt[2];
        dv_w_times_dt[3] =  dv_w_times_dt[3];

        //计算底盘坐标系(base_link)下x轴、y轴变化距离m与Yaw轴朝向变化rad 一段时间内的变化量
        dv_t_times_dt[0] = ( dv_w_times_dt[3] + dv_w_times_dt[2])/2.0;
        dv_t_times_dt[1] = ( dv_w_times_dt[2] - dv_w_times_dt[0])/2.0;
        dv_t_times_dt[2] = ( dv_w_times_dt[1] - dv_w_times_dt[2])/(2*wheel_track_cali);

        //积分计算里程计坐标系(odom_frame)下的机器人X,Y,Yaw轴坐标
        //dx = ( vx*cos(theta) - vy*sin(theta) )*dt
        //dy = ( vx*sin(theta) + vy*cos(theta) )*dt
        output[0] += (int16_t)(cos((double)output[2])*dv_t_times_dt[0] - sin((double)output[2])*dv_t_times_dt[1]);
        output[1] += (int16_t)(sin((double)output[2])*dv_t_times_dt[0] + cos((double)output[2])*dv_t_times_dt[1]);
        output[2] += (int16_t)(dv_t_times_dt[2]*1000);

        //Yaw轴坐标变化范围控制-2Π -> 2Π
        if(output[2] > PI)
            output[2] -= 2*PI;
        else if(output[2] < -PI)
            output[2] += 2*PI;

        //发送机器人X轴y轴Yaw轴瞬时变化量，在ROS端除以时间
        output[3] = (int16_t)(dv_t_times_dt[0]);
        output[4] = (int16_t)(dv_t_times_dt[1]);
        output[5] = (int16_t)(dv_t_times_dt[2]);
    }
};


/*PID*/

class MyPIDController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
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

    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

    void update(const ros::Time &time, const ros::Duration &period);
//        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
//        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
//        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
//        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
//
//        controller_nh.param("pid_controller")


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

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "pid_controller_node");
//    ros::NodeHandle nh;
//
//    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
//              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
//
//    control_toolbox::Pid pidController;
//    simple_chassis_controller::SimpleChassisController update;
//
//    pidController.Pid(6.0, 1.0, 2.0, 0.3, -0.3, bool antiwindup)
//      : dynamic_reconfig_initialized_(false)
//    {
//        setGains(p,i,d,i_max,i_min,antiwindup);
//
//        reset();
//    }
//
//
//    pidController.initPid(6.0, 1.0, 2.0, 0.3, -0.3, ros::NodeHandle &controller_nh)
//    {
//        initPid(p, i, d, i_max, i_min);
//
//        ros::NodeHandle nh(DEFAULT_NAMESPACE);
//        initDynamicReconfig(nh);
//    }
//
//    pidController.initParam(std::string& prefix, quiet)
//    {
//        ros::NodeHandle nh(Param);
//        return init(nh, quiet);
//    }
//
//    update.update(ros::Time &time, ros::Duration &boost::date_time::period ) {
//
//        static double cmd_[6][4] = {{v_w[0], v_w[1], v_w[2], v_w[3]}, //  forward
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}, //  backward
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}, //  left
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}, //  right
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}, //  clockwise
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}};  //  counterclockwise
//
//        if ((time - last_change_).toSec() > 2) {
//            state_ = (state_ + 1) % 6;
//            last_change_ = time;
//        }
//        front_left_joint_.setCommand(cmd_[state_][0]);
//        front_right_joint_.setCommand(cmd_[state_][1]);
//        back_left_joint_.setCommand(cmd_[state_][2]);
//        back_right_joint_.setCommand(cmd_[state_][3]);
//    }
//
//
//    controller_manager::ControllerManager controllerManager(&pidController, nh);
//
//    ros::Rate loop_rate(10);
//
//
//}
//


// mecanum_pid_node.cpp
#include "ros/ros.h"
#include "std_msgs/Float64.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_node");
    ros::NodeHandle nh;

    double p, i, d, max_output, min_output;
    nh.param("p", p, 1.0);
    nh.param("i", i, 0.1);
    nh.param("d", d, 0.01);
    nh.param("max_output", max_output, 10.0);
    nh.param("min_output", min_output, -10.0);

    // Create PID controllers for each wheel
    control_toolbox::Pid controller1(p, i, d, max_output, min_output);
    control_toolbox::Pid controller2(p, i, d, max_output, min_output);
    control_toolbox::Pid controller3(p, i, d, max_output, min_output);
    control_toolbox::Pid controller4(p, i, d, max_output, min_output);

    // ROS Publishers
    ros::Publisher wheel1_pub = nh.advertise<std_msgs::Float64>("left_front", 10);
    ros::Publisher wheel2_pub = nh.advertise<std_msgs::Float64>("right_front", 10);
    ros::Publisher wheel3_pub = nh.advertise<std_msgs::Float64>("left_back", 10);
    ros::Publisher wheel4_pub = nh.advertise<std_msgs::Float64>("right_back", 10);

//   double update(ros::Time &time, ros::Duration &boost::date_time::period )
//   {
//        static double cmd_[6][4] = {{v_w[0], v_w[1], v_w[2], v_w[3]}, //  forward
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}, //  backward
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}, //  left
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}, //  right
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}, //  clockwise
//                                    {v_w[0], v_w[1], v_w[2], v_w[3]}};  //  counterclockwise
//
//        if ((time - last_change_).toSec() > 2) {
//            state_ = (state_ + 1) % 6;
//            last_change_ = time;
//        }
//        front_left_joint_.setCommand(cmd_[state_][0]);
//        front_right_joint_.setCommand(cmd_[state_][1]);
//        back_left_joint_.setCommand(cmd_[state_][2]);
//        back_right_joint_.setCommand(cmd_[state_][3]);
//   }


//    // ROS Subscribers (assuming you have feedback from each wheel)
//    ros::Subscriber wheel1_sub = nh.subscribe("wheel1_feedback", 10, &control_toolbox::Pid::update, &controller1);
//    ros::Subscriber wheel2_sub = nh.subscribe("wheel2_feedback", 10, &control_toolbox::Pid::update, &controller2);
//    ros::Subscriber wheel3_sub = nh.subscribe("wheel3_feedback", 10, &control_toolbox::Pid::update, &controller3);
//    ros::Subscriber wheel4_sub = nh.subscribe("wheel4_feedback", 10, &control_toolbox::Pid::update, &controller4);

    // ROS loop
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // Assuming you have desired velocities for each wheel
        double setpoint1 = 5.0;
        double setpoint2 = -2.0;
        double setpoint3 = 3.0;
        double setpoint4 = 1.0;

        // Update control commands using PID controllers
        double update(ros::Time &time, const ros::Duration &period)
        {
            static double cmd_[6][4] = {{v_w[0], v_w[1], v_w[2], v_w[3]}, //  forward
                                        {v_w[0], v_w[1], v_w[2], v_w[3]}, //  backward
                                        {v_w[0], v_w[1], v_w[2], v_w[3]}, //  left
                                        {v_w[0], v_w[1], v_w[2], v_w[3]}, //  right
                                        {v_w[0], v_w[1], v_w[2], v_w[3]}, //  clockwise
                                        {v_w[0], v_w[1], v_w[2], v_w[3]}};  //  counterclockwise

            if ((time - last_change_).toSec() > 2) {
                state_ = (state_ + 1) % 6;
                last_change_ = time;
            }
            front_left_joint_.setCommand(cmd_[state_][0]);
            front_right_joint_.setCommand(cmd_[state_][1]);
            back_left_joint_.setCommand(cmd_[state_][2]);
            back_right_joint_.setCommand(cmd_[state_][3]);
        }

//        // Publish control commands
//        std_msgs::Float64 wheel1_msg, wheel2_msg, wheel3_msg, wheel4_msg;
//        wheel1_msg.data = cmd1;
//        wheel2_msg.data = cmd2;
//        wheel3_msg.data = cmd3;
//        wheel4_msg.data = cmd4;
//
//        wheel1_pub.publish(wheel1_msg);
//        wheel2_pub.publish(wheel2_msg);
//        wheel3_pub.publish(wheel3_msg);
//        wheel4_pub.publish(wheel4_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

}
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::MyPIDController, controller_interface::ControllerBase)