#ifndef CALLBACK_H
#define CALLBACK_H



#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
// #include <tf2/LinearMath/Quaternion.h>
#include <boost/thread.hpp>
#include <Eigen/Dense>  // Eigen 관련 헤더 추가
#include "sensor_msgs/msg/imu.hpp" //


#include "dynamixel.hpp"
#include "BRP_Kinematics.hpp"
#include "NewPattern2.hpp"

using Eigen::VectorXd;  // Eigen 벡터를 간단히 사용하기 위한 선언

class Callback : public rclcpp::Node
{
private:
    // double turn_angle = 0;     
    // int arm_indext = 0;        
    double z_c = 1.2 * 0.28224; 
    double g = 9.81;           
    double omega_w;            
    double _dt = 0.01;         


    double acc_x_ = 0.0, acc_y_ = 0.0, acc_z_ = 1.0; //
    double zmp_meas_ = 0.0;
    double kp_zmp_ = 0.05;


    Trajectory *trajectoryPtr;    // Trajectory 객체를 가리키는 포인터
    IK_Function *IK_Ptr;          // IK_Function 객체를 가리키는 포인터
    Dxl *dxlPtr;                  // Dxl 객체를 가리키는 포인터
    Pick *pick_Ptr;               // Pick 객체를 가리키는 포인터
    double Goal_joint_[NUMBER_OF_DYNAMIXELS];

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_; //

public:
    Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *pick_Ptr);
    

    // === ZMP 보정 관련 추가 ===
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // ROS 메시지 콜백 함수
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_subscriber_;

    // virtual void SetMode(const std_msgs::msg::Bool::SharedPtr set);
    virtual void StartMode(const std_msgs::msg::Bool::SharedPtr start);

    int GetIndexT() const { return indext; }



    // 모션 제어 함수들
    virtual void SelectMotion();            
    virtual void Write_All_Theta();           
    void callbackThread();
    void Set();
    void ResetMotion();

    //모션 종료 확인 여부
    bool IsMotionFinish();

    const int SPIN_RATE = 100;
    
            
    int go = 0;
    int re = 0;
    int emergency = 99;
    int indext = 0;                        

    double step = 0;
    double RL_th2 = 0, LL_th2 = 0;
    double RL_th1 = 0, LL_th1 = 0;
    double HS = 0;  
    double SR = 0;  

    VectorXd All_Theta = MatrixXd::Zero(NUMBER_OF_DYNAMIXELS, 1);
    VectorXd initial_theta = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
    bool initial_theta_saved = false;

















    // double rl_neckangle = 0;                
    // double ud_neckangle = 0;                
    // double tmp_turn_angle = 0;              
    // bool emergency_ = 1;                    
    // double vel_x = 0;
    // double vel_y = 0;
    // double vel_z = 0;
    // int error_counter = 0;
    // bool error_printed = false;

    // int8_t mode = 99;                       
    // double walkfreq = 1.48114;             
    // double walktime = 2 / walkfreq;        
    // int freq = 100;                        
    // int walktime_n = walktime * freq;      


    // int upper_indext = 0;                 
    // int check_indext = 0;                 
    // int stop_indext = 0;  

    // bool turn_left = false;
    // bool turn_right = false;

    // bool on_emergency = false;

    // double angle = 0;
    // int index_angle = 0;

    // double Real_CP_Y = 0;
    // double Real_CP_X = 0;
    // double xZMP_from_CP = 0;
    // double yZMP_from_CP = 0;
    // double Real_zmp_y_accel = 0;

    // MatrixXd RL_motion, LL_motion;
    // MatrixXd RL_motion0, LL_motion0;
    // MatrixXd RL_motion1, LL_motion1;
    // MatrixXd RL_motion2, LL_motion2;
    // MatrixXd RL_motion3, LL_motion3;
    // MatrixXd RL_motion4, LL_motion4;
    // MatrixXd RL_motion5, LL_motion5;
    // MatrixXd RL_motion6, LL_motion6;
    // MatrixXd RL_motion7, LL_motion7;

};

#endif // CALLBACK_H
