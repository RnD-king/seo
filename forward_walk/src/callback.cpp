#include "callback.hpp"
#include "sensor_msgs/msg/imu.hpp"  // IMU 메시지 타입 추가
#include <cmath>  // ZMP 계산용 수학 함수

bool flgflg = 0;
FILE *Trajectory_all;

Callback::Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *pick_Ptr)
    : Node("callback_node_call"),  // Node 생성 시 노드 이름을 추가
      trajectoryPtr(trajectoryPtr),
      IK_Ptr(IK_Ptr),
      dxlPtr(dxlPtr),
      pick_Ptr(pick_Ptr),
      SPIN_RATE(100)     
{
    // ROS 2에서의 Node 객체 생성
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("callback_node");
    
    // ROS 2에서 boost::thread 대신 std::thread 사용
    std::thread queue_thread(&Callback::callbackThread, this);
    queue_thread.detach();  // 비동기식 실행



    // set_subscriber_= this->create_subscription<std_msgs::msg::Bool>("/SET", 10, std::bind(&Callback::SetMode, this, std::placeholders::_1));

    // ROS 2의 subscription 생성
    start_subscriber_= this->create_subscription<std_msgs::msg::Bool>("/START", 10, std::bind(&Callback::StartMode, this, std::placeholders::_1));
    
    // callback.cpp
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&Callback::ImuCallback, this, std::placeholders::_1));

    trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Turn_Trajectory = VectorXd::Zero(135);
    omega_w = sqrt(g / z_c);

    pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 675);
    pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 675);
    pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 675);
    pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 675);

    
    emergency = 1;
    indext = 1;
    go = 0;

    RCLCPP_INFO(this->get_logger(), "Emergency value: %d", emergency);
    RCLCPP_INFO(this->get_logger(), "Callback activated");
}



void Callback::Set()
{
    All_Theta[0] = -IK_Ptr->RL_th[0];
    All_Theta[1] = IK_Ptr->RL_th[1] - RL_th1 * DEG2RAD - 3 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] - RL_th2 * DEG2RAD - 17 * DEG2RAD;
    All_Theta[3] = -IK_Ptr->RL_th[3] + 40 * DEG2RAD;
    All_Theta[4] = -IK_Ptr->RL_th[4] + 24.5 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5] + SR * DEG2RAD - 2 * DEG2RAD;
    All_Theta[6] = -IK_Ptr->LL_th[0];
    All_Theta[7] = IK_Ptr->LL_th[1] + LL_th1 * DEG2RAD + 2 * DEG2RAD;
    All_Theta[8] = -IK_Ptr->LL_th[2] + LL_th2 * DEG2RAD + 17 * DEG2RAD;
    All_Theta[9] = IK_Ptr->LL_th[3] - 40 * DEG2RAD;
    All_Theta[10] = IK_Ptr->LL_th[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5] + SR * DEG2RAD;

    // upper_body
    All_Theta[12] = pick_Ptr->WT_th[0] + step + 0 * DEG2RAD;  // waist
    All_Theta[13] = pick_Ptr->LA_th[0] + 90 * DEG2RAD; // L_arm
    All_Theta[14] = pick_Ptr->RA_th[0] - 90 * DEG2RAD; // R_arm
    All_Theta[15] = pick_Ptr->LA_th[1] - 60 * DEG2RAD; // L_arm
    All_Theta[16] = pick_Ptr->RA_th[1] + 60 * DEG2RAD; // R_arm
    All_Theta[17] = pick_Ptr->LA_th[2] - 90 * DEG2RAD; // L_elbow
    All_Theta[18] = pick_Ptr->RA_th[2] + 90 * DEG2RAD; // R_elbow
    All_Theta[19] = pick_Ptr->LA_th[3] - 0 * DEG2RAD; // L_hand
    All_Theta[20] = pick_Ptr->RA_th[3] + 0 * DEG2RAD; // R_hand
    All_Theta[21] = pick_Ptr->NC_th[0] + 0 * DEG2RAD; // neck_RL
    All_Theta[22] = pick_Ptr->NC_th[1] + 6 * DEG2RAD; // neck_UD
}

void Callback::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    acc_x_ = msg->linear_acceleration.x;
    acc_z_ = msg->linear_acceleration.z;
    
    RCLCPP_INFO(this->get_logger(), "IMU received: acc_x = %.4f, acc_z = %.4f", acc_x_, acc_z_);

    if (std::abs(acc_z_) > 1e-3)
    {
        zmp_meas_ = z_c * (acc_x_ / acc_z_);
    }
}


// ros2 topic pub /START std_msgs/msg/Bool "data: true" -1

void Callback::StartMode(const std_msgs::msg::Bool::SharedPtr start)
{
    RCLCPP_DEBUG(this->get_logger(), "StartMode called with data: %d", start->data);
    if (start->data)
    {
        indext = 0;
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 30);


        trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 30);

        emergency = 0;
        go = 1;

        RCLCPP_INFO(this->get_logger(), "StartMode activated with true data!");
    }
}

void Callback::callbackThread()
{
    // ROS 2의 spin 사용 대신 루프에서 메시지 처리
    rclcpp::Rate loop_rate(SPIN_RATE);
    
    while (rclcpp::ok())
    {
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }
}

void Callback::SelectMotion()
{
    if(re == 0)
    {
        re = 1;
        indext = 0;

    
        trajectoryPtr->Change_Freq(2);


        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Freq_Change_Straight(0.05, 0.2, 0.05, 1);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 3, 2, 2, 3, 2, 2, -2);   
        IK_Ptr->Set_Angle_Compensation(67);
        // trajectoryPtr->Picking_Motion(300, 150, 0.165);
    }
}

void Callback::PickMotion()
{
    if(re == 0)
    {
        re = 1;
        indext = 0;
        indext = 0;
    
        trajectoryPtr->Change_Freq(2);


        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Picking_Motion(300, 150, 0.165);
    }
}


// 동작 중인지 판별
bool Callback::IsMotionFinish()
{
    return (indext >= trajectoryPtr->Ref_RL_x.cols());
}

void Callback::ResetMotion()
{
    indext = 0;
    re = 0;

    RCLCPP_INFO(rclcpp::get_logger("Callback"), "[ResetMotion] 인덱스 및 상태 초기화 완료");
}



void Callback::Write_All_Theta()
{
    // RCLCPP_INFO(rclcpp::get_logger("Callback"), "[Write_All_Theta] 호출됨");
    RCLCPP_INFO(this->get_logger(), "initial_theta_saved 상태: %s", initial_theta_saved ? "true" : "false");


    if (emergency == 0)
    {

        // RCLCPP_INFO(rclcpp::get_logger("Callback"), "emergency == 0 OK");

        RCLCPP_INFO(rclcpp::get_logger("Callback"), "[Write_All_Theta] indext = %d / step_n = %ld", indext, trajectoryPtr->Ref_RL_x.cols());

        if (go == 1)
        {
            // if (indext < trajectoryPtr->Ref_RL_x.cols())
            // {
                // RCLCPP_INFO(rclcpp::get_logger("Callback"), "go == 1 → IK 실행");
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Fast_Angle_Compensation(indext);
            // }
            // else
            // {
            //     RCLCPP_WARN(this->get_logger(), "indext(%d) >= cols(%ld)", indext, trajectoryPtr->Ref_RL_x.cols());
            //     return;
            // }
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("Callback"), "go != 1 → IK 실행 안 됨");
        }
        
        // if (indext == 0 or indext == trajectoryPtr->Ref_RL_x.cols()){
        //     for (int i = 0; i < All_Theta.size(); ++i)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
        //     }
        // }

        indext += 1;

        if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
        {
            // RCLCPP_INFO(rclcpp::get_logger("Callback"), "indext 초과 → indext = %d, step_n = %ld", indext, trajectoryPtr->Ref_RL_x.cols());
            // indext -= 1;
            re = 0;
        }

    }

    else
    {
        RCLCPP_WARN(rclcpp::get_logger("Callback"), "emergency != 0 → 아무것도 안 함");
    }

    // ZMP 보정값 적용
    // double zmp_meas = z_c * (acc_x_ / acc_z_);
    // double zmp_ref = trajectoryPtr->GetZmpRef()[indext];
    // double zmp_err = zmp_ref - zmp_meas_;
    // double theta_correction = kp_zmp_ * zmp_err;
    // int hip_pitch_L = 8; //실제 모터 7번
    // int hip_pitch_R = 2; //실제 모터 6번
    // All_Theta(hip_pitch_L) += theta_correction;
    // All_Theta(hip_pitch_R) += theta_correction;
    // int test_motor_id = 3;
    // All_Theta(test_motor_id) += theta_correction;

    // RCLCPP_INFO(this->get_logger(),
    // "All_Theta(3) 보정 적용 전후: %.4f → %.4f (보정량: %.4f)", All_Theta(3), theta_correction);

    

    // All_Theta 계산 및 저장
    All_Theta[0] = -IK_Ptr->RL_th[0];
    All_Theta[1] = IK_Ptr->RL_th[1] - RL_th1 * DEG2RAD - 3 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] - RL_th2 * DEG2RAD - 17 * DEG2RAD;
    All_Theta[3] = -IK_Ptr->RL_th[3] + 40 * DEG2RAD;
    All_Theta[4] = -IK_Ptr->RL_th[4] + 24.5 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5] + SR * DEG2RAD - 2 * DEG2RAD;
    All_Theta[6] = -IK_Ptr->LL_th[0];
    All_Theta[7] = IK_Ptr->LL_th[1] + LL_th1 * DEG2RAD + 2 * DEG2RAD;
    All_Theta[8] = -IK_Ptr->LL_th[2] + LL_th2 * DEG2RAD + 17 * DEG2RAD;
    All_Theta[9] = IK_Ptr->LL_th[3] - 40 * DEG2RAD;
    All_Theta[10] = IK_Ptr->LL_th[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5] + SR * DEG2RAD;

    // upper_body
    All_Theta[12] = pick_Ptr->WT_th[0] + step + 0 * DEG2RAD;  // waist
    All_Theta[13] = pick_Ptr->LA_th[0] + 90 * DEG2RAD; // L_arm
    All_Theta[14] = pick_Ptr->RA_th[0] - 90 * DEG2RAD; // R_arm
    All_Theta[15] = pick_Ptr->LA_th[1] - 60 * DEG2RAD; // L_arm
    All_Theta[16] = pick_Ptr->RA_th[1] + 60 * DEG2RAD; // R_arm
    All_Theta[17] = pick_Ptr->LA_th[2] - 90 * DEG2RAD; // L_elbow
    All_Theta[18] = pick_Ptr->RA_th[2] + 90 * DEG2RAD; // R_elbow
    All_Theta[19] = pick_Ptr->LA_th[3] - 0 * DEG2RAD; // L_hand
    All_Theta[20] = pick_Ptr->RA_th[3] + 0 * DEG2RAD; // R_hand
    All_Theta[21] = pick_Ptr->NC_th[0] + 0 * DEG2RAD; // neck_RL
    All_Theta[22] = pick_Ptr->NC_th[1] + 6 * DEG2RAD; // neck_UD

    // [1] 첫 자세 저장
    if (!initial_theta_saved && indext == 1) {
        initial_theta = All_Theta;
        initial_theta_saved = true;
        RCLCPP_INFO(this->get_logger(), "[Write_All_Theta]  초기자세 저장 완료");
    }

    // [2] 마지막 자세 복원
    if (initial_theta_saved && indext == trajectoryPtr->Ref_RL_x.cols()) {
        All_Theta = initial_theta;
        initial_theta_saved = false;
        RCLCPP_INFO(this->get_logger(), "[Write_All_Theta]  마지막 프레임에서 자세 복원됨");

        // 디버깅 정보 출력
        for (int i = 0; i < All_Theta.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
        }
        return;
    }


    // // 디버깅 정보 출력
    // for (int i = 0; i < All_Theta.size(); ++i)
    // {
    //     RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
    // }

}
