#include "callback.hpp"
#include "sensor_msgs/msg/imu.hpp"  // IMU 메시지 타입 추가
#include <cmath>  
#include "robot_msgs/msg/line_result.hpp"

namespace {
constexpr double kRad2Deg = 57.2957795130823208768; // 180/pi
inline double Rad2Deg(double r) { return r * kRad2Deg; }
}


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

    line_subscriber_ = this->create_subscription<robot_msgs::msg::LineResult>(
        "/line_result", 10,
        std::bind(&Callback::OnLineResult, this, std::placeholders::_1));

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
    All_Theta[0] = 0.0;
    All_Theta[1] = -0.050419;
    All_Theta[2] = -0.785155;
    All_Theta[3] = -0.327585;
    All_Theta[4] = 0.959987;
    All_Theta[5] = -0.032966;
    All_Theta[6] = 0.0;
    All_Theta[7] = 0.036848;
    All_Theta[8] = 0.785155;
    All_Theta[9] = 0.327585;
    All_Theta[10] = -0.907627;
    All_Theta[11] = -0.032966;

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

void Callback::OnLineResult(const LineResult::SharedPtr msg)
{
    if (!line_turn)
    {
        return;
    }

    constexpr double kStepDeg = 8.0;    // 모션 1회 회전각
    constexpr double kRoundUp = 6.0;    // 6° 초과면 1회 추가 

    double line_angle_ = msg->angle;  
    int turncount = static_cast<int>(line_angle_ / kStepDeg);
    double extra_angle = std::fmod(line_angle_ , kStepDeg);

    if ( extra_angle >= kRoundUp) 
    {
        ++turncount;
    }

    pending_turns.store(turncount, std::memory_order_relaxed);
    line_turn.store(false, std::memory_order_release); // 원샷 종료

    RCLCPP_INFO(this->get_logger(),
        "[OnLineResult] angle=%.2f deg -> turncount=%d (extra_angle=%.2f %s %.1f)",
        line_angle_, turncount, extra_angle);
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

void Callback::TATA()
{
    double res_turn_angle = angle;

    if (res_turn_angle != 0)
    {
        turn_angle = res_turn_angle * DEG2RAD;
        trajectoryPtr->Make_turn_trajectory(turn_angle);
        // index_angle = 0;
    }
    // RCLCPP_WARN(this->get_logger(), "TURN_ANGLE : %.2f deg", res_turn_angle);
    // RCLCPP_INFO(this->get_logger(), "------------------------- TURN_ANGLE ----------------------------");
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

void Callback::SelectMotion(int go)
{

    go_ = go;

    // RCLCPP_WARN(this->get_logger(), "[SelectMotion] called: go=%d, re=%d", go, re);
    if(re == 0)
    {
        if(go == 0)
        {
            startRL_st[0] = 0;
            startRL_st[1] = 0.001941;
            startRL_st[2] = 0.122416;
            startRL_st[3] = 0.196013;
            startRL_st[4] = -0.073595;//1.148133;
            startRL_st[5] = 0.001941;

            startLL_st[0] = 0;
            startLL_st[1] = 0.001941;
            startLL_st[2] = -0.122416;
            startLL_st[3] = -0.196013;
            startLL_st[4] = 0.073595;//-1.148133;
            startLL_st[5] = 0.001941;

            // startRL_st[6] = { 0.0, 0.001941, 0.122416, 0.196013, 1.148133, 0.001941 };
            // startLL_st[6] = { 0.0, 0.001941, -0.122416, -0.196013, -1.148133, 0.001941 };
        }
        if(go == 1)//걷기
        {
            re = 1;
            indext = 0;
            angle = 3;


            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Freq_Change_Straight(0.05, 0.4, 0.05, 1);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2);   
            IK_Ptr->Set_Angle_Compensation(67);
        }

        else if(go == 2)//우회전
        {
            re = 1;
            indext = 0;
            angle = 8;
            line_turn = true;

            index_angle = 0; // ★ 턴 진행 인덱스 리셋 (로그용/안전) 
            pending_turns.store(0, std::memory_order_relaxed);

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Step_in_place(0.05, 0.25, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2);
            IK_Ptr->Set_Angle_Compensation(135);
            RCLCPP_WARN(this->get_logger(), "경로 생성");
        }

        else if(go == 3)//좌회전
        {
            re = 1;
            indext = 0;
            angle = 8;
            line_turn = true;

            index_angle = 0; // ★ 턴 진행 인덱스 리셋 (로그용/안전) 
            pending_turns.store(0, std::memory_order_relaxed);

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Step_in_place(0.05, 0.25, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2);
            IK_Ptr->Set_Angle_Compensation(135);
            RCLCPP_WARN(this->get_logger(), "경로 생성");
        }

        else if(go == 4)//Back_2step
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Back_2step;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Back_Straight(-0.04, -0.2, 0.05);
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2); 
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 5)//Back_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Back_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Back_Straight(-0.04, -0.12, 0.05);
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2); 
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation(135);
        }
                
        else if(go == 6)//Forward_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Forward_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight(0.01, 0.03, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2); 
            IK_Ptr->Set_Angle_Compensation(135);
        }


        else if(go == 7)//PickMotion
        {
            re = 1;
            indext = 0;
        
            trajectoryPtr->Change_Freq(2);


            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Picking_Motion(300, 150, 0.165);
        }
        
        else if(go == 8)//Huddle 1
        {
            re = 1;
            indext = 0;




            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Huddle;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Huddle_Motion(0.22, 0.14, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(6.5, 3.5, 0, 3.5, 6.5, 3.5, 0, -3.5);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        // else if(go == 7)//Shoot
        // {
        //     re = 1;
        //     indext = 0;


        // }

        // else if(go == 77)//RECOVERY    200  0~199 1~200
        // {
        //     re = 1;
        //     indext = 0;


        // }

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
    if (emergency == 0)
    {
        if(re == 1)
        {
            int go = go_;
            RCLCPP_INFO(rclcpp::get_logger("Callback"), "[Write_All_Theta] indext = %d / step_n = %ld / go = %d", indext, trajectoryPtr->Ref_RL_x.cols(), go);
            
            if (go == 1) //걷기
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Fast_Angle_Compensation(indext);
                if(indext>=67 && indext<=134)
                {
                    IK_Ptr->RL_th[0] = (trajectoryPtr->Turn_Trajectory(index_angle));
                    step = (IK_Ptr->RL_th[0])/2;
                    index_angle += 1;
                    // std::cout << "index_angle" << index_angle << "walktime_n" << walktime_n << std::endl;
                    if (index_angle > 66)
                    {
                        index_angle = 0;
                    }
                }
            }

            else if (go == 2)//Step_in_place 우회전
            {
                
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

                // int remain = pending_turns.load(std::memory_order_acquire);
                // std::cout << "indext" << indext << std::endl;
                if(indext>=67 && indext <=337)
                {
                    // IK_Ptr->RL_th[0] = -(trajectoryPtr->Turn_Trajectory(index_angle));
                    double yaw_cmd = trajectoryPtr->Turn_Trajectory(index_angle);
                    IK_Ptr->LL_th[0] = yaw_cmd;

                    // 최소 로그 (도 단위로 보기 편하게)

                    RCLCPP_INFO(this->get_logger(),
                        "[TURN][L] indext=%d idxAng=%d  yaw=%.2f deg  →  LL_th[0]=%.2f deg",
                        indext, index_angle, yaw_cmd, IK_Ptr->LL_th[0]);
                    step = (IK_Ptr->RL_th[0])/2;
                    index_angle += 1;
                    // std::cout << "index_angle" << index_angle << std::endl;
                    if (index_angle > walktime_n - 1)
                    {
                        index_angle = 0;
                        // int prev = pending_turns.fetch_sub(1, std::memory_order_acq_rel);
                        // RCLCPP_INFO(this->get_logger(),
                        //     "[TurnCycleEnd L] 1 cycle consumed. prev=%d -> now=%d",
                        //     prev, prev-1);
                        // if (prev <= 1)
                        // {
                        //     IK_Ptr->RL_th[0] = 0;
                        //     pending_turns.store(0, std::memory_order_relaxed);

                        //     go = 0;
                        //     SelectMotion(0);
                        //     return;        
                            
                        // }
                    }
                }
            }

            else if (go == 3)//Step_in_place 좌회전
            {
                
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

                // int remain = pending_turns.load(std::memory_order_acquire);
                // std::cout << "indext" << indext << std::endl;
                if(indext > 135 && indext <= 270)
                {
                    // IK_Ptr->LL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    double yaw_cmd = trajectoryPtr->Turn_Trajectory(index_angle);
                    IK_Ptr->RL_th[0] = -yaw_cmd;

                    RCLCPP_INFO(this->get_logger(),
                        "[TURN][R] indext=%d idxAng=%d  yaw=%.2f deg  →  RL_th[0]=%.2f deg",
                        indext, index_angle, yaw_cmd, IK_Ptr->RL_th[0]);
                    step = (IK_Ptr->LL_th[0])/2;
                    index_angle = index_angle + 1;
                    // std::cout << "index_angle" << index_angle << std::endl;
                    if (index_angle > walktime_n - 1)
                    {
                        index_angle = 0;
                        // int prev = pending_turns.fetch_sub(1, std::memory_order_acq_rel);
                        // if (prev <= 1)
                        // {
                        //     IK_Ptr->LL_th[0] = 0;
                        //     pending_turns.store(0, std::memory_order_relaxed);

                        //     go = 0;
                        //     SelectMotion(0);
                        //     return;        
                        // }
                    }
                }

            }

            else if (go == 4 || go == 5)//Back_Halfstep //Back_2step
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            }

            else if (go == 6)//Forward_Halfstep
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            }

            else if (go == 7) // Pick
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Fast_Angle_Compensation(indext);
            }


            else if(go == 8)//Huddle 1
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation_Huddle(indext);
            }

            // else if(go == 7)//Shoot
            // {
            //     IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            //     IK_Ptr->Angle_Compensation_Huddle(indext);
            // }

            // else if(go == 77)//RECOVERY
            // {
            //     IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            //     IK_Ptr->Angle_Compensation_Huddle(indext);
            // }

        }

    }

    indext += 1;

    if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
    {

        re = 0;

        
    }

    //     // ZMP 보정값 적용
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
    All_Theta[0] = -IK_Ptr->RL_th[0] +startRL_st[0];
    All_Theta[1] = IK_Ptr->RL_th[1] +startRL_st[1] -RL_th1 * DEG2RAD - 3 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] +startRL_st[2] -RL_th2 * DEG2RAD - 17 * DEG2RAD; //10.74 right
    All_Theta[3] = -IK_Ptr->RL_th[3] +startRL_st[3] + 40 * DEG2RAD; //38.34 * DEG2RAD;   
    All_Theta[4] = -IK_Ptr->RL_th[4] +startRL_st[4] + 24.22 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5] +startRL_st[5] - 2* DEG2RAD;

    All_Theta[6] = -IK_Ptr->LL_th[0] +startLL_st[0];
    All_Theta[7] = IK_Ptr->LL_th[1] +startLL_st[1] +LL_th1 * DEG2RAD + 2 * DEG2RAD;
    All_Theta[8] = -IK_Ptr->LL_th[2] +startLL_st[2] +LL_th2 * DEG2RAD + 17 * DEG2RAD; //left
    All_Theta[9] = IK_Ptr->LL_th[3] +startLL_st[3] - 40 * DEG2RAD; //40.34 * DEG2RAD;  
    All_Theta[10] = IK_Ptr->LL_th[4] +startLL_st[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5] +startLL_st[5] - 2 * DEG2RAD;

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
        // RCLCPP_INFO(this->get_logger(), "[Write_All_Theta]  초기자세 저장 완료");

        // for (int i = 0; i < All_Theta.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
        // }
        // return;
    }

    // [2] 마지막 자세 복원
    if (initial_theta_saved && indext == trajectoryPtr->Ref_RL_x.cols()) {
        All_Theta = initial_theta;
        initial_theta_saved = false;
        // RCLCPP_INFO(this->get_logger(), "[Write_All_Theta]  마지막 프레임에서 자세 복원됨");

        // // 디버깅 정보 출력
        // for (int i = 0; i < All_Theta.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
        // }
        // return;
    }


    if(go == 0)
    {
        for (int i = 0; i < 6; i++) 
        {
            startRL_st[i] = 0.0;
            startLL_st[i] = 0.0;
        }
    }

    // 디버깅 정보 출력
    // for (int i = 0; i < All_Theta.size(); ++i)
    // {
    //     RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
    // }
}




// void Callback::Write_All_Theta()
// {
//     // RCLCPP_INFO(rclcpp::get_logger("Callback"), "[Write_All_Theta] 호출됨");
//     RCLCPP_INFO(this->get_logger(), "initial_theta_saved 상태: %s", initial_theta_saved ? "true" : "false");


//     if (emergency == 0)
//     {

//         // RCLCPP_INFO(rclcpp::get_logger("Callback"), "emergency == 0 OK");

//         RCLCPP_INFO(rclcpp::get_logger("Callback"), "[Write_All_Theta] indext = %d / step_n = %ld", indext, trajectoryPtr->Ref_RL_x.cols());

//         if (go == 1)
//         {
//             // if (indext < trajectoryPtr->Ref_RL_x.cols())
//             // {
//                 // RCLCPP_INFO(rclcpp::get_logger("Callback"), "go == 1 → IK 실행");
//             IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
//             IK_Ptr->Fast_Angle_Compensation(indext);
//             // }
//             // else
//             // {
//             //     RCLCPP_WARN(this->get_logger(), "indext(%d) >= cols(%ld)", indext, trajectoryPtr->Ref_RL_x.cols());
//             //     return;
//             // }
//         }
//         else
//         {
//             RCLCPP_WARN(rclcpp::get_logger("Callback"), "go != 1 → IK 실행 안 됨");
//         }
        
//         // if (indext == 0 or indext == trajectoryPtr->Ref_RL_x.cols()){
//         //     for (int i = 0; i < All_Theta.size(); ++i)
//         //     {
//         //         RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
//         //     }
//         // }

//         indext += 1;

//         if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
//         {
//             // RCLCPP_INFO(rclcpp::get_logger("Callback"), "indext 초과 → indext = %d, step_n = %ld", indext, trajectoryPtr->Ref_RL_x.cols());
//             // indext -= 1;
//             re = 0;
//         }

//     }

//     else
//     {
//         RCLCPP_WARN(rclcpp::get_logger("Callback"), "emergency != 0 → 아무것도 안 함");
//     }

//     // ZMP 보정값 적용
//     // double zmp_meas = z_c * (acc_x_ / acc_z_);
//     // double zmp_ref = trajectoryPtr->GetZmpRef()[indext];
//     // double zmp_err = zmp_ref - zmp_meas_;
//     // double theta_correction = kp_zmp_ * zmp_err;
//     // int hip_pitch_L = 8; //실제 모터 7번
//     // int hip_pitch_R = 2; //실제 모터 6번
//     // All_Theta(hip_pitch_L) += theta_correction;
//     // All_Theta(hip_pitch_R) += theta_correction;
//     // int test_motor_id = 3;
//     // All_Theta(test_motor_id) += theta_correction;

//     // RCLCPP_INFO(this->get_logger(),
//     // "All_Theta(3) 보정 적용 전후: %.4f → %.4f (보정량: %.4f)", All_Theta(3), theta_correction);

    

//     // All_Theta 계산 및 저장
//     All_Theta[0] = -IK_Ptr->RL_th[0];
//     All_Theta[1] = IK_Ptr->RL_th[1] - RL_th1 * DEG2RAD - 3 * DEG2RAD;
//     All_Theta[2] = IK_Ptr->RL_th[2] - RL_th2 * DEG2RAD - 17 * DEG2RAD;
//     All_Theta[3] = -IK_Ptr->RL_th[3] + 40 * DEG2RAD;
//     All_Theta[4] = -IK_Ptr->RL_th[4] + 24.5 * DEG2RAD;
//     All_Theta[5] = -IK_Ptr->RL_th[5] + SR * DEG2RAD - 2 * DEG2RAD;
//     All_Theta[6] = -IK_Ptr->LL_th[0];
//     All_Theta[7] = IK_Ptr->LL_th[1] + LL_th1 * DEG2RAD + 2 * DEG2RAD;
//     All_Theta[8] = -IK_Ptr->LL_th[2] + LL_th2 * DEG2RAD + 17 * DEG2RAD;
//     All_Theta[9] = IK_Ptr->LL_th[3] - 40 * DEG2RAD;
//     All_Theta[10] = IK_Ptr->LL_th[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
//     All_Theta[11] = -IK_Ptr->LL_th[5] + SR * DEG2RAD;

//     // upper_body
//     All_Theta[12] = pick_Ptr->WT_th[0] + step + 0 * DEG2RAD;  // waist
//     All_Theta[13] = pick_Ptr->LA_th[0] + 90 * DEG2RAD; // L_arm
//     All_Theta[14] = pick_Ptr->RA_th[0] - 90 * DEG2RAD; // R_arm
//     All_Theta[15] = pick_Ptr->LA_th[1] - 60 * DEG2RAD; // L_arm
//     All_Theta[16] = pick_Ptr->RA_th[1] + 60 * DEG2RAD; // R_arm
//     All_Theta[17] = pick_Ptr->LA_th[2] - 90 * DEG2RAD; // L_elbow
//     All_Theta[18] = pick_Ptr->RA_th[2] + 90 * DEG2RAD; // R_elbow
//     All_Theta[19] = pick_Ptr->LA_th[3] - 0 * DEG2RAD; // L_hand
//     All_Theta[20] = pick_Ptr->RA_th[3] + 0 * DEG2RAD; // R_hand
//     All_Theta[21] = pick_Ptr->NC_th[0] + 0 * DEG2RAD; // neck_RL
//     All_Theta[22] = pick_Ptr->NC_th[1] + 6 * DEG2RAD; // neck_UD

//     // [1] 첫 자세 저장
//     if (!initial_theta_saved && indext == 1) {
//         initial_theta = All_Theta;
//         initial_theta_saved = true;
//         RCLCPP_INFO(this->get_logger(), "[Write_All_Theta]  초기자세 저장 완료");
//     }

//     // [2] 마지막 자세 복원
//     if (initial_theta_saved && indext == trajectoryPtr->Ref_RL_x.cols()) {
//         All_Theta = initial_theta;
//         initial_theta_saved = false;
//         RCLCPP_INFO(this->get_logger(), "[Write_All_Theta]  마지막 프레임에서 자세 복원됨");

//         // 디버깅 정보 출력
//         for (int i = 0; i < All_Theta.size(); ++i)
//         {
//             RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
//         }
//         return;
//     }


//     // // 디버깅 정보 출력
//     // for (int i = 0; i < All_Theta.size(); ++i)
//     // {
//     //     RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
//     // }

// }
