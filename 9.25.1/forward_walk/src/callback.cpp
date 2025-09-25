#include "callback.hpp"
#include "sensor_msgs/msg/imu.hpp"  // IMU 메시지 타입 추가
#include <cmath>  
#include "robot_msgs/msg/line_result.hpp"


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


int Callback::GetTurnsRemaining() const {
    return turns_remaining_.load(std::memory_order_acquire);
}
const void* Callback::GetTurnsRemainingAddr() const {
    return static_cast<const void*>(&turns_remaining_);
}
// "남은 유닛 턴 수"를 delta만큼 감소시키고, '감소 전 값'을 반환.
// acq_rel: 감소 전 읽기(acquire)와 감소 쓰기(release) 모두의 메모리 순서 보장.
int Callback::FetchSubTurnsRemaining(int delta) {
    return turns_remaining_.fetch_sub(delta, std::memory_order_acq_rel);
}

// "남은 유닛 턴 수"를 v로 설정. release 오더: 이후 acquire 로드에서 이 쓰기 이전의 효과가 보장된다.
void Callback::SetTurnsRemaining(int v) {
    turns_remaining_.store(v, std::memory_order_release);
}


void Callback::OnLineResult(int angle)
{
    
    if (!line_turn)
    {
        RCLCPP_INFO(this->get_logger(), "[OnLineResult] ignored because line_turn==false");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[OnLineResult] recv angle=%.2d, line_turn=%d",
            angle, (int)line_turn);

    double line_angle_ = angle;
    // 원래는 8도인데 실제 로봇이 8도를 넘게 돌아서 13으로 임의 설정
    const int stepdeg = 13;

    int turncount = static_cast<int>(line_angle_ / stepdeg);
    double extra_angle = static_cast<int>(line_angle_ - stepdeg * turncount);

    if (extra_angle > 11)
    {
        turncount += 1;
    }

    int before = turns_remaining_.load(std::memory_order_seq_cst);
    turns_remaining_.store(turncount, std::memory_order_seq_cst);
    int after  = turns_remaining_.load(std::memory_order_seq_cst);

    // RCLCPP_INFO(this->get_logger(),
    //   "[OnLineResult] turns_remaining_: %d -> %d (set=%d) @%p",
    //   before, after, turncount, static_cast<const void*>(&turns_remaining_));

    line_turn = false;

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
        // go = 1;

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

void Callback::SetLineTurn(bool on)
{
    line_turn.store(on, std::memory_order_release);
    RCLCPP_INFO(this->get_logger(), "[line_turn] <- %d", static_cast<int>(on));
}

void Callback::SelectMotion(int go)
{

    go_ = go;

    // RCLCPP_WARN(this->get_logger(), "[SelectMotion] called: go=%d, re=%d", go, re);
    if(re == 0)
    {
        if(go == 0) //start_pose
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
        if(go == 1)//걷기 4step
        {
            re = 1;
            indext = 0;
            // angle = 1.5;


            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Freq_Change_Straight(0.05, 0.2, 0.05, 1);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 3, -2, 2, 1, -3, -2);   
            IK_Ptr->Set_Angle_Compensation(67);




            // re = 1;
            // indext = 0;
            // angle = 8;
            // // turn_st = 1; //1 -> turn_left, 2 -> turn_right

            // trajectoryPtr->Change_Freq(2);
            // IK_Ptr->Change_Com_Height(30);
            // trajectoryPtr->Go_Straight_start(0.05, 0.2, 0.05);
            // IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            // IK_Ptr->Change_Angle_Compensation(2, 3, 0, -2, 2, 1, 0, -2); 
            // IK_Ptr->Set_Angle_Compensation(135);
            // trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        
        }

        else if(go == 2 || go == 13)//좌회전
        {
            re = 1;
            indext = 0;
            angle = 8;

            index_angle = 0; // ★ 턴 진행 인덱스 리셋 (로그용/안전) 
            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Step_in_place(0.05, 0.25, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 1, 0, -2);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 3 || go == 14)//우회전
        {
            re = 1;
            indext = 0;
            angle = 8;

            index_angle = 0; // ★ 턴 진행 인덱스 리셋 (로그용/안전) 

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Step_in_place(0.05, 0.25, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 1, 0, -2);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 4)//Back_2step
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Back_2step;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Back_Straight(-0.04, -0.18, 0.05);
            IK_Ptr->Change_Angle_Compensation(3, 4, 2, 0, 2, 2, 2, -4); 
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 5  || go == 22)//Back_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Back_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Back_Straight(-0.04, -0.1, 0.05);
            IK_Ptr->Change_Angle_Compensation(3, 4, 2, 0, 2, 2, 2, -4);
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

        else if(go == 7)//Left_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Left_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Side_Left1(0.027);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 7, -4, 0, 2, 7, 4, -8);
            // IK_Ptr->Change_Angle_Compensation(2, 7, -4, 0, 2, 7, 4, -8);
            IK_Ptr->Set_Angle_Compensation(135);
        }


        else if(go == 8)//Right_Halfstep
        {
             re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Right_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Side_Right1(0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 7, 4, 0, 2, 7, -4, -4);
            // IK_Ptr->Change_Angle_Compensation(2, 7, 1, 1, 2, 4, 1, -1);
            IK_Ptr->Set_Angle_Compensation(135);
        }



        else if(go == 9 || go == 30)//PickMotion
        {
            re = 1;
            indext = 0;
            // mode = Motion_Index::Picking_Ball;
            
            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr -> Picking_Motion(300, 150, 0.165);
        }
        
        else if(go == 10)   //공 다시 잡기
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1,250);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 250);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 250);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 250);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 250);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 250);

            pick_Ptr->WT_Trajectory(0,250);
            pick_Ptr->RA_Trajectory(0,0,0,0,250);
            pick_Ptr->LA_Trajectory(-60,0,0,-30,250);
            pick_Ptr->NC_Trajectory(0,0,250);
        }


        else if(go == 11) //Catch_Finish
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 250);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 250);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 250);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 250);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 250);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 250);

            pick_Ptr->WT_Trajectory(0,250);
            pick_Ptr->RA_Trajectory(0,0,0,0,250);  
            pick_Ptr->LA_Trajectory(60,0,0,30,250);
            pick_Ptr->NC_Trajectory(0,0,250.
            
            );
        }

        else if(go == 12) //1 step
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.15, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -2, 2, 1, 0, -2); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if (go == 17) //shoot_ready
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);

            pick_Ptr->WT_Trajectory(-10,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(-180,-22,50,-50,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());

        }

        else if (go == 18) //shoot
        {
            re = 1;            
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(10);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);
        }

        else if (go == 19) //shoot_finish
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            // trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            // trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_z = trajectoryPtr->zsimulation_standup_Shoot_FINISH(500, -0.02);
            trajectoryPtr->Ref_LL_z = trajectoryPtr->zsimulation_standup_Shoot_FINISH(500, -0.02);

            pick_Ptr->WT_Trajectory(30,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(150,22,-110,50,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());


        }

        else if (go == 77)
        {
            re = 1;            
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 1200);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 1200);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 1200);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 1200);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 1200);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 1200);


        }

        else if(go == 20)//Huddle
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Huddle;
            IK_Ptr->Change_Com_Height(35);
            trajectoryPtr->Huddle_Motion(0.22, 0.14, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(6.5, 2.6, 0.0, 3.5, 6.5, 2.6, 0.0, -3.5);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(go == 21)//Fast_6step
        {
            re = 1;
            indext = 0;
            angle = 1.5;


            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Freq_Change_Straight(0.05, 0.4, 0.05, 1);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(3, 3, -2, 6, 3, 2, 2, -6);   
            IK_Ptr->Set_Angle_Compensation(67);
        }

        else if (go == 23) //Up_Nc
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);

            pick_Ptr->WT_Trajectory(0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,30,trajectoryPtr->Ref_RL_x.cols());

        }

        else if (go == 24) //Down_Nc
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);

            pick_Ptr->WT_Trajectory(0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,-30,trajectoryPtr->Ref_RL_x.cols());

        }

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
    index_angle = 0;

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
            
            if (go == 1 || go == 21) //fast forward 4step, 6step
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
                // IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            }

            else if (go == 2 || go == 13)//Step_in_place 좌회전
            {
                
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

                // std::cout << "indext" << indext << std::endl;
                if(indext > 135 && indext <= 270)
                {

                    IK_Ptr->LL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    step = (IK_Ptr->LL_th[0])/2;
                    index_angle = index_angle + 1;
                    // std::cout << "index_angle" << index_angle << std::endl;
                    if (index_angle > walktime_n - 1)
                    {
                        index_angle = 0;
                    }
                }

            }

            else if (go == 3 || go == 14)//Step_in_place 우회전
            {
                
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

                // std::cout << "indext" << indext << std::endl;
                if(indext>=67 && indext <=337)
                {
                    IK_Ptr->RL_th[0] = -(trajectoryPtr->Turn_Trajectory(index_angle));
                    step = (IK_Ptr->RL_th[0])/2;
                    index_angle += 1;
                    // std::cout << "index_angle" << index_angle << std::endl;
                    if (index_angle > walktime_n - 1)
                    {
                        index_angle = 0;
                    }
                }
            }

            else if (go == 4 || go == 5 || go == 22)//Back_Halfstep //Back_2step
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            }

            else if (go == 6)//Forward_Halfstep
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            }

            else if (go == 7)//Left_Halfstep
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation_Leftwalk(indext);
            }

            else if (go == 8)//Right_Halfstep
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation_Rightwalk(indext);
            }
            
            else if (go == 9 || go == 30) // Pick
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                
                pick_Ptr->Picking(trajectoryPtr->Ref_RL_x, indext, RL_th2, LL_th2);
            }

            else if (go == 10 || go == 11 || go == 17 || go == 19 || go == 24 || go == 23) //Re_Catch, Catch_Finish, shoot_ready, shoot_finish, Up_Nc, Down_Nc
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                pick_Ptr->UPBD_SET(pick_Ptr->Ref_WT_th, pick_Ptr->Ref_RA_th, pick_Ptr->Ref_LA_th, pick_Ptr->Ref_NC_th, indext);

            }
            else if (go == 12) //1step
            {                
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            }
            
            else if (go == 18) //shoot
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                pick_Ptr->WT_th[0] =  -20 * DEG2RAD;
                pick_Ptr->LA_th[0] = -150 * DEG2RAD;
                pick_Ptr->LA_th[1] = -22 * DEG2RAD;
                pick_Ptr->LA_th[2] = 110 * DEG2RAD;
                pick_Ptr->LA_th[3]  = -50 * DEG2RAD;
            }

            else if(go == 20)//Huddle 1
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                IK_Ptr->Angle_Compensation_Huddle(indext);
            }


            else if (go == 77)
            {
                IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
                
                pick_Ptr->Stand_up(trajectoryPtr->Ref_RL_x, indext);
            }
        }

    }

    indext += 1;

    if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
    {

        re = 0;

        
    }



    // All_Theta 계산 및 저장
    All_Theta[0] = -IK_Ptr->RL_th[0] + pick_Ptr->RL_th_ALL[0] + startRL_st[0];
    All_Theta[1] = IK_Ptr->RL_th[1] + pick_Ptr->RL_th_ALL[1] +startRL_st[1] -RL_th1 * DEG2RAD - 3 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] + pick_Ptr->RL_th_ALL[2] +startRL_st[2] -RL_th2 * DEG2RAD - 17 * DEG2RAD; //10.74 right
    All_Theta[3] = -IK_Ptr->RL_th[3] + pick_Ptr->RL_th_ALL[3] +startRL_st[3] + 40 * DEG2RAD; //38.34 * DEG2RAD;   
    All_Theta[4] = -IK_Ptr->RL_th[4] + pick_Ptr->RL_th_ALL[4] +startRL_st[4] + 24.22 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5] + pick_Ptr->RL_th_ALL[5] +startRL_st[5] - 2* DEG2RAD;

    All_Theta[6] = -IK_Ptr->LL_th[0] + pick_Ptr->LL_th_ALL[0] +startLL_st[0];
    All_Theta[7] = IK_Ptr->LL_th[1] + pick_Ptr->LL_th_ALL[1] +startLL_st[1] +LL_th1 * DEG2RAD + 2 * DEG2RAD;
    All_Theta[8] = -IK_Ptr->LL_th[2] + pick_Ptr->LL_th_ALL[2] +startLL_st[2] +LL_th2 * DEG2RAD + 17 * DEG2RAD; //left
    All_Theta[9] = IK_Ptr->LL_th[3] + pick_Ptr->LL_th_ALL[3] +startLL_st[3] - 40 * DEG2RAD; //40.34 * DEG2RAD;  
    All_Theta[10] = IK_Ptr->LL_th[4] + pick_Ptr->LL_th_ALL[4] +startLL_st[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5] + pick_Ptr->LL_th_ALL[5] +startLL_st[5] - 2 * DEG2RAD;
    // All_Theta 계산 및 저장
    // All_Theta[0] = -IK_Ptr->RL_th[0] +startRL_st[0] + turnRL_st;
    // All_Theta[1] = IK_Ptr->RL_th[1] +startRL_st[1] -RL_th1 * DEG2RAD - 3 * DEG2RAD;
    // All_Theta[2] = IK_Ptr->RL_th[2] +startRL_st[2] -RL_th2 * DEG2RAD - 17 * DEG2RAD; //10.74 right
    // All_Theta[3] = -IK_Ptr->RL_th[3] +startRL_st[3] + 40 * DEG2RAD; //38.34 * DEG2RAD;   
    // All_Theta[4] = -IK_Ptr->RL_th[4] +startRL_st[4] + 24.22 * DEG2RAD;
    // All_Theta[5] = -IK_Ptr->RL_th[5] +startRL_st[5] - 2* DEG2RAD;

    // All_Theta[6] = -IK_Ptr->LL_th[0] +startLL_st[0] + turnLL_st;
    // All_Theta[7] = IK_Ptr->LL_th[1] +startLL_st[1] +LL_th1 * DEG2RAD + 2 * DEG2RAD;
    // All_Theta[8] = -IK_Ptr->LL_th[2] +startLL_st[2] +LL_th2 * DEG2RAD + 17 * DEG2RAD; //left
    // All_Theta[9] = IK_Ptr->LL_th[3] +startLL_st[3] - 40 * DEG2RAD; //40.34 * DEG2RAD;  
    // All_Theta[10] = IK_Ptr->LL_th[4] +startLL_st[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
    // All_Theta[11] = -IK_Ptr->LL_th[5] +startLL_st[5] - 2 * DEG2RAD;

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


    if(go == 0)
    {
        for (int i = 0; i < 6; i++) 
        {
            startRL_st[i] = 0.0;
            startLL_st[i] = 0.0;
        }
    }

    // 디버깅 정보 출력
//     for (int i = 0; i < All_Theta.size(); ++i)
//     {
//         RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
//     }
}


