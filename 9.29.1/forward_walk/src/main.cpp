#include "rclcpp/rclcpp.hpp"                 // ROS 2 기본 헤더
#include "dynamixel.hpp"                     // 사용자 정의: 다이나믹셀 제어 클래스
#include "callback.hpp"                      // 사용자 정의: 콜백 연산 모듈
#include "dynamixel_controller.hpp"          // 사용자 정의: θ 제어 인터페이스
#include "BRP_Kinematics.hpp"
#include "NewPattern2.hpp"     // 사용자 정의: 보행 궤적 생성기
#include "robot_msgs/msg/motion_command.hpp"  //motion command 불러오기
#include "robot_msgs/msg/motion_end.hpp"  //motion end 불러오기
#include "sensor_msgs/msg/imu.hpp" //imu sensor

#include <unistd.h> //sleep 관련
#include <atomic> 
#include <chrono>
#include <memory>
#include <cstdio>    // For FILE*

using namespace std::chrono_literals;



class MainNode : public rclcpp::Node
{
public:
    MainNode()
    : Node("main_node"), motion_in_progress_(false)
    {
        // FTDI USB latency timer 설정 (주의: 여전히 sudo 필요)
        system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

        // 객체 초기화
        dxl_ = std::make_shared<Dxl>();
        trajectory_ = std::make_shared<Trajectory>();
        ik_ = std::make_shared<IK_Function>();
        pick_ = std::make_shared<Pick>();

        dxl_ctrl_ = std::make_shared<Dxl_Controller>(dxl_.get());
        callback_ = std::make_shared<Callback>(trajectory_.get(), ik_.get(), dxl_.get(), pick_.get());

        VectorXd theta_zero = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        dxl_->MoveToTargetSmoothCos(theta_zero, 150, 10);

        callback_->Set();
        // VectorXd theta_goal = Eigen::Map<VectorXd>(callback_->All_Theta, NUMBER_OF_DYNAMIXELS);
        dxl_->MoveToTargetSmoothCos(callback_->All_Theta, 150, 10);
        std::cout << "[Info] Start is half!!!!!!" << std::endl;

        // /START 퍼블리셔 생성
        start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/START", 10);

        // motion_end 퍼블리셔 생성
        motion_end_pub_ = this->create_publisher<robot_msgs::msg::MotionEnd>("/motion_end", 10);

        // 타이머를 이용해 1.5초 후 /START 메시지 전송
        start_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1500),
            [this]() {
                std_msgs::msg::Bool msg;
                msg.data = true;
                start_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "[AUTO] /START 퍼블리시 완료");
                start_timer_->cancel();  // 한 번만 실행
            }
        );

        // 타이머를 이용해 MotionEnd 메시지 전송
        // motion_start_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(15000),
        //     [this]() {
        //         robot_msgs::msg::MotionEnd msg;
        //         msg.motion_end_detect = true;
        //         motion_end_pub_->publish(msg);
        //         RCLCPP_INFO(this->get_logger(), "MotionEnd 1회 퍼블리쉬");
        //         motion_start_timer_->cancel();  // 한 번만 실행
        //     }
        // );

        command_subscription_ = this->create_subscription<robot_msgs::msg::MotionCommand>(
            "/motion_command", 10, std::bind(&MainNode::MotionCallback, this, std::placeholders::_1));

        // 100Hz 루프 타이머 생성 (처음엔 중단 상태)
        motion_loop_timer_ = this->create_wall_timer(
            10ms, std::bind(&MainNode::MotionLoop, this));
        motion_loop_timer_->cancel();  // 초기에는 멈춰있음
    }

private:

    
    std::atomic<int> turns_remaining_{0};
    bool motion_in_progress_ = false;                         // 현재 모션 실행 중인지 여부
    int current_go_ = 0;
    int count1 = 0;
    int count2 = 0;
    int pick_count = 0;
    int back_count = 0;
    bool pick_mode = false;
    bool shoot_mode = false;
    bool pick_fail = false;


    rclcpp::TimerBase::SharedPtr motion_loop_timer_;       // 반복 제어용 타이머

    rclcpp::Publisher<robot_msgs::msg::MotionEnd>::SharedPtr motion_end_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::TimerBase::SharedPtr motion_start_timer_;
    // motioncommand subscribe
    rclcpp::Subscription<robot_msgs::msg::MotionCommand>::SharedPtr command_subscription_;


    bool out_turn_end = false; // out일때 회전 했는지

    void MotionCallback(const robot_msgs::msg::MotionCommand::SharedPtr msg)        
    {
        RCLCPP_INFO(this->get_logger(), "command=%d, angle=%d", msg->command,msg->angle);

        if (motion_in_progress_) {

            // if(msg->command == 99){
            //     RCLCPP_INFO(this->get_logger(), "[STOP] 명령 수신");
            //     callback_->Set();
            //     dxl_->MoveToTargetSmoothCos(callback_->All_Theta, 150, 10);
            //     callback_->ResetMotion();
            //     RCLCPP_INFO(this->get_logger(), "[MotionEnd] 모션 종료 메시지 전송");
            //     motion_in_progress_ = false;
            //     return;
            // }

            RCLCPP_WARN(this->get_logger(), "동작 중: 명령 무시됨");
            return;
        }
        
        int command_ = 0;
        int angle_ = 0;

        switch (msg->command) 
        {
            case 1: command_ = 1; break;    // 직진

            case 2: command_ = 2; break;    // 좌회전

            case 3: command_ = 3; break;    // 우회전

            case 4: command_ = 4; break;    // BACK

            case 5: command_ = 5; break;    // BACK_HALF

            case 6: command_ = 6; break;    // FORWARD_HALF

            case 7: command_ = 7; break;    // Left_half

            case 8: command_ = 8; break;    // RIGHT_Half

            case 9: command_ = 9;  break;   // pick

            case 10: command_ = 10; pick_mode = true; break;  // Recatch

            case 11: command_ = 11; break;  // Catch_Finish

            case 12: command_ = 12; break;  // 1step

            case 13: 
                if (out_turn_end)
                {
                    RCLCPP_WARN(this->get_logger(), "이미 turn함"); 
                    return; 
                }
                command_ = 13; break;  // out일때 좌회전 + 1step
        
            case 14: 
                if (out_turn_end)
                {
                    RCLCPP_WARN(this->get_logger(), "이미 turn함"); 
                    return; 
                }
                command_ = 14; break;  // out일때 우회전 + 1step

            case 15: command_ = 15; break;  // left_short

            case 16: command_ = 16; break;  // right_short

            case 17: //shoot_ready
                command_ = 17; 
                shoot_mode = true;
                break;

            case 18: command_ = 18; break; //shoot

            case 19: command_ = 19; break; //shoot_finish

            case 20: command_ = 20; break; //hurdle

            case 21: command_ = 21; break; //forward_six

            case 22: command_ = 22; pick_fail = true; break; //pick_fail(back 1step)

            case 23: command_ = 23; break; // Up_Neck

            case 24: command_ = 24; break; // Down_Neck

            case 77: command_ = 77; break; //Recovery

            // case 99: // STOP (즉시 처리 그대로 유지)
            //     RCLCPP_INFO(this->get_logger(), "[STOP] 명령 수신");
            //     callback_->Set();
            //     dxl_->MoveToTargetSmoothCos(callback_->All_Theta, 150, 10);
            //     callback_->ResetMotion();
            //     RCLCPP_INFO(this->get_logger(), "[MotionEnd] 모션 종료 메시지 전송");
            //     return;

            default:
                RCLCPP_WARN(this->get_logger(), "정의되지 않은 command=%d", msg->command);
                return;
        }

        // 모션 command 저장
        current_go_ = command_;

        if (current_go_ == 2 || current_go_ == 3 || current_go_ == 13 || current_go_ == 14)
        {
        angle_ = msg->angle;
        callback_->SetLineTurn(true);     // line_turn 키기
        callback_->OnLineResult(angle_);
        callback_->ResetMotion();        // re=0, indext=0, (index_angle=0 권장)
        }   

        motion_in_progress_ = true;
        motion_loop_timer_->reset();
    }

    // 주기적으로 각도 갱신 및 모션 종료 여부 판단
    void MotionLoop()
    {
        if (!motion_in_progress_) {
            // RCLCPP_WARN(this->get_logger(), "[MotionLoop] motion_in_progress_ == false");
            return;
        }

        RCLCPP_INFO(this->get_logger(),
        "[MotionLoop] remain turns=%d",
        callback_->GetTurnsRemaining()
        );


        callback_->SelectMotion(current_go_);
        callback_->TATA();
        callback_->Write_All_Theta();                      // 현재 프레임의 목표 각도 계산
        dxl_->SetThetaRef(callback_->All_Theta);           // 목표 각도 설정
        dxl_->syncWriteTheta();                            // 모터에 전송

        // 모션 종료 판단
        if (callback_->IsMotionFinish())
        {
            //current_go_가 2,3,13,14이면 true
            const bool is_turn = (current_go_ == 2 || current_go_ == 3 || current_go_ == 13 || current_go_ == 14);
            if (is_turn){

                int prev  = callback_->FetchSubTurnsRemaining(1);   // turn_remaing 1 감소 시키고 감소 전의 값을 prev에 저장
                int after = prev - 1; //로그 확인용

                RCLCPP_INFO(this->get_logger(),
                    "[MotionLoop] FINISH: CB.prev=%d -> after=%d @%p",
                    prev, after, callback_->GetTurnsRemainingAddr()
                );
 
                if (prev > 1) {
                    RCLCPP_INFO(this->get_logger(), "[MotionLoop] next unit turn → ResetMotion()");
                    callback_->ResetMotion();
                    return; // 타이머 유지 
                }
                else if (prev <= 0) {
                    // 언더플로 방지(혹시 seed 안 됐거나 중복 감소 시)
                    RCLCPP_WARN(this->get_logger(),
                        "[MotionLoop] UNDERFLOW: CB.turns was %d — force 0", prev);
                    callback_->SetTurnsRemaining(0);
                    // 아래로 내려가 종료 처리
                }
                // prev == 1 → 지금 마지막 턴이 막 끝남 → 종료 처리로 이동 
                
                if (current_go_ == 13 || current_go_ == 14){
                    out_turn_end = true;
                }
            }
            
            // out 상태에서 turn이후에 강제로 1step
            if (out_turn_end){
                current_go_ = 12; // 1step
                RCLCPP_INFO(this->get_logger(), "current_go_ =%d", current_go_);
                callback_->ResetMotion();
                out_turn_end = false;  
                return;
            }

            robot_msgs::msg::MotionEnd end_msg;

            // pick 모션 시퀀스
            if (pick_mode)
            {
                if (current_go_ == 10){
                    current_go_ = 11;
                    callback_->ResetMotion();
                    return;
                }

                if (current_go_ == 11 || back_count <= 3){
                    current_go_ = 5; //back 1 step
                    back_count += 1;
                    callback_->ResetMotion();
                    return;
                }
                
                // 공을 집은 후 제자리에서 회전하는 방법
                if (current_go_ == 5){
                    if(count1 == 0){
                        current_go_ = 3;
                        callback_->SetLineTurn(true);
                        callback_->OnLineResult(30);
                        callback_->ResetMotion();
                        RCLCPP_INFO(this->get_logger(), "current_go_ =%d", current_go_);
                        count1 += 1;   //count로 회전 방향 강제 시키기
                        return;
                    }
                    else if(count1 == 1){
                        current_go_ = 2;
                        callback_->SetLineTurn(true);
                        callback_->OnLineResult(30);
                        callback_->ResetMotion();
                        RCLCPP_INFO(this->get_logger(), "current_go_ =%d", current_go_);
                        count1 = 0;
                        return;
                    }
                }
                pick_mode = false;
                back_count = 0;
            }

            if (pick_fail)
            {
                if (current_go_ == 22 || back_count < 3){
                    if (current_go_ == 22){
                        current_go_ = 5; //back 1 step
                        back_count += 1;
                        callback_->ResetMotion();
                        return;
                    }

                    if (current_go_ == 5){
                        if(count1 == 0){
                            current_go_ = 3;
                            callback_->SetLineTurn(true);
                            callback_->OnLineResult(30);
                            callback_->ResetMotion();
                            RCLCPP_INFO(this->get_logger(), "current_go_ =%d", current_go_);
                            count1 += 1;   //count로 회전 방향 강제 시키기
                            return;
                        }
                        else if(count1 == 1){
                            current_go_ = 2;
                            callback_->SetLineTurn(true);
                            callback_->OnLineResult(30);
                            callback_->ResetMotion();
                            RCLCPP_INFO(this->get_logger(), "current_go_ =%d", current_go_);
                            count1 = 0;
                            return;
                        }
                    }
                pick_fail = false;
                back_count = 0;
                count2 += 1; // 못 집었을때 이제 골대를 안 볼 계획이므로 count2 += 1
                }
            }

            // shoot 모션 시퀀스 
            if (shoot_mode)
            {
                if (current_go_ == 17){
                    current_go_ = 18;
                    RCLCPP_INFO(this->get_logger(), "current_go_ =%d", current_go_);
                    callback_->ResetMotion(); 
                    return;
                }

                if (current_go_ == 18){
                    current_go_ = 19; 
                    callback_->ResetMotion();   
                    return;
                }

                // 공을 던진 후 제자리에서 회전하는 방법
                if (current_go_ == 19){
                    if(count2 == 0){
                        current_go_ = 3; //우회전
                        callback_->SetLineTurn(true);
                        callback_->OnLineResult(30);
                        callback_->ResetMotion();
                        RCLCPP_INFO(this->get_logger(), "current_go_ =%d", current_go_);
                        count2 += 1;   //count로 회전 방향 강제 시키기
                        return;
                    }
                    else if(count2 == 1){
                        current_go_ = 2; //좌회전
                        callback_->SetLineTurn(true);
                        callback_->OnLineResult(30);
                        callback_->ResetMotion();
                        RCLCPP_INFO(this->get_logger(), "current_go_ =%d", current_go_);
                        count2 = 0;
                        return;
                    }
                shoot_mode = false;
                }
            }
            

            end_msg.motion_end_detect = true;
            motion_end_pub_->publish(end_msg);
            RCLCPP_INFO(this->get_logger(),
                    "motion_end publish = %d", end_msg.motion_end_detect);

            callback_->SetTurnsRemaining(0);   //turn_remaing을 0으로 초기화
            motion_in_progress_ = false;            // 상태 초기화             
            motion_loop_timer_->cancel();                  // 타이머 중지
            current_go_ = 0;
        }
    }

    // 구성 요소들 (shared_ptr로 메모리 관리)
    std::shared_ptr<Dxl> dxl_;
    std::shared_ptr<Trajectory> trajectory_;
    std::shared_ptr<IK_Function> ik_;
    std::shared_ptr<Pick> pick_;
    std::shared_ptr<Dxl_Controller> dxl_ctrl_;
    std::shared_ptr<Callback> callback_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                            // ROS 2 초기화
    rclcpp::spin(std::make_shared<MainNode>());          // 메인 노드 실행 (타이머 포함)
    rclcpp::shutdown();                                  // 종료 처리
    return 0;
}





