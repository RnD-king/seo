#include "rclcpp/rclcpp.hpp"                 // ROS 2 기본 헤더
#include "dynamixel.hpp"                     // 사용자 정의: 다이나믹셀 제어 클래스
#include "callback.hpp"                      // 사용자 정의: 콜백 연산 모듈
#include "dynamixel_controller.hpp"          // 사용자 정의: θ 제어 인터페이스
#include "BRP_Kinematics.hpp"
#include "NewPattern2.hpp"     // 사용자 정의: 보행 궤적 생성기
#include "robot_msgs/msg/motion_command.hpp"  //motion command 불러오기
#include "robot_msgs/msg/motion_end.hpp"  //motion end 불러오기


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

        subscription_ = this->create_subscription<robot_msgs::msg::MotionCommand>(
            "/motion_command", 10, std::bind(&MainNode::MotionCallback, this, std::placeholders::_1));

        
        //motion_end를 publish
        motion_end_pub_ = this->create_publisher<robot_msgs::msg::MotionEnd>("/motion_end", 10);


        // 100Hz 루프 타이머 생성 (처음엔 중단 상태)
        motion_loop_timer_ = this->create_wall_timer(
            10ms, std::bind(&MainNode::MotionLoop, this));
        motion_loop_timer_->cancel();  // 초기에는 멈춰있음

    }

private:


    bool motion_in_progress_;                              // 현재 모션 실행 중인지 여부
    rclcpp::TimerBase::SharedPtr motion_loop_timer_;       // 반복 제어용 타이머\

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
    rclcpp::TimerBase::SharedPtr start_timer_;

    bool ball_catch_ = false;  // 공 잡았는지 확인용 


    void MotionCallback(const robot_msgs::msg::MotionCommand::SharedPtr msg)        
    {
        if (msg->command == 1) //직진
        {
            if (motion_in_progress_) {
                RCLCPP_WARN(this->get_logger(), "동작 중: 명령 무시됨");
                return;
            }
            int go = 1;
            callback_->SelectMotion(go);
            motion_in_progress_ = true;
            motion_loop_timer_->reset();
        }

        else if (msg->command == 2) //우회전
        {
            if (motion_in_progress_) {
                RCLCPP_WARN(this->get_logger(), "동작 중: 명령 무시됨");
                return;
            }
            int go = 2;
            callback_->SelectMotion(go);
            motion_in_progress_ = true;
            motion_loop_timer_->reset();
        }

        else if (msg->command == 3) //좌회전
        {
            if (motion_in_progress_) {
                RCLCPP_WARN(this->get_logger(), "동작 중: 명령 무시됨");
                return;
            }
            int go = 3;
            callback_->SelectMotion(go);
            motion_in_progress_ = true;
            motion_loop_timer_->reset();
        }

        // else if (msg->command == 4) //징검다리
        // {
        //     if (motion_in_progress_) {
        //         RCLCPP_WARN(this->get_logger(), "동작 중: 명령 무시됨");
        //         return;
        //     }
        //     int go = 4;

        //     motion_in_progress_ = true;
        //     motion_loop_timer_->reset();
        // }


        else if (msg->command == 5) //Pick
        {
            if (motion_in_progress_) {
                RCLCPP_WARN(this->get_logger(), "동작 중: 명령 무시됨");
                return;
            }

            if (ball_catch_) { // 이미 공 들고 있으면 Pick 금지
                RCLCPP_WARN(this->get_logger(), "[PICK] 이미 공 보유(ball_catch_=true) → 무시");
                return;
            }

            int go = 5;
            callback_->SelectMotion(go);
            ball_catch_ = true;
            motion_in_progress_ = true;
            motion_loop_timer_->reset();
        }

        else if (msg->command == 6) //Hurdle
        {
            if (motion_in_progress_) {
                RCLCPP_WARN(this->get_logger(), "동작 중: 명령 무시됨");
                return;
            }
            int go = 6;
            callback_->SelectMotion(go);
            motion_in_progress_ = true;
            motion_loop_timer_->reset();
        }

        else if (msg->command == 7) //Shoot
        {
            if (motion_in_progress_) {
                RCLCPP_WARN(this->get_logger(), "동작 중: 명령 무시됨");
                return;
            }

            if (!ball_catch_) { // 공 없으면 슛 금지
                RCLCPP_WARN(this->get_logger(), "[SHOOT] 공 없음(ball_catch_=false) → 무시");
                return;
            }

            int go = 7;
            callback_->SelectMotion(go);
            ball_catch_ = false;
            motion_in_progress_ = true;
            motion_loop_timer_->reset();
        }

        else if (msg->command == 77) //RECOVERY 
        {
            RCLCPP_INFO(this->get_logger(), "[RECOVERY] 명령 수신");

            motion_in_progress_ = false;
            motion_loop_timer_->cancel();

            int go = 77;
            callback_->SelectMotion(go);
            callback_->Set();
            dxl_->MoveToTargetSmoothCos(callback_->All_Theta, 150, 10);

            //recovery 모션 함수
            //RecoveryMotion();
        }

        else if (msg->command == 99) //STOP
        {
            RCLCPP_INFO(this->get_logger(), "[STOP] 명령 수신");

            motion_in_progress_ = false;
            motion_loop_timer_->cancel();
        
            callback_->Set();
            dxl_->MoveToTargetSmoothCos(callback_->All_Theta, 150, 10);

            //경로 초기화
            callback_->ResetMotion();

            robot_msgs::msg::MotionEnd motion_end_msg;
            motion_end_msg.motion_end_detect = true;

            motion_end_pub_->publish(motion_end_msg);      // /motion_end 퍼블리시
            RCLCPP_INFO(this->get_logger(), "[MotionEnd] 모션 종료 메시지 전송");
        }


        else
        {
            RCLCPP_WARN(this->get_logger(), "정의되지 않은 command=%d", msg->command);
        }
    }

    // 주기적으로 각도 갱신 및 모션 종료 여부 판단
    void MotionLoop()
    {
        // RCLCPP_INFO(this->get_logger(), "[MotionLoop] called");

        if (!motion_in_progress_) {
            // RCLCPP_WARN(this->get_logger(), "[MotionLoop] motion_in_progress_ == false");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[MotionLoop] before Write_All_Theta()");
        callback_->Write_All_Theta();                      // 현재 프레임의 목표 각도 계산
        dxl_->SetThetaRef(callback_->All_Theta);           // 목표 각도 설정
        dxl_->syncWriteTheta();                            // 모터에 전송

        // 모션 종료 판단
        if (callback_->IsMotionFinish())
        {

            RCLCPP_INFO(this->get_logger(), "[MotionLoop] Motion finished");
            robot_msgs::msg::MotionEnd motion_end_msg;
            motion_end_msg.motion_end_detect = true;

            motion_end_pub_->publish(motion_end_msg);      // /motion_end 퍼블리시
            RCLCPP_INFO(this->get_logger(), "[MotionEnd] 모션 종료 메시지 전송");

            int go = 0;
            motion_in_progress_ = false;                   // 상태 초기화
            motion_loop_timer_->cancel();                  // 타이머 중지
        }
    }

    // 구성 요소들 (shared_ptr로 메모리 관리)
    std::shared_ptr<Dxl> dxl_;
    std::shared_ptr<Trajectory> trajectory_;
    std::shared_ptr<IK_Function> ik_;
    std::shared_ptr<Pick> pick_;
    std::shared_ptr<Dxl_Controller> dxl_ctrl_;
    std::shared_ptr<Callback> callback_;


    // motioncommand subscribe
    rclcpp::Subscription<robot_msgs::msg::MotionCommand>::SharedPtr subscription_;
    // motion_end publisher
    rclcpp::Publisher<robot_msgs::msg::MotionEnd>::SharedPtr motion_end_pub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                            // ROS 2 초기화
    rclcpp::spin(std::make_shared<MainNode>());          // 메인 노드 실행 (타이머 포함)
    rclcpp::shutdown();                                  // 종료 처리
    return 0;
}





