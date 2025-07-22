import rclpy

from robot_msgs.msg import MotionCommand, LineResult, FallResult, MotionEnd
from rclpy.node import Node

class Motion:
    FORWARD = 1 
    TURN_LEFT = 2
    TURN_RIGHT = 3
    STOP = 4

class MotionDecision(Node):
    def __init__(self):
        super().__init__('line_result_subscribe')

        #초기값 설정
        self.fall_detect = False
        self.motion_end_detect = False
        self.res = 0
        self.first_motion_triggered = False

        #라인 결과 subscribe
        self.line_result_subscribe = self.create_subscription( 
            LineResult,
            '/line_result', 
            self.LineResultCallback,
            10
        )

        #fall 상태 subscribe
        self.fall_subscribe = self.create_subscription(FallResult, '/fall_result', self.FallResultCallback, 10)

        #원하는 motion publish
        self.motion_publish = self.create_publisher(MotionCommand, '/motion_command', 10)

        #motion 종료 subscribe
        self.motion_end = self.create_subscription(MotionEnd, '/motion_end', self.MotionEndCallback, 10)

        # 최초 1회만 실행되는 타이머 등록 (1초 뒤 실행), 강제로 motion_end_detect를 True로 변환
        self.initial_timer = self.create_timer(1.0, self.InitialTrigger)

    def InitialTrigger(self):
        if self.res != 0:
            self.motion_end_detect = True
            self.get_logger().info("최초 모션 시작 허용됨 → 라인 감지 완료")
            self.MotionResult()
            self.initial_timer.cancel()  # 타이머 종료
        else:
            self.get_logger().info("라인 미감지 → 모션 시작 대기 중")


    def FallResultCallback(self, fall_msg: FallResult):
        self.get_logger().info(f"[FallResult] yaw: {fall_msg.prev_yaw_deg}, state: {fall_msg.fall_detect}")
        self.fall_detect = fall_msg.fall_detect
        self.MotionResult()

    def LineResultCallback(self, line_msg: LineResult):
        self.get_logger().info(f"[LineResult] res: {line_msg.res}, angle: {line_msg.angle}")
        self.res = line_msg.res

        if not self.first_motion_triggered and self.res != 0:
            self.motion_end_detect = True
            self.first_motion_triggered = True
            self.get_logger().info("최초 모션 시작 허용됨 (res 감지됨)")


        self.MotionResult()

    def MotionEndCallback(self, motion_end_msg: MotionEnd):
        self.get_logger().info(f"[MotionEnd] motion_end: {motion_end_msg.motion_end_detect}")
        self.motion_end_detect = motion_end_msg.motion_end_detect
        self.MotionResult()

    # subscribe한 fall_msg와 line_msg를 이용한 motion 결정
    def MotionResult(self):
        self.get_logger().info(f"[MotionResult] res: {self.res}, fall_detect: {self.fall_detect}, motion_end_detect: {self.motion_end_detect}")
        
        if not self.motion_end_detect:
            self.get_logger().info("아직 모션 미완료")
            return

        motion_msg = MotionCommand()

  
        if self.fall_detect:
            motion_msg.command = Motion.STOP
            motion_msg.detail = "Test: STOP"

        elif self.res == 1:
            motion_msg.command = Motion.FORWARD
            motion_msg.detail = "Test: Moving forward"

        elif self.res == 2:
            motion_msg.command = Motion.TURN_LEFT
            motion_msg.detail = "Test: Moving Turn Left"
        
        elif self.res == 3:
            motion_msg.command = Motion.TURN_RIGHT
            motion_msg.detail = "Test: Moving Turn Right"

        # 명령 보내면 다시 false로 변환
        self.motion_end_detect = False          

        self.motion_publish.publish(motion_msg)
        self.get_logger().info(f"Published motion command: {motion_msg.detail}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
