import rclpy

from robot_msgs.msg import MotionCommand, LineResult, FallResult, BallResult, MotionEnd
from rclpy.node import Node

class Motion:
    Forward_Four = 1
    Turn_Left = 2
    Turn_Right = 3
    Back = 4
    Back_Half = 5
    Forward_Half = 6
    Forward_Left = 7
    Forward_Right = 8
    Pick = 9
    Shoot = 10
    Hurdle = 11
    Forward_One = 12
    Out_Right = 13
    Out_Left = 14
    Re_Catch = 15
    Recovery = 77
    Stop = 99

class MotionDecision(Node):
    def __init__(self):
        super().__init__('line_result_subscribe')

        #초기값 설정
        self.fall_detect = False
        self.ball_detect = False
        self.pick_end = False
        self.res = 0

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

        # ball 관련 subscribe
        self.ball_subscribe = self.create_subscription(BallResult, '/ball_result', self.BallCallback, 10)

        # motion_end subscibe
        self.motion_end_sub = self.create_subscription(MotionEnd, '/motion_end', self.MotionEndCallback, 10)

    # pick motion이 끝나면 pick_end를 받아오기
    def MotionEndCallback(self, end_msg: MotionEnd):
        self.pick_end = end_msg.pick_end
        self.get_logger().info(f"pick_end: {end_msg.pick_end}")
        if(self.pick_end):
            self.MotionResult()

    def BallCallback(self, Ball_msg: BallResult):
        self.ball_detect = Ball_msg.ball_detect
        self.get_logger().info(f"ball_detect: {Ball_msg.ball_detect}")
        self.res = Ball_msg.res
        self.angle = Ball_msg.angle
        self.MotionResult()

    def FallResultCallback(self, fall_msg: FallResult):
        self.get_logger().info(f"[FallResult] yaw: {fall_msg.prev_yaw_deg}, state: {fall_msg.fall_detect}")
        self.fall_detect = fall_msg.fall_detect
        self.MotionResult()

    def LineResultCallback(self, line_msg: LineResult):
        # ball을 감지하면 라인 안 받아오게
        if(self.ball_detect):
            self.get_logger().info(f"ball 감지함")
            return
        
        self.get_logger().info(f"[LineResult] res: {line_msg.res}, angle: {line_msg.angle}")
        self.res = line_msg.res
        self.angle = line_msg.angle
        self.MotionResult()
        



    # subscribe한 fall_msg와 line_msg를 이용한 motion 결정
    def MotionResult(self):
        self.get_logger().info(f"[MotionResult] res: {self.res}, fall_detect: {self.fall_detect}, angle: {self.angle}")
        
        motion_msg = MotionCommand()

        # pick이 끝나면 다시 라인 보게 하기
        if (self.pick_end):
            self.ball_detect = False
            return
  
        if self.fall_detect:
            motion_msg.command = Motion.Stop

        elif self.res == 1:
            motion_msg.command = Motion.Forward_Four

        elif self.res == 2:
            motion_msg.command = Motion.Turn_Left
        
        elif self.res == 3:
            motion_msg.command = Motion.Turn_Right
        
        elif self.res == 4:   
            motion_msg.command = Motion.Back

        elif self.res == 5:   
            motion_msg.command = Motion.Back_Half

        elif self.res == 6:   
            motion_msg.command = Motion.Forward_Half

        elif self.res == 7:
            motion_msg.command = Motion.Forward_Left

        elif self.res == 8:
            motion_msg.command = Motion.Forward_Right

        elif self.res == 9:
            motion_msg.command = Motion.Pick

        elif self.res == 10:
            motion_msg.command = Motion.Shoot

        elif self.res == 11:
            motion_msg.command = Motion.Hurdle

        elif self.res == 12:
            motion_msg.command = Motion.Forward_One

        elif self.res == 13:
            motion_msg.command = Motion.Out_Right # 왼쪽으로 돌고 1step
            
        elif self.res == 14:
            motion_msg.command = Motion.Out_Left # 오른쪽으로 돌고 1step

        elif self.res == 15:
            motion_msg.command = Motion.Re_Catch # 공 다시 잡기
        
        elif self.res == 77:
            motion_msg.command = Motion.Recovery

        elif self.res == 99:
            motion_msg.command = Motion.Stop
        
        motion_msg.angle = self.angle

    
        self.motion_publish.publish(motion_msg)
        self.get_logger().info(f"Published motion command: {motion_msg.command}")


def main(args=None):
    rclpy.init(args=args)
    node = MotionDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
