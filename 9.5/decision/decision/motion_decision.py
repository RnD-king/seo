import rclpy

from robot_msgs.msg import MotionCommand, LineResult, FallResult
from rclpy.node import Node

class Motion:
    FORWARD_FOUR = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    BACK = 4
    BACK_HALF = 5
    FORWARD_HALF = 6
    FORWARD_LEFT = 7
    FORWARD_RIGHT = 8
    PICK = 9
    SHOOT = 10
    HURDLE = 11
    FORWARD_ONE = 12
    Out_Right = 13
    Out_Left = 14
    RECOVERY = 77
    STOP = 99

class MotionDecision(Node):
    def __init__(self):
        super().__init__('line_result_subscribe')

        #초기값 설정
        self.fall_detect = False
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


    def FallResultCallback(self, fall_msg: FallResult):
        self.get_logger().info(f"[FallResult] yaw: {fall_msg.prev_yaw_deg}, state: {fall_msg.fall_detect}")
        self.fall_detect = fall_msg.fall_detect
        self.MotionResult()

    def LineResultCallback(self, line_msg: LineResult):
        self.get_logger().info(f"[LineResult] res: {line_msg.res}, angle: {line_msg.angle}")
        self.res = line_msg.res
        self.angle = line_msg.angle
        self.MotionResult()



    # subscribe한 fall_msg와 line_msg를 이용한 motion 결정
    def MotionResult(self):
        self.get_logger().info(f"[MotionResult] res: {self.res}, fall_detect: {self.fall_detect}")
        
        motion_msg = MotionCommand()

  
        if self.fall_detect:
            motion_msg.command = Motion.STOP

        elif self.res == 1:
            motion_msg.command = Motion.FORWARD_FOUR

        elif self.res == 2:
            motion_msg.command = Motion.TURN_LEFT
        
        elif self.res == 3:
            motion_msg.command = Motion.TURN_RIGHT
        
        elif self.res == 4:   
            motion_msg.command = Motion.BACK

        elif self.res == 5:   
            motion_msg.command = Motion.BACK_HALF

        elif self.res == 6:   
            motion_msg.command = Motion.FORWARD_HALF

        elif self.res == 7:
            motion_msg.command = Motion.FORWARD_LEFT

        elif self.res == 8:
            motion_msg.command = Motion.FORWARD_RIGHT

        elif self.res == 9:
            motion_msg.command = Motion.PICK

        elif self.res == 10:
            motion_msg.command = Motion.SHOOT

        elif self.res == 11:
            motion_msg.command = Motion.HURDLE

        elif self.res == 12:
            motion_msg.command = Motion.FORWARD_ONE

        elif self.res == 13:
            motion_msg.command = Motion.Out_Right # 왼쪽으로 돌고 1step
            
        elif self.res == 14:
            motion_msg.command = Motion.Out_Left # 오른쪽으로 돌고 1step

        elif self.res == 77:
            motion_msg.command = Motion.RECOVERY

        elif self.res == 99:
            motion_msg.command = Motion.STOP
       

        self.motion_publish.publish(motion_msg)
        self.get_logger().info(f"Published motion command: {motion_msg.command}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
